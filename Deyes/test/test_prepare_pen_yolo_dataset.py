"""ROS-free tests for the external, batch-disjoint YOLO dataset builder."""

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools" / "prepare_pen_yolo_dataset.py"
SPEC = importlib.util.spec_from_file_location("prepare_pen_yolo_dataset", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


def write_batch(root: Path, session: str, image_count: int) -> None:
    images = root / session / "images"
    images.mkdir(parents=True)
    for index in range(image_count):
        (images / f"image_{index:03d}.jpg").write_bytes(b"jpeg")


def inventory(records: list[dict]) -> dict:
    return {"schema_version": 1, "records": records}


def record(batch_id: str, session: str, count: int) -> dict:
    return {"batch_id": batch_id, "session_path": session, "image_count": count, "status": "eligible_pending_annotation"}


def test_prepare_copies_only_complete_batch_disjoint_sources(tmp_path: Path) -> None:
    dataset_root = tmp_path / "pen"
    records = [record("a", "session_a", 1), record("b", "session_b", 1), record("c", "session_c", 1)]
    for item in records:
        write_batch(dataset_root, item["session_path"], 1)
    inventory_path = dataset_root / "dataset_inventory.json"
    inventory_path.write_text(json.dumps(inventory(records)), encoding="utf-8")
    annotation_root = dataset_root / "annotation_v1"
    for item in records:
        labels = annotation_root / item["batch_id"]
        labels.mkdir(parents=True)
        (labels / "image_000.txt").write_text("" if item["batch_id"] == "c" else "0 0.5 0.5 0.2 0.1\n", encoding="utf-8")
    plan = dataset_root / "split_plan.json"
    plan.write_text(json.dumps({"assignments": {"a": "train", "b": "val", "c": "test"}}), encoding="utf-8")

    manifest = module.prepare_dataset(dataset_root=dataset_root, inventory_path=inventory_path, annotation_root=annotation_root, split_plan_path=plan, output_root=dataset_root / "yolo_v1")

    assert manifest["image_counts"] == {"test": 1, "train": 1, "val": 1}
    assert (dataset_root / "yolo_v1" / "labels" / "test" / "c__image_000.txt").read_text() == ""
    assert {row["batch_id"] for row in manifest["records"]} == {"a", "b", "c"}


def test_prepare_rejects_missing_label_or_batch_split_leakage(tmp_path: Path) -> None:
    dataset_root = tmp_path / "pen"
    records = [record("a", "session_a", 1), record("b", "session_b", 1)]
    for item in records:
        write_batch(dataset_root, item["session_path"], 1)
    inventory_path = dataset_root / "inventory.json"
    inventory_path.write_text(json.dumps(inventory(records)), encoding="utf-8")
    annotation_root = dataset_root / "labels"
    annotation_root.mkdir()
    plan = dataset_root / "split.json"
    plan.write_text(json.dumps({"assignments": {"a": "train", "b": "train"}}), encoding="utf-8")

    with pytest.raises(module.DatasetPreparationError, match="nonempty_train_val_and_test"):
        module.prepare_dataset(dataset_root=dataset_root, inventory_path=inventory_path, annotation_root=annotation_root, split_plan_path=plan, output_root=dataset_root / "out")
