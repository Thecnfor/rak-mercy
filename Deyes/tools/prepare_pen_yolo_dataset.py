#!/usr/bin/env python3
"""Build an auditable, batch-disjoint YOLOv5 pen dataset outside the repository.

The tool never edits source captures or annotations.  It copies only after every
eligible batch has one explicit split and every source image has a valid label.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from pathlib import Path
from typing import Any, Iterable, Mapping


class DatasetPreparationError(ValueError):
    """Raised before any output is created when source evidence is incomplete."""


SPLITS = frozenset({"train", "val", "test"})


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DatasetPreparationError(f"invalid_json:{path}:{exc}") from exc
    if not isinstance(payload, dict):
        raise DatasetPreparationError(f"json_root_must_be_an_object:{path}")
    return payload


def validate_label(path: Path) -> None:
    """Require one class-0 YOLO label per line; zero-byte labels are valid negatives."""
    if not path.is_file():
        raise DatasetPreparationError(f"missing_label:{path}")
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        fields = line.split()
        if len(fields) != 5 or fields[0] != "0":
            raise DatasetPreparationError(f"invalid_yolo_label:{path}:{number}")
        try:
            values = [float(value) for value in fields[1:]]
        except ValueError as exc:
            raise DatasetPreparationError(f"invalid_yolo_label:{path}:{number}") from exc
        if not all(0.0 <= value <= 1.0 for value in values):
            raise DatasetPreparationError(f"normalized_yolo_value_out_of_range:{path}:{number}")
        if values[2] <= 0.0 or values[3] <= 0.0:
            raise DatasetPreparationError(f"nonpositive_yolo_box:{path}:{number}")


def eligible_records(inventory: Mapping[str, Any]) -> list[dict[str, Any]]:
    records = inventory.get("records")
    if not isinstance(records, list):
        raise DatasetPreparationError("inventory_records_must_be_a_list")
    selected: list[dict[str, Any]] = []
    for record in records:
        if not isinstance(record, dict) or record.get("status") != "eligible_pending_annotation":
            continue
        if not isinstance(record.get("batch_id"), str) or not isinstance(record.get("session_path"), str):
            raise DatasetPreparationError("eligible_record_requires_batch_id_and_session_path")
        selected.append(record)
    if not selected:
        raise DatasetPreparationError("inventory_has_no_eligible_batches")
    return selected


def validate_split_plan(records: Iterable[Mapping[str, Any]], plan: Mapping[str, Any]) -> dict[str, str]:
    assignments = plan.get("assignments")
    if not isinstance(assignments, dict):
        raise DatasetPreparationError("split_plan_requires_assignments_object")
    batch_ids = {str(record["batch_id"]) for record in records}
    if set(assignments) != batch_ids:
        missing = sorted(batch_ids - set(assignments))
        extra = sorted(set(assignments) - batch_ids)
        raise DatasetPreparationError(f"split_plan_batch_mismatch:missing={missing}:extra={extra}")
    normalized: dict[str, str] = {}
    for batch_id, split in assignments.items():
        if split not in SPLITS:
            raise DatasetPreparationError(f"invalid_split:{batch_id}:{split}")
        normalized[str(batch_id)] = str(split)
    if set(normalized.values()) != SPLITS:
        raise DatasetPreparationError("split_plan_requires_nonempty_train_val_and_test")
    return normalized


def prepare_dataset(
    *, dataset_root: Path, inventory_path: Path, annotation_root: Path, split_plan_path: Path, output_root: Path
) -> dict[str, Any]:
    """Copy a validated external dataset into a fresh YOLO directory and manifest."""
    dataset_root = dataset_root.resolve()
    output_root = output_root.resolve()
    try:
        output_root.relative_to(dataset_root)
    except ValueError as exc:
        raise DatasetPreparationError("output_root_must_be_inside_external_dataset_root") from exc
    if output_root.exists():
        raise DatasetPreparationError(f"output_root_already_exists:{output_root}")

    inventory = load_json(inventory_path)
    records = eligible_records(inventory)
    assignments = validate_split_plan(records, load_json(split_plan_path))
    planned: list[tuple[dict[str, Any], str, Path, Path]] = []
    for record in records:
        batch_id = str(record["batch_id"])
        session = dataset_root / str(record["session_path"])
        images = sorted((session / "images").glob("*.jpg"))
        expected_count = int(record.get("image_count", -1))
        if len(images) != expected_count:
            raise DatasetPreparationError(f"source_image_count_mismatch:{batch_id}:{len(images)}!={expected_count}")
        for image in images:
            label = annotation_root / batch_id / f"{image.stem}.txt"
            validate_label(label)
            planned.append((record, assignments[batch_id], image, label))

    output_root.mkdir(parents=True)
    manifest_rows: list[dict[str, Any]] = []
    counts = {split: 0 for split in sorted(SPLITS)}
    for record, split, image, label in planned:
        batch_id = str(record["batch_id"])
        filename = f"{batch_id}__{image.name}"
        image_dest = output_root / "images" / split / filename
        label_dest = output_root / "labels" / split / f"{Path(filename).stem}.txt"
        image_dest.parent.mkdir(parents=True, exist_ok=True)
        label_dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(image, image_dest)
        shutil.copy2(label, label_dest)
        counts[split] += 1
        manifest_rows.append({
            "batch_id": batch_id,
            "split": split,
            "source_image": image.relative_to(dataset_root).as_posix(),
            "source_label": label.relative_to(annotation_root).as_posix(),
            "image": image_dest.relative_to(output_root).as_posix(),
            "label": label_dest.relative_to(output_root).as_posix(),
            "image_sha256": sha256_file(image),
            "label_sha256": sha256_file(label),
        })
    data_yaml = "\n".join([
        f"path: {output_root.as_posix()}",
        "train: images/train",
        "val: images/val",
        "test: images/test",
        "names: [pen]",
        "",
    ])
    (output_root / "data.yaml").write_text(data_yaml, encoding="utf-8")
    manifest = {
        "schema_version": 1,
        "inventory": str(inventory_path),
        "inventory_sha256": sha256_file(inventory_path),
        "split_plan": str(split_plan_path),
        "split_plan_sha256": sha256_file(split_plan_path),
        "annotation_root": str(annotation_root),
        "class_names": {"0": "pen"},
        "image_counts": counts,
        "records": manifest_rows,
    }
    (output_root / "dataset_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description="Prepare a batch-disjoint single-class YOLOv5 pen dataset.")
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--inventory", type=Path, required=True)
    parser.add_argument("--annotation-root", type=Path, required=True)
    parser.add_argument("--split-plan", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    args = parser.parse_args()
    manifest = prepare_dataset(
        dataset_root=args.dataset_root,
        inventory_path=args.inventory,
        annotation_root=args.annotation_root,
        split_plan_path=args.split_plan,
        output_root=args.output_root,
    )
    print(json.dumps({"image_counts": manifest["image_counts"], "output_root": str(args.output_root)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
