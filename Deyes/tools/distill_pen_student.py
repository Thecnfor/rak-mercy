#!/usr/bin/env python3
"""Build a teacher-labelled YOLOv5 dataset for the small Deyes pen student.

Human-labelled train/val/test data remains authoritative.  The frozen teacher
may add only elongated, high-confidence detections from otherwise-unlabelled
frames to the train split.  Teacher misses are never converted into negative
labels.  Every added image, box, confidence and model hash is recorded.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
from pathlib import Path
from typing import Any, Iterable


IMAGE_SUFFIXES = frozenset({".jpg", ".jpeg", ".png", ".bmp", ".ppm"})
EXCLUDED_SOURCE_PARTS = frozenset({"qa", "annotation_v1", "annotation_v2", "annotation_final", "distillation"})


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def elongated_box(xyxy: Iterable[float], *, min_aspect: float, max_area_fraction: float,
                  image_width: int, image_height: int) -> bool:
    x1, y1, x2, y2 = (float(value) for value in xyxy)
    width, height = max(0.0, x2 - x1), max(0.0, y2 - y1)
    if width <= 1.0 or height <= 1.0:
        return False
    aspect = max(width, height) / min(width, height)
    area_fraction = (width * height) / float(image_width * image_height)
    return aspect >= min_aspect and area_fraction <= max_area_fraction


def yolo_box(xyxy: Iterable[float], *, image_width: int, image_height: int) -> tuple[float, ...]:
    x1, y1, x2, y2 = (float(value) for value in xyxy)
    return ((x1 + x2) / (2.0 * image_width), (y1 + y2) / (2.0 * image_height),
            (x2 - x1) / image_width, (y2 - y1) / image_height)


def resolve_dataset_root(data_file: Path, document: dict[str, Any]) -> Path:
    configured = Path(str(document.get("path", ".")))
    return configured if configured.is_absolute() else (data_file.parent / configured).resolve()


def copy_authoritative_dataset(data_file: Path, output: Path) -> tuple[dict[str, Any], set[str]]:
    import yaml

    document = yaml.safe_load(data_file.read_text(encoding="utf-8")) or {}
    if int(document.get("nc", 0)) != 1:
        raise ValueError("pen_distillation_requires_one_class")
    source_root = resolve_dataset_root(data_file, document)
    hashes: set[str] = set()
    counts: dict[str, int] = {}
    for split in ("train", "val", "test"):
        image_dir = source_root / str(document[split])
        label_dir = source_root / str(document[split]).replace("images", "labels", 1)
        destination_images = output / "images" / split
        destination_labels = output / "labels" / split
        destination_images.mkdir(parents=True, exist_ok=True)
        destination_labels.mkdir(parents=True, exist_ok=True)
        count = 0
        for image in sorted(path for path in image_dir.iterdir() if path.suffix.lower() in IMAGE_SUFFIXES):
            label = label_dir / f"{image.stem}.txt"
            if not label.is_file():
                raise ValueError(f"authoritative_label_missing:{label}")
            digest = sha256_file(image)
            hashes.add(digest)
            shutil.copy2(image, destination_images / image.name)
            shutil.copy2(label, destination_labels / label.name)
            count += 1
        counts[split] = count
    return counts, hashes


def load_teacher(yolov5_root: Path, weights: Path, device_name: str):
    sys.path.insert(0, str(yolov5_root))
    from models.common import DetectMultiBackend
    from utils.torch_utils import select_device

    device = select_device(device_name)
    model = DetectMultiBackend(str(weights), device=device, dnn=False, fp16=device.type != "cpu")
    return model, device


def infer_boxes(model, device, image, *, image_size: int, confidence: float, iou: float):
    import numpy as np
    import torch
    from utils.augmentations import letterbox
    from utils.general import non_max_suppression, scale_boxes

    prepared = letterbox(image, (image_size, image_size), stride=int(model.stride), auto=False)[0]
    tensor = np.ascontiguousarray(prepared.transpose((2, 0, 1))[::-1])
    tensor = torch.from_numpy(tensor).to(device)
    tensor = tensor.half() if model.fp16 else tensor.float()
    tensor /= 255.0
    prediction = model(tensor.unsqueeze(0))
    detections = non_max_suppression(prediction, confidence, iou, classes=[0], max_det=4)[0]
    if not len(detections):
        return []
    detections[:, :4] = scale_boxes(tensor.shape[1:], detections[:, :4], image.shape).round()
    return detections.detach().cpu().tolist()


def build(args: argparse.Namespace) -> dict[str, Any]:
    import cv2
    import yaml

    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"output_must_be_empty:{output}")
    output.mkdir(parents=True, exist_ok=True)
    authoritative, known_hashes = copy_authoritative_dataset(args.data.resolve(), output)
    model, device = load_teacher(args.yolov5_root.resolve(), args.teacher.resolve(), args.device)
    records: list[dict[str, Any]] = []
    candidates = sorted({path.resolve() for root in args.unlabelled for path in root.rglob("*")
                         if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
                         and path.parent.name.lower() == "images"
                         and not EXCLUDED_SOURCE_PARTS.intersection(part.lower() for part in path.parts)})
    for image_path in candidates:
        digest = sha256_file(image_path)
        if digest in known_hashes:
            continue
        known_hashes.add(digest)
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            records.append({"source": str(image_path), "status": "unreadable"})
            continue
        raw = infer_boxes(model, device, image, image_size=args.image_size,
                          confidence=args.confidence, iou=args.iou)
        accepted = [row for row in raw if elongated_box(
            row[:4], min_aspect=args.min_aspect, max_area_fraction=args.max_area_fraction,
            image_width=image.shape[1], image_height=image.shape[0])]
        if len(accepted) != args.expected_target_count:
            records.append({"source": str(image_path), "sha256": digest, "status": "teacher_rejected",
                            "accepted_box_count": len(accepted)})
            continue
        name = f"pseudo_{digest[:16]}{image_path.suffix.lower()}"
        destination = output / "images" / "train" / name
        shutil.copy2(image_path, destination)
        label_path = output / "labels" / "train" / f"{Path(name).stem}.txt"
        lines, boxes = [], []
        for row in accepted:
            box = yolo_box(row[:4], image_width=image.shape[1], image_height=image.shape[0])
            lines.append("0 " + " ".join(f"{value:.8f}" for value in box))
            boxes.append({"xyxy": row[:4], "confidence": float(row[4]), "class_id": int(row[5])})
        label_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        records.append({"source": str(image_path), "sha256": digest, "status": "teacher_accepted",
                        "output": str(destination), "boxes": boxes})
    dataset = {"path": str(output), "train": "images/train", "val": "images/val",
               "test": "images/test", "nc": 1, "names": ["pen"]}
    (output / "data.yaml").write_text(yaml.safe_dump(dataset, sort_keys=False), encoding="utf-8")
    manifest = {"schema_version": 1, "method": "teacher_pseudo_label_distillation",
                "teacher": str(args.teacher.resolve()), "teacher_sha256": sha256_file(args.teacher),
                "image_size": args.image_size, "confidence": args.confidence, "iou": args.iou,
                "min_aspect": args.min_aspect, "max_area_fraction": args.max_area_fraction,
                "expected_target_count": args.expected_target_count,
                "authoritative_counts": authoritative,
                "pseudo_accepted_images": sum(row["status"] == "teacher_accepted" for row in records),
                "records": records}
    (output / "distillation_manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    return manifest


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--yolov5-root", type=Path, required=True)
    result.add_argument("--teacher", type=Path, required=True)
    result.add_argument("--data", type=Path, required=True)
    result.add_argument("--unlabelled", type=Path, nargs="+", required=True)
    result.add_argument("--output", type=Path, required=True)
    result.add_argument("--device", default="0")
    result.add_argument("--image-size", type=int, default=416)
    result.add_argument("--confidence", type=float, default=0.010)
    result.add_argument("--iou", type=float, default=0.45)
    result.add_argument("--min-aspect", type=float, default=1.8)
    result.add_argument("--max-area-fraction", type=float, default=0.25)
    result.add_argument("--expected-target-count", type=int, default=1)
    return result


def main() -> int:
    args = parser().parse_args()
    manifest = build(args)
    print(json.dumps({key: manifest[key] for key in (
        "teacher_sha256", "authoritative_counts", "pseudo_accepted_images")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
