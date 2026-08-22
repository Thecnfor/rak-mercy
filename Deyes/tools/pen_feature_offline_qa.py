"""Offline geometric-availability QA for pen_feature; never edits the dataset."""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "deyes_stereo"))

from deyes_stereo.pen_feature_extractor import RectifiedImage, extract_one_pen  # noqa: E402


def _expected_orientation(batch_id: str) -> str | None:
    for name in ("horizontal", "vertical", "diagonal"):
        if name in batch_id:
            return name
    return None


def _axis_angle_deg(feature: dict[str, Any]) -> float:
    first, second = feature["axis_endpoints_px"]
    angle = math.degrees(math.atan2(second[1] - first[1], second[0] - first[0])) % 180.0
    return round(angle, 2)


def _orientation_matches(angle: float, expected: str | None) -> bool | None:
    if expected is None:
        return None
    if expected == "horizontal":
        return min(angle, 180.0 - angle) <= 20.0
    if expected == "vertical":
        # These placements were described as near-vertical.  The oblique camera
        # view maps them to roughly 60--70 degrees in rectified image pixels.
        return abs(angle - 90.0) <= 35.0
    return 25.0 <= angle <= 75.0 or 105.0 <= angle <= 155.0


def _payload(record: dict[str, Any]) -> dict[str, Any]:
    return {
        "stamp_sec": 0,
        "stamp_nanosec": 1,
        "frame_id": "offline_rectified_left",
        "image_width": record["width"],
        "image_height": record["height"],
        "detections": [
            {
                "class_name": "pen",
                "target_id": "offline_gt_00",
                "det_index": 0,
                "confidence": 1.0,
                "bbox_xyxy": box,
            }
            for box in record["boxes"]
        ],
    }


def evaluate(
    records: list[dict[str, Any]],
    bbox_source: str = "manual ground_truth_index.jsonl",
) -> dict[str, Any]:
    batches: dict[str, dict[str, Any]] = defaultdict(
        lambda: {"positive_images": 0, "not_applicable_images": 0, "usable": 0,
                 "angle_checked": 0, "angle_consistent": 0, "failures": []}
    )
    total = {"positive_images": 0, "not_applicable_images": 0, "usable": 0,
             "angle_checked": 0, "angle_consistent": 0, "failures": []}
    for record in records:
        batch = batches[record["batch_id"]]
        boxes = record["boxes"]
        if len(boxes) != 1:
            batch["not_applicable_images"] += 1
            total["not_applicable_images"] += 1
            continue
        gray = cv2.imread(record["image_path"], cv2.IMREAD_GRAYSCALE)
        if gray is None:
            reason = "image_unreadable"
            feature = None
        else:
            image = RectifiedImage(1, "offline_rectified_left", record["width"], record["height"], gray)
            feature, reason = extract_one_pen(image, _payload(record))
        for value in (batch, total):
            value["positive_images"] += 1
        if feature is None or not feature["axis_complete"]:
            failure = {"image_id": record["image_id"], "reason": reason}
            batch["failures"].append(failure)
            total["failures"].append(failure)
            continue
        for value in (batch, total):
            value["usable"] += 1
        expected = _expected_orientation(record["batch_id"])
        matched = _orientation_matches(_axis_angle_deg(feature), expected)
        if matched is not None:
            for value in (batch, total):
                value["angle_checked"] += 1
                value["angle_consistent"] += int(matched)
            if not matched:
                failure = {"image_id": record["image_id"], "reason": "axis_angle_unexpected",
                           "angle_deg": _axis_angle_deg(feature), "expected": expected}
                batch["failures"].append(failure)
                total["failures"].append(failure)
    for value in [*batches.values(), total]:
        positives = value["positive_images"]
        value["geometry_usable_rate"] = round(value["usable"] / positives, 4) if positives else None
        checked = value["angle_checked"]
        value["axis_direction_consistency_rate"] = round(value["angle_consistent"] / checked, 4) if checked else None
        value["failures"] = value["failures"][:10]
    return {
        "report_type": "pen_feature_geometric_availability",
        "claim_limit": "No pixel ground truth: this is not segmentation accuracy.",
        "bbox_source": bbox_source,
        "evaluation_gate": "exactly one pen bbox per image",
        "totals": total,
        "batches": dict(sorted(batches.items())),
    }


def apply_yolo_predictions(
    records: list[dict[str, Any]],
    label_dir: Path,
) -> list[dict[str, Any]]:
    """Replace manual boxes with normalized YOLO prediction labels."""
    predicted: list[dict[str, Any]] = []
    for source in records:
        record = dict(source)
        boxes: list[list[float]] = []
        label_path = label_dir / f"{record['image_id']}.txt"
        if label_path.exists():
            for line_number, line in enumerate(label_path.read_text(encoding="utf-8").splitlines(), 1):
                values = line.split()
                if len(values) not in (5, 6):
                    raise ValueError(f"{label_path}:{line_number}: expected 5 or 6 YOLO fields")
                _, center_x, center_y, width, height = map(float, values[:5])
                image_width, image_height = float(record["width"]), float(record["height"])
                boxes.append([
                    (center_x - width / 2.0) * image_width,
                    (center_y - height / 2.0) * image_height,
                    (center_x + width / 2.0) * image_width,
                    (center_y + height / 2.0) * image_height,
                ])
        record["boxes"] = boxes
        predicted.append(record)
    return predicted


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument(
        "--prediction-label-dir",
        type=Path,
        help="Optional YOLO save-txt directory; missing files mean zero detections.",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    records = [json.loads(line) for line in args.ground_truth.read_text(encoding="utf-8").splitlines() if line]
    bbox_source = "manual ground_truth_index.jsonl"
    if args.prediction_label_dir is not None:
        if not args.prediction_label_dir.is_dir():
            parser.error(f"prediction label directory does not exist: {args.prediction_label_dir}")
        records = apply_yolo_predictions(records, args.prediction_label_dir)
        bbox_source = f"YOLO saved predictions: {args.prediction_label_dir}"
    report = evaluate(records, bbox_source=bbox_source)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report["totals"], sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
