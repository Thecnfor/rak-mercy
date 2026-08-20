"""Offline, ROS-free acceptance metrics for the held-out pen batches.

All inputs are external JSONL evidence.  No image, annotation, weight, ONNX or
TensorRT engine is created or stored by this module.
"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from .yolo_detector_contract import normalize_sha256


class PenEvaluationError(ValueError):
    """Raised for incomplete, leaking, or unauditable evaluation evidence."""


def _bbox(value: Any, field: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise PenEvaluationError(f"{field}_must_be_a_four_value_xyxy_bbox")
    try:
        x0, y0, x1, y1 = (float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise PenEvaluationError(f"{field}_must_contain_numeric_values") from exc
    if not x1 > x0 or not y1 > y0:
        raise PenEvaluationError(f"{field}_must_have_positive_area")
    return [x0, y0, x1, y1]


def bbox_iou(first: Sequence[float], second: Sequence[float]) -> float:
    left, top = max(first[0], second[0]), max(first[1], second[1])
    right, bottom = min(first[2], second[2]), min(first[3], second[3])
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    first_area = (first[2] - first[0]) * (first[3] - first[1])
    second_area = (second[2] - second[0]) * (second[3] - second[1])
    union = first_area + second_area - intersection
    return intersection / union if union > 0 else 0.0


def validate_ground_truth(records: Iterable[Mapping[str, Any]]) -> dict[str, Mapping[str, Any]]:
    """Validate the batch split assignment and return image-id keyed evidence."""
    by_id: dict[str, Mapping[str, Any]] = {}
    batch_splits: dict[str, str] = {}
    for record in records:
        image_id = str(record.get("image_id") or "").strip()
        batch_id = str(record.get("batch_id") or "").strip()
        split = str(record.get("split") or "").strip()
        if not image_id or not batch_id or split not in {"train", "val", "test"}:
            raise PenEvaluationError("ground_truth_requires_image_id_batch_id_and_train_val_or_test_split")
        if image_id in by_id:
            raise PenEvaluationError(f"duplicate_ground_truth_image_id:{image_id}")
        prior = batch_splits.setdefault(batch_id, split)
        if prior != split:
            raise PenEvaluationError(f"batch_split_leakage:{batch_id}:{prior}:{split}")
        boxes = record.get("boxes")
        if not isinstance(boxes, list):
            raise PenEvaluationError(f"ground_truth_boxes_must_be_a_list:{image_id}")
        for index, box in enumerate(boxes):
            _bbox(box, f"ground_truth[{image_id}].boxes[{index}]")
        by_id[image_id] = record
    if not any(record.get("split") == "test" for record in by_id.values()):
        raise PenEvaluationError("ground_truth_contains_no_held_out_test_batch")
    return by_id


def _match(expected: list[list[float]], candidates: list[list[float]], threshold: float) -> tuple[int, int, int]:
    unmatched = set(range(len(expected)))
    true_positive = 0
    false_positive = 0
    for candidate in candidates:
        best_index = -1
        best_iou = threshold
        for index in unmatched:
            score = bbox_iou(expected[index], candidate)
            if score >= best_iou:
                best_index, best_iou = index, score
        if best_index < 0:
            false_positive += 1
        else:
            true_positive += 1
            unmatched.remove(best_index)
    return true_positive, false_positive, len(unmatched)


def evaluate_test_records(
    ground_truth: Iterable[Mapping[str, Any]], predictions: Iterable[Mapping[str, Any]], *, iou_threshold: float = 0.5
) -> dict[str, Any]:
    """Measure detection and end-to-end 3D candidate recall on held-out batches."""
    if not 0.0 < iou_threshold <= 1.0:
        raise PenEvaluationError("iou_threshold_must_be_in_(0,1]")
    truth_by_id = validate_ground_truth(ground_truth)
    test_truth = {key: value for key, value in truth_by_id.items() if value.get("split") == "test"}
    predicted_by_id: dict[str, Mapping[str, Any]] = {}
    identities: set[tuple[str, str]] = set()
    for record in predictions:
        image_id = str(record.get("image_id") or "").strip()
        if image_id not in test_truth or image_id in predicted_by_id:
            raise PenEvaluationError(f"prediction_must_match_one_unique_test_image:{image_id or '<empty>'}")
        model_id = str(record.get("model_id") or "").strip()
        if not model_id:
            raise PenEvaluationError("prediction_requires_non_empty_model_id")
        try:
            model_sha256 = normalize_sha256(str(record.get("model_sha256") or ""))
        except ValueError as exc:
            raise PenEvaluationError(str(exc)) from exc
        identities.add((model_id, model_sha256))
        predicted_by_id[image_id] = record
    if set(predicted_by_id) != set(test_truth):
        missing = sorted(set(test_truth) - set(predicted_by_id))
        raise PenEvaluationError(f"missing_test_predictions:{','.join(missing[:5])}")
    if len(identities) != 1:
        raise PenEvaluationError("test_predictions_must_use_exactly_one_model_identity")

    det_tp = det_fp = det_fn = three_d_tp = 0
    for image_id, truth in test_truth.items():
        expected = [_bbox(box, "ground_truth_box") for box in truth["boxes"]]
        prediction = predicted_by_id[image_id]
        detections = prediction.get("detections", [])
        if not isinstance(detections, list):
            raise PenEvaluationError(f"detections_must_be_a_list:{image_id}")
        det_boxes = [_bbox(item.get("bbox_xyxy"), "detection_bbox") for item in detections]
        tp, fp, fn = _match(expected, det_boxes, iou_threshold)
        det_tp, det_fp, det_fn = det_tp + tp, det_fp + fp, det_fn + fn

        objects = prediction.get("objects_3d", prediction.get("objects", []))
        if not isinstance(objects, list):
            raise PenEvaluationError(f"objects_3d_must_be_a_list:{image_id}")
        successful = [
            _bbox(item.get("bbox_xyxy"), "object_3d_bbox")
            for item in objects
            if isinstance(item, Mapping) and item.get("status") == "ok"
        ]
        three_d_tp += _match(expected, successful, iou_threshold)[0]

    precision = det_tp / (det_tp + det_fp) if det_tp + det_fp else 0.0
    recall = det_tp / (det_tp + det_fn) if det_tp + det_fn else 0.0
    three_d_rate = three_d_tp / (det_tp + det_fn) if det_tp + det_fn else 0.0
    model_id, model_sha256 = next(iter(identities))
    return {
        "model_id": model_id,
        "model_sha256": model_sha256,
        "iou_threshold": iou_threshold,
        "test_image_count": len(test_truth),
        "test_pen_count": det_tp + det_fn,
        "detection": {"tp": det_tp, "fp": det_fp, "fn": det_fn, "precision": precision, "recall": recall},
        "end_to_end_3d_candidate": {"success_count": three_d_tp, "success_rate": three_d_rate},
        "passes_95_precision_recall": precision >= 0.95 and recall >= 0.95,
    }


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    try:
        return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    except (OSError, json.JSONDecodeError) as exc:
        raise PenEvaluationError(f"invalid_jsonl:{path}:{exc}") from exc


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate held-out pen detection and 3D candidates.")
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--predictions", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--iou-threshold", type=float, default=0.5)
    args = parser.parse_args()
    report = evaluate_test_records(_read_jsonl(args.ground_truth), _read_jsonl(args.predictions), iou_threshold=args.iou_threshold)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, sort_keys=True))
    return 0 if report["passes_95_precision_recall"] else 2


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
