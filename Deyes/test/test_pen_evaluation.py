"""ROS-free regression tests for held-out pen evaluation evidence."""

import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "deyes_stereo"))

from deyes_stereo.pen_evaluation import PenEvaluationError, evaluate_test_records  # noqa: E402


SHA = "a" * 64


def truth() -> list[dict]:
    return [
        {"image_id": "train-1", "batch_id": "batch-a", "split": "train", "boxes": [[0, 0, 10, 10]]},
        {"image_id": "test-1", "batch_id": "batch-b", "split": "test", "boxes": [[0, 0, 10, 10], [20, 0, 30, 10]]},
    ]


def prediction() -> list[dict]:
    return [{
        "image_id": "test-1", "model_id": "pen_yolov5n_v1", "model_sha256": SHA,
        "detections": [{"bbox_xyxy": [0, 0, 10, 10]}, {"bbox_xyxy": [20, 0, 30, 10]}],
        "objects_3d": [{"status": "ok", "bbox_xyxy": [0, 0, 10, 10]}, {"status": "ok", "bbox_xyxy": [20, 0, 30, 10]}],
    }]


def test_evaluation_reports_precision_recall_and_3d_candidate_rate() -> None:
    report = evaluate_test_records(truth(), prediction())
    assert report["detection"] == {"tp": 2, "fp": 0, "fn": 0, "precision": 1.0, "recall": 1.0}
    assert report["end_to_end_3d_candidate"] == {"success_count": 2, "success_rate": 1.0}
    assert report["passes_95_precision_recall"] is True


def test_evaluation_rejects_split_leakage_and_missing_3d_candidate() -> None:
    leaking = truth() + [{"image_id": "val-1", "batch_id": "batch-b", "split": "val", "boxes": []}]
    with pytest.raises(PenEvaluationError, match="batch_split_leakage"):
        evaluate_test_records(leaking, prediction())

    incomplete = prediction()
    incomplete[0]["objects_3d"] = [{"status": "ok", "bbox_xyxy": [0, 0, 10, 10]}]
    report = evaluate_test_records(truth(), incomplete)
    assert report["end_to_end_3d_candidate"] == {"success_count": 1, "success_rate": 0.5}
