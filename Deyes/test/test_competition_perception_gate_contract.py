import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "deyes_stereo"))

from deyes_stereo.competition_perception_gate_contract import evaluate_detection_depth


def payload(box=(10, 10, 30, 30), count=1, permitted=True):
    return {
        "detection_count": count,
        "auto_grasp_permitted": permitted,
        "detections": [{"bbox_xyxy": list(box)}] * count,
    }


def test_accepts_one_pen_with_metric_depth():
    depth = np.full((40, 40), np.nan, dtype=np.float32)
    depth[12:28, 12:28] = 0.55
    result = evaluate_detection_depth(payload(), depth)
    assert result.accepted
    assert abs(result.depth_m - 0.55) < 1e-5


def test_rejects_zero_or_multiple_targets():
    depth = np.full((40, 40), 0.5, dtype=np.float32)
    assert not evaluate_detection_depth(payload(count=0), depth).accepted
    assert not evaluate_detection_depth(payload(count=2), depth).accepted


def test_rejects_invalid_or_out_of_range_depth():
    assert not evaluate_detection_depth(payload(), np.full((40, 40), np.nan, np.float32)).accepted
    assert not evaluate_detection_depth(payload(), np.full((40, 40), 1.5, np.float32)).accepted
