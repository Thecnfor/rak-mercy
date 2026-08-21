"""Conservative multi-instance splitting within one YOLO pen box."""

import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "deyes_stereo"))

from deyes_stereo.pen_feature_extractor import RectifiedImage, extract_pen_features  # noqa: E402


def _payload(box, class_name="pen"):
    return {"stamp_sec": 0, "stamp_nanosec": 77, "frame_id": "left", "image_width": 240, "image_height": 160, "detections": [{"class_name": class_name, "target_id": "merged", "det_index": 2, "confidence": .9, "bbox_xyxy": box}]}


def test_one_yolo_box_splits_two_separated_different_contrast_pens_stably():
    image = np.full((160, 240), 150, np.uint8)
    cv2.line(image, (42, 54), (198, 54), 35, 5)   # dark pen
    cv2.line(image, (48, 103), (204, 103), 205, 5)  # light pen, still edge/contrast visible
    rect = RectifiedImage(77, "left", 240, 160, image)
    results = [extract_pen_features(rect, _payload([30, 38, 215, 118])) for _ in range(2)]
    for features, rejected, reason in results:
        assert reason is None and rejected == [] and len(features) == 2
        assert [item["target_id"] for item in features] == ["merged__split_00", "merged__split_01"]
        assert all(item["axis_complete"] and item["split_from_merged_box"] for item in features)
    assert results[0][0][0]["axis_endpoints_px"] == results[1][0][0]["axis_endpoints_px"]


def test_trained_yolo_pencil_class_uses_the_same_pen_pipeline():
    image = np.full((160, 240), 150, np.uint8)
    cv2.line(image, (42, 54), (198, 54), 35, 5)
    cv2.line(image, (48, 103), (204, 103), 205, 5)
    rect = RectifiedImage(77, "left", 240, 160, image)
    features, rejected, reason = extract_pen_features(
        rect, _payload([30, 38, 215, 118], class_name="pencil")
    )
    assert reason is None and rejected == [] and len(features) == 2


def test_inconclusive_multi_component_box_is_rejected_not_partially_published():
    image = np.full((160, 240), 150, np.uint8)
    cv2.line(image, (42, 54), (198, 54), 35, 5)
    cv2.line(image, (3, 102), (50, 102), 30, 4)  # elongated but clipped at image edge
    rect = RectifiedImage(77, "left", 240, 160, image)
    features, rejected, reason = extract_pen_features(rect, _payload([0, 38, 215, 120]))
    assert features == [] and reason is None
    assert rejected[0]["reason"] == "merged_box_split_inconclusive"
