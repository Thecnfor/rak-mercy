"""ROS-free multi-pen feature extraction and frame-contract tests."""

import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "deyes_stereo"))

from deyes_stereo.pen_feature_extractor import (  # noqa: E402
    RectifiedImage,
    build_features_payload,
    extract_pen_features,
)


WIDTH, HEIGHT, STAMP = 240, 160, 41


def _image(lines: tuple[tuple[int, int, int], ...]) -> np.ndarray:
    gray = np.full((HEIGHT, WIDTH), 150, dtype=np.uint8)
    for center_x, center_y, length in lines:
        cv2.line(gray, (center_x - length // 2, center_y), (center_x + length // 2, center_y), 30, 5)
    return gray


def _bbox(center_x: int, center_y: int, length: int = 72) -> list[float]:
    return [center_x - length / 2 - 7, center_y - 10, center_x + length / 2 + 7, center_y + 10]


def _payload(detections, **changes):
    value = {"stamp_sec": 0, "stamp_nanosec": STAMP, "frame_id": "left", "image_width": WIDTH, "image_height": HEIGHT, "detections": detections}
    value.update(changes)
    return value


def _pen(center_x: int, center_y: int, *, target_id: str | None = None, det_index: int | None = None, length: int = 72):
    value = {"class_name": "pen", "confidence": .9, "bbox_xyxy": _bbox(center_x, center_y, length)}
    if target_id is not None:
        value["target_id"] = target_id
    if det_index is not None:
        value["det_index"] = det_index
    return value


@pytest.mark.parametrize("count", [2, 3, 4])
def test_each_detected_pen_gets_a_distinct_stable_feature(count):
    ys = tuple(24 + 30 * index for index in range(count))
    image = RectifiedImage(STAMP, "left", WIDTH, HEIGHT, _image(tuple((120, y, 72) for y in ys)))
    detections = [_pen(120, y) for y in reversed(ys)]
    first = extract_pen_features(image, _payload(detections))
    second = extract_pen_features(image, _payload(detections))
    features, rejections, frame_reason = first
    assert frame_reason is None and not rejections and len(features) == count
    assert [(item["det_index"], item["target_id"]) for item in features] == [(index, f"target_{index:02d}") for index in range(count)]
    assert [(item["det_index"], item["target_id"]) for item in second[0]] == [(item["det_index"], item["target_id"]) for item in features]


def test_duplicate_source_ids_are_deterministically_renamed_not_merged():
    image = RectifiedImage(STAMP, "left", WIDTH, HEIGHT, _image(((120, 40, 72), (120, 100, 72))))
    features, rejections, reason = extract_pen_features(image, _payload([_pen(120, 40, target_id="same", det_index=0), _pen(120, 100, target_id="same", det_index=0)]))
    assert reason is None and not rejections
    assert [(feature["det_index"], feature["target_id"]) for feature in features] == [(0, "same"), (1, "same__det_01")]


def test_bad_box_is_isolated_and_payload_reports_per_target_failure():
    image = RectifiedImage(STAMP, "left", WIDTH, HEIGHT, _image(((120, 35, 72), (120, 125, 72))))
    features, rejections, reason = extract_pen_features(image, _payload([_pen(120, 35, target_id="good-a"), _pen(120, 80, target_id="bad"), _pen(120, 125, target_id="good-b")]))
    assert reason is None and {feature["target_id"] for feature in features} == {"good-a", "good-b"}
    assert rejections == [{"target_id": "bad", "det_index": 1, "reason": "no_elongated_component", "bbox_xyxy": _bbox(120, 80)}]
    result = build_features_payload(image, features, rejections)
    assert result["success_count"] == 2 and result["failure_count"] == 1
    assert result["features"] == features and result["target_rejections"] == rejections


@pytest.mark.parametrize("changes, expected", [
    ({"stamp_nanosec": STAMP + 1}, "detection_image_stamp_mismatch"),
    ({"frame_id": "raw_left"}, "detection_image_frame_mismatch"),
    ({"image_width": WIDTH - 1}, "detection_image_size_mismatch"),
])
def test_stamp_frame_and_size_are_whole_frame_gates(changes, expected):
    image = RectifiedImage(STAMP, "left", WIDTH, HEIGHT, _image(((120, 40, 72), (120, 100, 72))))
    features, rejections, reason = extract_pen_features(image, _payload([_pen(120, 40), _pen(120, 100)], **changes))
    assert features == [] and rejections == [] and reason == expected
    result = build_features_payload(image, features, rejections, reason)
    assert result["success_count"] == result["failure_count"] == 0 and result["rejection_reason"] == expected


def test_overlapping_boxes_remain_independent_and_edge_axis_is_incomplete():
    image = RectifiedImage(STAMP, "left", WIDTH, HEIGHT, _image(((120, 50, 72), (120, 74, 72), (24, 130, 70))))
    features, rejections, reason = extract_pen_features(image, _payload([
        _pen(120, 50, target_id="overlap-a"), _pen(120, 74, target_id="overlap-b"), _pen(24, 130, target_id="edge", length=70),
    ]))
    assert reason is None and not rejections and {feature["target_id"] for feature in features} == {"overlap-a", "overlap-b", "edge"}
    edge = next(feature for feature in features if feature["target_id"] == "edge")
    assert edge["axis_complete"] is False
