"""ROS-free tests for the pen detector runtime contract."""

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "src" / "deyes_stereo"
sys.path.insert(0, str(PACKAGE_ROOT))

from deyes_stereo.yolo_detector_contract import (  # noqa: E402
    filter_detections_by_allowed_class_ids,
    gate_target_detections,
    normalize_sha256,
    parse_allowed_class_ids_json,
    TensorRTBinding,
    validate_tensorrt_yolov5_contract,
)


NODE_PATH = PACKAGE_ROOT / "deyes_stereo" / "yolo_detector_node.py"
LAUNCH_PATH = ROOT / "src" / "deyes_bringup" / "launch" / "imx219_stereo.launch.py"
DEFAULTS_PATH = ROOT / "config" / "stereo" / "yolo_detector.defaults.yaml"
PEN_DEFAULTS_PATH = ROOT / "config" / "stereo" / "pen_detector.defaults.yaml"


def test_empty_allowlist_is_a_pass_through() -> None:
    allowed, error = parse_allowed_class_ids_json("[]")
    detections = [{"class_id": 0}, {"class_id": 7}]

    assert error is None
    assert filter_detections_by_allowed_class_ids(detections, allowed) == detections


def test_pen_class_zero_allowlist_filters_other_classes() -> None:
    allowed, error = parse_allowed_class_ids_json("[0]")

    assert error is None
    assert filter_detections_by_allowed_class_ids(
        [{"class_id": 0, "class_name": "pen"}, {"class_id": 1}], allowed
    ) == [{"class_id": 0, "class_name": "pen"}]


def test_invalid_allowlist_json_is_rejected_without_fallback() -> None:
    for value in ("", "{\"0\": true}", "[0, 0]", "[true]", "[-1]", "[0.0]"):
        _, error = parse_allowed_class_ids_json(value)
        assert error is not None
        assert "allowed_class_ids_json" in error


def test_all_backends_share_the_single_final_class_filter() -> None:
    content = NODE_PATH.read_text(encoding="utf-8")
    timer_body = content[content.index("    def _on_timer(") : content.index("\n\ndef main()")]

    assert "detections, inference_ms = self._infer(frame)" in timer_body
    assert "filter_detections_by_allowed_class_ids(detections, self._allowed_class_ids)" in timer_body
    assert timer_body.index("detections, inference_ms = self._infer(frame)") < timer_body.index(
        "filter_detections_by_allowed_class_ids(detections, self._allowed_class_ids)"
    )
    for backend_method in ("_infer_ultralytics", "_infer_opencv_dnn", "_infer_tensorrt"):
        assert f"return self.{backend_method}(frame)" in content


def test_launch_and_configs_preserve_default_and_provide_pen_profile() -> None:
    launch = LAUNCH_PATH.read_text(encoding="utf-8")
    defaults = DEFAULTS_PATH.read_text(encoding="utf-8")
    pen_defaults = PEN_DEFAULTS_PATH.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("detector_image_topic", default_value="/x1/stereo/debug/left_rect")' in launch
    assert '"image_topic": LaunchConfiguration("detector_image_topic")' in launch
    assert 'image_topic: "/x1/stereo/debug/left_rect"' in defaults
    assert 'allowed_class_ids_json: "[]"' in defaults
    assert 'image_topic: "/x1/stereo/debug/left_rect"' in pen_defaults
    assert 'model_path: ""' in pen_defaults
    assert "class_names_json: '{\"0\":\"pen\"}'" in pen_defaults
    assert 'allowed_class_ids_json: "[0]"' in pen_defaults
    assert 'expected_class_count: 1' in pen_defaults
    assert 'DeclareLaunchArgument("detector_expected_model_sha256", default_value="")' in launch


def test_tensorrt_contract_accepts_only_static_yolov5_single_io_layout() -> None:
    contract = validate_tensorrt_yolov5_contract(
        [
            TensorRTBinding(0, "images", True, (1, 3, 640, 640), "float32"),
            TensorRTBinding(1, "output0", False, (1, 25200, 6), "float16"),
        ],
        input_width=640,
        input_height=640,
        class_count=1,
    )
    assert contract.input_index == 0
    assert contract.output_index == 1
    assert contract.output_shape == (1, 25200, 6)


def test_tensorrt_contract_rejects_ambiguous_or_non_yolov5_layouts() -> None:
    base = [
        TensorRTBinding(0, "images", True, (1, 3, 640, 640), "float32"),
        TensorRTBinding(1, "output0", False, (1, 84, 8400), "float16"),
    ]
    for bindings in (
        base,
        [*base, TensorRTBinding(2, "aux", False, (1, 1, 1), "float16")],
        [
            TensorRTBinding(0, "images", True, (-1, 3, 640, 640), "float32"),
            TensorRTBinding(1, "output0", False, (1, 25200, 6), "float16"),
        ],
    ):
        try:
            validate_tensorrt_yolov5_contract(
                bindings, input_width=640, input_height=640, class_count=1
            )
        except ValueError:
            pass
        else:  # pragma: no cover - explicit failure message is clearer than pytest.raises in loop
            raise AssertionError("unsafe TensorRT engine layout was accepted")


def test_model_digest_is_mandatory_and_normalized() -> None:
    digest = "AB" * 32
    assert normalize_sha256(digest) == digest.lower()
    for invalid in ("", "a" * 63, "g" * 64):
        try:
            normalize_sha256(invalid)
        except ValueError:
            pass
        else:  # pragma: no cover
            raise AssertionError("invalid digest was accepted")


def test_node_enforces_identity_and_runtime_shape_contract() -> None:
    content = NODE_PATH.read_text(encoding="utf-8")
    assert '"model_id": ""' in content
    assert '"expected_model_sha256": ""' in content
    assert "model_sha256_mismatch" in content
    assert "validate_tensorrt_yolov5_contract(" in content
    assert "runtime_output_shape_does_not_match_validated_tensorrt_contract" in content


def test_two_pen_targets_receive_stable_ids_and_are_not_rejected() -> None:
    detections, rejection = gate_target_detections(
        [
            {"class_id": 0, "confidence": 0.85, "bbox_xyxy": [340, 30, 450, 60]},
            {"class_id": 0, "confidence": 0.91, "bbox_xyxy": [20, 40, 120, 70]},
        ],
        expected_max_targets=2,
        duplicate_iou=0.8,
    )
    assert rejection is None
    assert [(item["det_index"], item["target_id"]) for item in detections] == [
        (0, "target_00"),
        (1, "target_01"),
    ]
    assert detections[0]["confidence"] == 0.91


def test_more_than_two_or_duplicate_targets_fail_closed_for_grasping() -> None:
    many, many_reason = gate_target_detections(
        [{"class_id": 0, "bbox_xyxy": [i * 40, 0, i * 40 + 20, 20]} for i in range(3)],
        expected_max_targets=2,
        duplicate_iou=0.8,
    )
    duplicate, duplicate_reason = gate_target_detections(
        [
            {"class_id": 0, "bbox_xyxy": [0, 0, 100, 30]},
            {"class_id": 0, "bbox_xyxy": [1, 0, 101, 30]},
        ],
        expected_max_targets=2,
        duplicate_iou=0.8,
    )
    assert many == [] and many_reason == "target_count_exceeds_expected_max:3>2"
    assert duplicate == [] and duplicate_reason == "severe_bbox_overlap_suspected_duplicate"
