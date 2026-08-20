"""ROS-free tests for the pen detector runtime contract."""

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "src" / "deyes_stereo"
sys.path.insert(0, str(PACKAGE_ROOT))

from deyes_stereo.yolo_detector_contract import (  # noqa: E402
    filter_detections_by_allowed_class_ids,
    parse_allowed_class_ids_json,
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

    assert 'DeclareLaunchArgument("detector_image_topic", default_value="/x1/left_camera/image_raw")' in launch
    assert '"image_topic": LaunchConfiguration("detector_image_topic")' in launch
    assert 'image_topic: "/x1/left_camera/image_raw"' in defaults
    assert 'allowed_class_ids_json: "[]"' in defaults
    assert 'image_topic: "/x1/stereo/debug/left_rect"' in pen_defaults
    assert 'model_path: ""' in pen_defaults
    assert "class_names_json: '{\"0\":\"pen\"}'" in pen_defaults
    assert 'allowed_class_ids_json: "[0]"' in pen_defaults
