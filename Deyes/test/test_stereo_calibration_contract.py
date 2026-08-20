"""ROS-free checks for the stereo calibration and rectified-geometry contract."""

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
CALIBRATION_PATH = ROOT / "config" / "camera" / "stereo_calib.yaml"
CUDA_NODE_PATH = ROOT / "src" / "deyes_capture_cpp" / "src" / "cuda_stereo_depth_node.cpp"
CAPTURE_NODE_PATH = ROOT / "src" / "deyes_capture_cpp" / "src" / "imx219_stereo_capture_node.cpp"


def scale_intrinsics(k: list[list[float]], scale_x: float, scale_y: float) -> list[list[float]]:
    """The non-ROS reference for CUDA debug-resolution rectification."""
    scaled = [row[:] for row in k]
    scaled[0][0] *= scale_x
    scaled[0][1] *= scale_x
    scaled[0][2] *= scale_x
    scaled[1][1] *= scale_y
    scaled[1][2] *= scale_y
    return scaled


def test_debug_intrinsic_scaling_is_axis_correct() -> None:
    k = [[800.0, 2.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]]
    assert scale_intrinsics(k, 0.5, 0.5) == [
        [400.0, 1.0, 320.0],
        [0.0, 450.0, 180.0],
        [0.0, 0.0, 1.0],
    ]


def test_spec_calibration_is_explicitly_unvalidated() -> None:
    content = CALIBRATION_PATH.read_text(encoding="utf-8")
    for key in (
        "calibration_id:",
        "robot_id:",
        "camera_pair_id:",
        "img_size:",
        "board_inner_corners: [8, 7]",
        "square_size_m:",
        "reproj_rms_px:",
        "epipolar_p95_px:",
        "date:",
        "source:",
        "validated: false",
    ):
        assert key in content
    assert "source: \"spec_imx219\"" in content
    assert "reproj_rms_px: null" in content
    assert "epipolar_p95_px: null" in content
    for key in ("source", "validated", "reproj_rms_px"):
        assert len(re.findall(rf"^{key}:.*$", content, flags=re.MULTILINE)) == 1
    # A debug/spec file must not retain a made-up numeric measurement under a
    # legacy name while the contract fields correctly state null.
    assert "reproj_error:" not in content
    assert re.search(r"^reproj_rms_px:\s*null\s*$", content, flags=re.MULTILINE)
    assert re.search(r"^epipolar_p95_px:\s*null\s*$", content, flags=re.MULTILINE)


def test_cuda_geometry_uses_rectified_camera_info_not_raw_camera_info() -> None:
    content = CUDA_NODE_PATH.read_text(encoding="utf-8")
    assert 'declare_parameter<std::string>("left_rect_camera_info_topic"' in content
    assert '"/x1/stereo/left/camera_info_rect"' in content
    assert "make_rectified_left_camera_info" in content
    assert "left_rect_info_pub_->publish" in content
    assert "validated_calibration_resolution_mismatch" in content
    assert "k->at<double>(0, 0) *= scale_x" in content
    assert "k->at<double>(1, 1) *= scale_y" in content
    assert "left_camera_info_topic" not in content
    assert "right_camera_info_topic" not in content


def test_cuda_accepts_recorded_physical_board_shapes_not_a_9x6_literal() -> None:
    content = CUDA_NODE_PATH.read_text(encoding="utf-8")
    assert "only the physical 9x6 checkerboard" not in content
    assert "value < 4.0" in content
    assert "std::floor(value) != value" in content
    assert "validated stereo calibration measurement gates are not satisfied" in content
    assert "validated stereo calibration resolution must be 640x360" in content


def test_capture_does_not_rewrite_pair_stamps_to_a_midpoint() -> None:
    content = CAPTURE_NODE_PATH.read_text(encoding="utf-8")
    assert "pair_diagnostics" in content
    assert "window_p95_skew_ms" in content
    assert "left_output_frame.stamp_sec" in content
    assert "right_output_frame.stamp_sec" in content
    assert "best_left->stamp_sec + best_right->stamp_sec" not in content
