import hashlib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "src/deyes_bringup/launch/real_robot_single_shot_dry_run.launch.py"
SCRIPT = ROOT / "tools/run_real_robot_dry_run.sh"
MODEL = ROOT / "models/pen/pen_student_01875_416_v1.onnx"
DEFAULTS = ROOT / "config/stereo/single_shot_pick.defaults.yaml"
EXPECTED_SHA256 = "8916cbf25949d6e8b03c01e6ca1c7871aeac0ad105c931f1dd9881cb5d5a4c4e"


def test_versioned_model_has_expected_identity():
    assert MODEL.is_file()
    assert hashlib.sha256(MODEL.read_bytes()).hexdigest() == EXPECTED_SHA256


def test_selected_student_uses_yolov5_decoder_contract():
    assert 'tensorrt_output_layout: "yolov5"' in DEFAULTS.read_text(encoding="utf-8")


def test_unified_launch_starts_depth_plane_and_fail_closed_pick():
    source = LAUNCH.read_text(encoding="utf-8")
    assert '"enable_cuda_depth": "true"' in source
    assert '"cuda_depth_publish_debug_rect": "true"' in source
    assert '"enable_ground_plane": "true"' in source
    assert '"dry_run": "true"' in source
    assert '"enable_live_execution": "false"' in source
    assert '"operator_confirmed": "false"' in source
    assert '"expected_target_count": "1"' in source


def test_entrypoint_verifies_assets_and_builds_device_local_engine():
    source = SCRIPT.read_text(encoding="utf-8")
    for required in ("ROBOT_IP", "STEREO_CALIB", "HANDEYE_CALIB", "RIGHT_ARM_SITE"):
        assert f'${{{required}:-}}' in source
    assert EXPECTED_SHA256 in source
    assert "--fp16" in source
    assert "sha256sum" in source
    assert "dry_run:=\"true\"" not in source  # launch owns the immutable safe defaults
    assert "real_robot_single_shot_dry_run.launch.py" in source
