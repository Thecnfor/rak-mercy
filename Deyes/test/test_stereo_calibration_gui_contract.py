"""ROS/Qt-free checks for desktop stereo calibration capture state."""

from datetime import datetime
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "deyes_stereo"))

from deyes_stereo.stereo_calibration_gui_contract import CaptureSettings, StabilityTracker, make_manifest, session_directory  # noqa: E402


def test_capture_settings_preserves_formal_sample_bounds() -> None:
    assert CaptureSettings(target_samples=39).errors() == ("样本数必须在 40–60 之间",)
    assert CaptureSettings(board_cols=8, board_rows=7, square_size_m=0.020, target_samples=50).errors() == ()


def test_stability_requires_continuous_low_corner_motion() -> None:
    tracker = StabilityTracker(hold_s=0.8, max_motion_px=0.8)
    points = np.array([[10.0, 10.0], [20.0, 20.0]], dtype=np.float32)
    assert tracker.update(points, 0.0)[0] is False
    assert tracker.update(points + 0.1, 0.3)[0] is False
    assert tracker.update(points + 0.2, 1.2)[0] is True
    assert tracker.update(points + 2.0, 1.3)[0] is False


def test_manifest_matches_formal_capture_shape() -> None:
    settings = CaptureSettings()
    manifest = make_manifest(settings=settings, samples=[], reject_counts={"motion_blur": 2}, coverage={(0, 0)}, left_topic="/x1/left_camera/image_raw", right_topic="/x1/right_camera/image_raw", revision="test")
    assert manifest["schema_version"] == 2
    assert manifest["source"] == "ros_topics"
    assert manifest["resolution"] == [640, 360]
    assert manifest["board_inner_corners"] == [9, 6]
    assert manifest["coverage_cells"] == [[0, 0]]
    assert session_directory("/tmp/runs", datetime(2026, 8, 21, 9, 5, 3)).name == "stereo_20260821_090503"


def test_gui_default_matches_the_formal_checkerboard() -> None:
    settings = CaptureSettings()
    assert (settings.board_cols, settings.board_rows) == (9, 6)


def test_clean_package_install_declares_desktop_assets() -> None:
    setup_text = (ROOT / "src" / "deyes_stereo" / "setup.py").read_text(encoding="utf-8")
    for asset in (
        "launch_stereo_calibration_gui.sh",
        "install_stereo_calibration_desktop.sh",
        "mercury-x1-stereo-calibration.desktop",
    ):
        assert asset in setup_text
    gui_text = (ROOT / "src" / "deyes_stereo" / "deyes_stereo" / "stereo_calibration_gui.py").read_text(encoding="utf-8")
    launcher_text = (ROOT / "tools" / "launch_stereo_calibration_gui.sh").read_text(encoding="utf-8")
    assert "/home/elephant" not in gui_text
    assert "deyes_physical_ws" not in launcher_text
