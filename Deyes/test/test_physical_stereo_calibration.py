"""ROS-free tests for the physical 9x6 calibration contract and solver helpers."""

from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "deyes_stereo"))

from deyes_stereo.physical_stereo_calibration import (  # noqa: E402
    board_descriptor,
    is_duplicate_pose,
    rectified_epipolar_errors,
)
from deyes_stereo.stereo_calibration_contract import (  # noqa: E402
    CALIBRATION_SIZE,
    coverage_cells,
    coverage_complete,
    validate_capture_arguments,
    validation_gate,
)


def test_capture_rejects_guessed_square_size_wrong_resolution_and_sample_count() -> None:
    assert validate_capture_arguments(
        square_size_m=0.0, requested_samples=30, width=1280, height=720
    ) == (
        "square_size_m_must_be_explicit_and_positive",
        "requested_samples_must_be_between_40_and_60",
        "resolution_must_be_exactly_640x360",
    )
    assert validate_capture_arguments(
        square_size_m=float("nan"), requested_samples=50, width=640, height=360
    ) == ("square_size_m_must_be_explicit_and_positive",)


def test_validation_requires_measurements_and_all_three_operator_confirmations() -> None:
    blocked = validation_gate(
        sample_count=50,
        resolution=CALIBRATION_SIZE,
        reproj_rms_px=0.2,
        epipolar_p95_px=0.2,
        source="physical_checkerboard",
        left_right_confirmed=True,
        baseline_sign_confirmed=False,
        scale_confirmed=False,
        coverage_complete=True,
    )
    assert not blocked.validated
    assert blocked.reasons == (
        "operator_baseline_sign_not_confirmed",
        "operator_scale_not_confirmed",
    )


def test_validation_has_hard_metric_and_resolution_gates() -> None:
    gate = validation_gate(
        sample_count=40,
        resolution=(641, 360),
        reproj_rms_px=0.501,
        epipolar_p95_px=0.501,
        source="spec_imx219",
        left_right_confirmed=True,
        baseline_sign_confirmed=True,
        scale_confirmed=True,
        coverage_complete=True,
    )
    assert not gate.validated
    assert set(gate.reasons) == {
        "resolution_not_640x360",
        "source_is_not_physical_checkerboard",
        "reproj_rms_exceeds_0_50_px",
        "epipolar_p95_exceeds_0_50_px",
    }


def test_coverage_requires_all_nine_cells() -> None:
    corners = [(30 + 220 * x, 30 + 110 * y) for x in range(3) for y in range(3)]
    assert coverage_complete(coverage_cells(corners, *CALIBRATION_SIZE))
    assert not coverage_complete({(0, 0), (1, 1)})


def test_duplicate_pose_descriptor_is_rejected() -> None:
    board = np.array([[100.0, 100.0], [300.0, 100.0], [100.0, 200.0], [300.0, 200.0]])
    descriptor = board_descriptor(board)
    assert is_duplicate_pose(descriptor, [descriptor.copy()])
    moved = board + np.array([170.0, 0.0])
    assert not is_duplicate_pose(board_descriptor(moved), [descriptor])


def test_rectified_epipolar_error_uses_vertical_pixel_difference() -> None:
    points_left = [np.array([[20.0, 40.0], [30.0, 50.0]], dtype=np.float32)]
    points_right = [np.array([[18.0, 40.25], [28.0, 49.5]], dtype=np.float32)]
    identity = np.eye(3, dtype=np.float64)
    projection = np.hstack([identity, np.zeros((3, 1))])
    error = rectified_epipolar_errors(
        points_left,
        points_right,
        identity,
        np.zeros((5, 1)),
        identity,
        np.zeros((5, 1)),
        identity,
        projection,
        identity,
        projection,
    )
    assert np.allclose(error, [0.25, 0.5])
