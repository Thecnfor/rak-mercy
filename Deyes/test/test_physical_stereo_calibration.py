"""ROS-free tests for the physical checkerboard calibration contract and solver helpers."""

import argparse
import json
from pathlib import Path
import sys

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "deyes_stereo"))

from deyes_stereo.physical_stereo_calibration import (  # noqa: E402
    board_descriptor,
    command_compute,
    is_duplicate_pose,
    load_observations,
    parser,
    rectified_epipolar_errors,
)
from deyes_stereo.stereo_calibration_contract import (  # noqa: E402
    CALIBRATION_SIZE,
    DEFAULT_BOARD_INNER_CORNERS,
    coverage_cells,
    coverage_complete,
    validate_capture_arguments,
    validation_gate,
)


def test_capture_rejects_guessed_square_size_wrong_resolution_and_sample_count() -> None:
    assert validate_capture_arguments(
        square_size_m=0.0, requested_samples=30, width=1280, height=720, board_inner_corners=(3, 7)
    ) == (
        "square_size_m_must_be_explicit_and_positive",
        "requested_samples_must_be_between_40_and_60",
        "resolution_must_be_exactly_640x360",
        "board_inner_corners_must_be_explicit_integers_at_least_4x4",
    )
    assert validate_capture_arguments(
        square_size_m=float("nan"), requested_samples=50, width=640, height=360,
        board_inner_corners=DEFAULT_BOARD_INNER_CORNERS,
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
        board_inner_corners=DEFAULT_BOARD_INNER_CORNERS,
        square_size_m=0.031,
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
        board_inner_corners=(9, 6),
        square_size_m=0.031,
    )
    assert not gate.validated
    assert set(gate.reasons) == {
        "resolution_not_640x360",
        "source_is_not_physical_checkerboard",
        "reproj_rms_exceeds_0_50_px",
        "epipolar_p95_exceeds_0_50_px",
    }


def test_board_shape_is_required_for_capture_and_validation() -> None:
    assert validate_capture_arguments(
        square_size_m=0.031, requested_samples=50, width=640, height=360, board_inner_corners=(8, 7)
    ) == ()
    gate = validation_gate(
        sample_count=50, resolution=CALIBRATION_SIZE, reproj_rms_px=0.2, epipolar_p95_px=0.2,
        source="physical_checkerboard", left_right_confirmed=True, baseline_sign_confirmed=True,
        scale_confirmed=True, coverage_complete=True, board_inner_corners=(8, 3), square_size_m=0.031,
    )
    assert not gate.validated
    assert gate.reasons == ("board_inner_corners_must_be_explicit_integers_at_least_4x4",)
    metric_gate = validation_gate(
        sample_count=50, resolution=CALIBRATION_SIZE, reproj_rms_px=float("nan"), epipolar_p95_px=float("nan"),
        source="physical_checkerboard", left_right_confirmed=True, baseline_sign_confirmed=True,
        scale_confirmed=True, coverage_complete=True, board_inner_corners=(8, 7), square_size_m=0.031,
    )
    assert set(metric_gate.reasons) == {"reproj_rms_exceeds_0_50_px", "epipolar_p95_exceeds_0_50_px"}


def test_observation_loader_rejects_sample_board_mixed_with_session(tmp_path: Path) -> None:
    manifest = {
        "samples": [{
            "board_inner_corners": [8, 7], "pair_skew_ms": 0.0, "left": "left.png", "right": "right.png",
        }],
    }
    try:
        load_observations(tmp_path, manifest, (9, 6))
    except ValueError as error:
        assert str(error) == "sample_board_does_not_match_capture_session_board"
    else:  # pragma: no cover - assertion path documents the fail-closed contract.
        raise AssertionError("mixed board sample was accepted")


def test_compute_rejects_legacy_8x7_manifest_for_official_9x6_session(tmp_path: Path) -> None:
    (tmp_path / "capture_manifest.json").write_text(json.dumps({
        "source": "ros_topics",
        "left_topic": "/x1/left_camera/image_raw",
        "right_topic": "/x1/right_camera/image_raw",
        "board_inner_corners": [8, 7],
        "square_size_m": 0.031,
        "resolution": [640, 360],
        "samples": [],
    }), encoding="utf-8")
    args = argparse.Namespace(
        session_dir=str(tmp_path), robot_id="robot", camera_pair_id="pair", square_size_m=0.031,
        board_cols=9, board_rows=6, confirm_left_right=True, confirm_baseline_sign=True, confirm_scale=True,
    )
    try:
        command_compute(args)
    except ValueError as error:
        assert str(error) == "compute_board_does_not_match_capture_session"
    else:  # pragma: no cover - assertion path documents the fail-closed contract.
        raise AssertionError("legacy board session was accepted")


def test_parser_defaults_to_the_formal_9x6_board_but_requires_a_square_measurement() -> None:
    args = parser().parse_args(["capture", "--session-dir", "session", "--square-size-m", "0.031"])
    assert (args.board_cols, args.board_rows, args.square_size_m) == (9, 6, 0.031)
    compute_args = parser().parse_args([
        "compute", "--session-dir", "session", "--robot-id", "robot", "--camera-pair-id", "pair",
        "--square-size-m", "0.031",
    ])
    assert (compute_args.board_cols, compute_args.board_rows, compute_args.square_size_m) == (9, 6, 0.031)
    with pytest.raises(SystemExit):
        parser().parse_args(["capture", "--session-dir", "session"])
    with pytest.raises(SystemExit):
        parser().parse_args([
            "compute", "--session-dir", "session", "--robot-id", "robot", "--camera-pair-id", "pair",
        ])


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
