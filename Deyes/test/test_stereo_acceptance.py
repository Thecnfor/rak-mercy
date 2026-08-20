"""ROS-free regression tests for the fixed Phase 4 acceptance thresholds."""

from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "deyes_stereo"))

from deyes_stereo.stereo_acceptance import (  # noqa: E402
    AcceptanceInputError,
    evaluate_runtime_metrics,
    evaluate_truth_samples,
    percentile95,
)
from deyes_stereo.runtime_acceptance_monitor import _rate  # noqa: E402


def samples(distance: float, error: float, count: int = 100) -> list[dict[str, object]]:
    return [
        {"truth_m": distance, "measured_m": distance + error, "valid": True, "plane_residual_m": 0.001}
        for _ in range(count)
    ]


def valid_truth_data() -> list[dict[str, object]]:
    return samples(0.30, 0.009) + samples(0.50, -0.009) + samples(0.80, 0.019) + samples(1.00, -0.019)


def valid_runtime_data() -> dict[str, object]:
    return {
        "duration_sec": 600.0,
        "capture_failures": 0,
        "pair_max_skew_ms": 10.0,
        "left_image_hz": 28.0,
        "right_image_hz": 28.0,
        "depth_hz": 12.0,
        "points_hz": 12.0,
        "center_roi_coverage_min": 0.85,
        "processing_overrun_sustained": False,
        "pair_diagnostics_observed": True,
        "depth_status_observed": True,
        "depth_coverage_observed": True,
        "calibration_validated": True,
        "calibration_id": "physical-x1-pair-20260820",
        "points_status_observed": True,
        "pointcloud_status_always_validated": True,
        "pointcloud_calibration_identity_consistent": True,
        "rviz_manual_checks": {
            "flat_plane_has_no_obvious_warping_or_layering": True,
            "no_obvious_ghosting": True,
            "optical_axes_are_x_right_y_down_z_forward": True,
        },
    }


def test_nearest_rank_p95_is_reproducible() -> None:
    assert percentile95(list(range(1, 101))) == 95


def test_all_four_truth_distances_pass_at_the_fixed_boundaries() -> None:
    data = valid_truth_data()
    data[0]["plane_residual_m"] = 0.002
    report = evaluate_truth_samples(data)
    assert report["overall_validated"]
    assert [group["truth_m"] for group in report["groups"]] == [0.30, 0.50, 0.80, 1.00]
    assert report["groups"][0]["plane_residual_m"]["count"] == 100


def test_insufficient_samples_and_threshold_failure_cannot_pass() -> None:
    report = evaluate_truth_samples(samples(0.30, 0.011, 99) + samples(0.50, 0.0) + samples(0.80, 0.0) + samples(1.00, 0.0))
    assert not report["overall_validated"]
    assert "0.30m:insufficient_samples" in report["reasons"]
    assert "0.30m:mae_exceeds_limit" in report["reasons"]


def test_truth_cannot_pass_with_invalid_or_plane_less_measurements() -> None:
    data = valid_truth_data()
    data[0]["valid"] = False
    data[0].pop("plane_residual_m")
    report = evaluate_truth_samples(data)
    assert not report["overall_validated"]
    assert "0.30m:insufficient_valid_measurements" in report["reasons"]
    data = valid_truth_data()
    data[0].pop("plane_residual_m")
    report = evaluate_truth_samples(data)
    assert not report["overall_validated"]
    assert "0.30m:plane_residual_missing_for_valid_measurements" in report["reasons"]


def test_bad_truth_field_is_rejected_instead_of_counted_as_a_zero() -> None:
    with pytest.raises(AcceptanceInputError, match="missing_measured"):
        evaluate_truth_samples([{"truth_m": 0.30, "valid": True}])


def test_runtime_gate_requires_every_fixed_requirement() -> None:
    report = evaluate_runtime_metrics(valid_runtime_data())
    assert report["overall_validated"]
    bad = valid_runtime_data()
    bad.update({"pair_max_skew_ms": 10.001, "processing_overrun_sustained": True})
    failed = evaluate_runtime_metrics(bad)
    assert not failed["overall_validated"]
    assert set(failed["reasons"]) == {"every_static_pair_at_most_10ms", "no_sustained_processing_overrun"}


def test_runtime_cannot_pass_without_physical_calibration_or_rviz_confirmation() -> None:
    metrics = valid_runtime_data()
    metrics["calibration_validated"] = False
    metrics["calibration_id"] = "unassigned"
    metrics["rviz_manual_checks"] = {}
    report = evaluate_runtime_metrics(metrics)
    assert not report["overall_validated"]
    assert "physical_calibration_validated" in report["reasons"]
    assert "rviz_no_obvious_ghosting" in report["reasons"]


def test_runtime_cannot_pass_without_observed_diagnostics_or_consistent_pointcloud_identity() -> None:
    metrics = valid_runtime_data()
    metrics.update({
        "pair_diagnostics_observed": False,
        "points_status_observed": False,
        "pointcloud_status_always_validated": False,
        "pointcloud_calibration_identity_consistent": False,
    })
    report = evaluate_runtime_metrics(metrics)
    assert not report["overall_validated"]
    assert "pair_diagnostics_observed" in report["reasons"]
    assert "points_status_observed" in report["reasons"]
    assert "pointcloud_calibration_identity_consistent" in report["reasons"]


def test_rate_uses_the_entire_observation_window_so_a_stopped_stream_fails() -> None:
    # 30 Hz for only the first five minutes of a ten-minute acceptance run.
    assert _rate([float(index) / 30.0 for index in range(30 * 300)], 600.0) == 15.0
