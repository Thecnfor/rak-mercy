import pytest

from deyes_stereo.competition_fullchain_sim import (
    CompetitionFullChain,
    FAIL_CLOSED_FAULTS,
    SimulatedCompetitionAdapter,
)


EXPECTED_PHASES = [
    "navigate_table_1",
    "stabilize_table_1",
    "stereo_snapshot",
    "cuda_depth_camera_info",
    "table_plane_audit",
    "single_pen_detection",
    "project_target",
    "validate_ik",
    "pick",
    "verify_grasp",
    "navigate_table_2",
    "place",
    "retreat",
]


def test_synthetic_fullchain_runs_once_and_keeps_physical_claims_false():
    adapter = SimulatedCompetitionAdapter()

    result = CompetitionFullChain(adapter).run()

    assert result["state"] == "completed"
    assert [event["phase"] for event in result["events"]] == EXPECTED_PHASES
    assert all(event["attempt"] == 1 for event in result["events"])
    assert result["synthetic_inputs"] is True
    assert result["physical_validated"] is False
    assert result["hardware_commands_emitted"] is False
    assert result["scene"]["table_height_m"] == pytest.approx(0.650)
    assert result["scene"]["pen_count"] == 1
    assert result["target"]["right_arm_sdk_target_m"][2] == pytest.approx(0.135)
    assert result["trajectory"]["transport"]["position_residual_mm"] <= 5.0
    assert result["trajectory"]["transport"]["joint_limits_passed"] is True
    assert result["trajectory"]["transport"]["minimum_clearance_mm"] >= 50.0
    assert result["grasp_verification"]["navigation_permitted"] is True


@pytest.mark.parametrize(
    ("fault", "failed_phase"),
    [
        ("navigation_table_1_failed", "navigate_table_1"),
        ("stale_frame", "stereo_snapshot"),
        ("zero_pen", "single_pen_detection"),
        ("multiple_pens", "single_pen_detection"),
        ("invalid_depth", "cuda_depth_camera_info"),
        ("table_plane_conflict", "table_plane_audit"),
        ("target_out_of_bounds", "project_target"),
        ("ik_failed", "validate_ik"),
        ("grasp_verification_failed", "verify_grasp"),
        ("navigation_table_2_failed", "navigate_table_2"),
    ],
)
def test_faults_fail_closed_without_retry_or_later_actions(fault, failed_phase):
    adapter = SimulatedCompetitionAdapter(fault=fault)

    result = CompetitionFullChain(adapter).run()

    assert result["state"] == "failed"
    assert result["failed_phase"] == failed_phase
    assert max(event["attempt"] for event in result["events"]) == 1
    assert adapter.calls == [event["phase"] for event in result["events"]]
    if failed_phase != "navigate_table_2":
        assert "navigate_table_2" not in adapter.calls
    assert "place" not in adapter.calls


def test_fixed_xy_requires_explicit_force_flag_even_in_simulation():
    rejected = CompetitionFullChain(
        SimulatedCompetitionAdapter(fault="projector_unavailable")
    ).run()
    forced = CompetitionFullChain(
        SimulatedCompetitionAdapter(fault="projector_unavailable"),
        force_fixed_target=True,
    ).run()

    assert rejected["state"] == "failed"
    assert rejected["reason"] == "projector_unavailable"
    assert forced["state"] == "completed"
    assert forced["target"]["selection_source"] == "forced_fixed_xy"
    assert forced["target"]["force_fixed_target"] is True
    assert forced["target"]["physical_validated"] is False


@pytest.mark.parametrize("fault", ["plane_missing", "plane_low_quality"])
def test_missing_or_low_quality_plane_warns_but_keeps_fixed_650mm(fault):
    result = CompetitionFullChain(SimulatedCompetitionAdapter(fault=fault)).run()

    assert result["state"] == "completed"
    assert result["scene"]["table_height_m"] == pytest.approx(0.650)
    assert result["plane_audit"]["state"] == "fixed_height_unverified"
    assert fault in result["plane_audit"]["warning"]
    assert result["retry_count"] == 0


def test_fault_catalog_covers_every_fail_closed_injection():
    assert set(FAIL_CLOSED_FAULTS) == {
        "navigation_table_1_failed",
        "stale_frame",
        "zero_pen",
        "multiple_pens",
        "invalid_depth",
        "table_plane_conflict",
        "projector_unavailable",
        "target_out_of_bounds",
        "ik_failed",
        "grasp_verification_failed",
        "navigation_table_2_failed",
    }
