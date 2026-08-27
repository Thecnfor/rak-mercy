import pytest

from deyes_stereo.competition_fullchain_sim import (
    CompetitionFullChain,
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
