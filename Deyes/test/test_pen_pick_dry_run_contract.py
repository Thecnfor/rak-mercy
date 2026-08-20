import numpy as np

from deyes_stereo.pen_pick_dry_run_contract import PickDryRunStateMachine, PickPlanLimits, build_dry_run_plan


NOW = 20_000_000_000
LIMITS = PickPlanLimits(workspace_min_base_m=(0.10, -0.40, 0.05), workspace_max_base_m=(0.90, 0.40, 0.70))


def _payload(**candidate_changes):
    candidate = {
        "valid": True, "trusted_for_grasp": True, "target_id": "pen-01", "target_frame": "base_link",
        "grasp_point_base_m": [.45, .0, .20], "axis_base_unit": [1., 0., 0.],
        "approach_normal_base_unit": [0., 0., 1.],
        "quality": {"detection_confidence": .91, "depth_mad_m": .004, "mask_depth_valid_ratio": .80},
    }
    candidate.update(candidate_changes)
    return {"stamp_sec": 19, "stamp_nanosec": 800_000_000, "valid": True, "trusted_for_grasp": True, "candidate_count": 1, "candidates": [candidate]}


def test_valid_candidate_generates_pregrasp_to_retreat_intents_but_no_commands():
    result = build_dry_run_plan(_payload(), now_stamp_ns=NOW, limits=LIMITS)
    assert result["state"] == "dry_run_ready"
    assert result["commands_emitted"] is False
    assert [step["name"] for step in result["steps"]] == ["verify_navigation_arrival", "pre_grasp", "approach", "grasp", "close_gripper", "lift", "safe_retreat"]
    pre = result["steps"][1]["pose"]["position_m"]
    assert np.allclose(pre, [.45, 0., .32])
    assert "motion_adapter_not_implemented" in result["execution_block_reasons"]


def test_multiple_or_untrusted_candidates_are_rejected_before_planning():
    bad = _payload()
    bad["candidate_count"] = 2
    assert build_dry_run_plan(bad, now_stamp_ns=NOW, limits=LIMITS)["reason"] == "candidate_count_must_be_one"
    bad = _payload(trusted_for_grasp=False)
    assert build_dry_run_plan(bad, now_stamp_ns=NOW, limits=LIMITS)["reason"] == "candidate_not_validated_for_grasp"


def test_stale_quality_and_workspace_gates_fail_closed():
    assert build_dry_run_plan(_payload(), now_stamp_ns=NOW + 1_000_000_000, limits=LIMITS)["reason"] == "candidate_timestamp_stale_or_invalid"
    assert build_dry_run_plan(_payload(quality={"detection_confidence": .2, "depth_mad_m": .004, "mask_depth_valid_ratio": .8}), now_stamp_ns=NOW, limits=LIMITS)["reason"] == "detection_confidence_below_limit"
    assert build_dry_run_plan(_payload(), now_stamp_ns=NOW, limits=PickPlanLimits())["reason"] == "workspace_bounds_not_configured"


def test_enable_execution_cannot_bypass_missing_motion_adapter():
    result = build_dry_run_plan(_payload(), now_stamp_ns=NOW, limits=LIMITS, site_profile_validated=True, enable_execution=True, operator_approved=True)
    assert result["execution_eligible"] is False
    assert result["execution_block_reasons"] == ["motion_adapter_not_implemented"]


def test_state_machine_records_rejection_then_dry_run_ready_without_motion():
    machine = PickDryRunStateMachine()
    rejected = machine.transition({}, now_stamp_ns=NOW, limits=LIMITS)
    ready = machine.transition(_payload(), now_stamp_ns=NOW, limits=LIMITS)
    assert rejected["state"] == "rejected"
    assert ready["previous_state"] == "rejected"
    assert ready["state"] == "dry_run_ready"
    assert ready["commands_emitted"] is False
