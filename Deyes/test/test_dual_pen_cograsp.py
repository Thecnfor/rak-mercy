import json
from pathlib import Path

import pytest

from deyes_stereo.dual_pen_cograsp_contract import DualPenCograspSiteProfile, WorkspaceBounds, build_dual_pen_cograsp_plan, validate_dual_pen_cograsp_candidate
from deyes_stereo.dual_pen_cograsp_simulation import FakeDualArmGripperAdapter, ManualClock, SimulationTimeouts, run_dual_pen_cograsp_simulation, trace_json


def profile(**changes):
    values = dict(
        validated=True,
        left_workspace=WorkspaceBounds(-.3, .3, .001, .3, .1, .6),
        right_workspace=WorkspaceBounds(-.3, .3, -.3, -.001, .1, .6),
        max_candidate_age_ns=1000,
        max_barrier_skew_sec=.1,
        lift_vector_base_unit=(0, 0, 1),
    )
    values.update(changes)
    return DualPenCograspSiteProfile(**values)


def candidate(**changes):
    values = dict(valid=True, trusted_for_grasp=True, target_frame="base_link", target_id="pen-1", confidence=.95, stamp_ns=100, axis_base_unit=[1, 0, 0], approach_normal_base_unit=[0, 0, 1], grasp_interval_base_m=[[.05, -.07, .30], [.05, .07, .30]])
    values.update(changes)
    return values


def test_plan_is_no_nav_and_assigns_higher_y_to_left():
    plan = build_dual_pen_cograsp_plan(candidate(), now_stamp_ns=200, profile=profile())
    assert plan["state"] == "ready"
    assert plan["navigation_included"] is False
    assert [step["phase"] for step in plan["steps"]] == ["pregrasp", "approach", "contact", "close", "confirm", "lift", "hold"]
    assert plan["assignments"]["left"][1] > plan["assignments"]["right"][1]
    assert all(step["commands_emitted"] is False for step in plan["steps"])


@pytest.mark.parametrize("payload,reason", [
    ({"candidates": []}, "candidate_count_zero"),
    ({"candidates": [candidate(), candidate(target_id="p2")]}, "candidate_count_must_be_exactly_one"),
    (candidate(trusted_for_grasp=False), "candidate_not_validated_or_trusted"),
    (candidate(stamp_ns=1), "candidate_stale_or_stamp_missing"),
    (candidate(stamp_ns=201), "candidate_stale_or_stamp_missing"),
    (candidate(target_frame="camera_link"), "candidate_frame_must_be_base_link"),
    (candidate(grasp_interval_base_m=[[0, 0, .3], [float("nan"), .1, .3]]), "grasp_interval_base_m[1]_must_be_finite"),
])
def test_candidate_rejections_fail_closed(payload, reason):
    result = validate_dual_pen_cograsp_candidate(payload, now_stamp_ns=2_000 if payload.get("stamp_ns") == 1 else 200, profile=profile())
    assert result["valid"] is False and result["reason"] == reason and result["commands_emitted"] is False


@pytest.mark.parametrize("points,reason,custom", [
    ([[0, -.01, .3], [0, .01, .3]], "contact_distance_too_small", {}),
    ([[0, -.2, .3], [0, .2, .3]], "contact_distance_too_large", {}),
    ([[0, -.01, .3], [0, .01, .3]], "tool_point_clearance_insufficient", {"min_contact_distance_m": .01, "min_tool_point_clearance_m": .03}),
    ([[.20, -.07, .3], [.05, .07, .3]], "right_workspace_out_of_bounds", {"max_contact_distance_m": .30, "right_workspace": WorkspaceBounds(-.3, .1, -.3, -.001, .1, .6)}),
])
def test_contact_geometry_and_workspace_are_checked(points, reason, custom):
    assert validate_dual_pen_cograsp_candidate(candidate(grasp_interval_base_m=points), now_stamp_ns=200, profile=profile(**custom))["reason"] == reason


def test_default_profile_is_frozen_and_fail_closed():
    assert validate_dual_pen_cograsp_candidate(candidate(), now_stamp_ns=200)["reason"] == "site_profile_not_validated"
    with pytest.raises(Exception):
        DualPenCograspSiteProfile().validated = True


def test_one_candidate_envelope_inherits_root_trust_and_stamp_evidence():
    envelope = {"valid": True, "trusted_for_grasp": True, "stamp_sec": 1, "nanosec": 2, "candidate_count": 1, "candidates": [candidate(stamp_ns=9)]}
    result = validate_dual_pen_cograsp_candidate(envelope, now_stamp_ns=1_000_000_003, profile=profile())
    assert result["valid"] is True and result["source_stamp_ns"] == 1_000_000_002


@pytest.mark.parametrize("root_trust,nested_trust", [(True, False), (False, True)])
def test_envelope_never_overwrites_negative_trust_evidence(root_trust, nested_trust):
    envelope = {"valid": True, "trusted_for_grasp": root_trust, "stamp_ns": 100, "candidate_count": 1, "candidates": [candidate(trusted_for_grasp=nested_trust)]}
    assert validate_dual_pen_cograsp_candidate(envelope, now_stamp_ns=200, profile=profile())["reason"] == "candidate_not_validated_or_trusted"


def test_missing_lift_vector_and_equal_y_fail_closed():
    assert validate_dual_pen_cograsp_candidate(candidate(), now_stamp_ns=200, profile=profile(lift_vector_base_unit=None))["reason"] == "lift_vector_base_unit_missing"
    assert validate_dual_pen_cograsp_candidate(candidate(grasp_interval_base_m=[[.05, 0, .3], [.08, 0, .3]]), now_stamp_ns=200, profile=profile(min_contact_distance_m=.01))[
        "reason"
    ] == "left_right_assignment_ambiguous_equal_y"


def test_approach_and_lift_use_independent_vectors_and_all_pose_workspaces_are_checked():
    tilted = candidate(approach_normal_base_unit=[0, 1, 0])
    plan = build_dual_pen_cograsp_plan(tilted, now_stamp_ns=200, profile=profile(lift_vector_base_unit=(0, 0, 1)))
    assert plan["steps"][0]["left"]["tool_point_m"] == pytest.approx([.05, .12, .30])
    assert plan["steps"][1]["left"]["tool_point_m"] == pytest.approx([.05, .09, .30])
    assert plan["steps"][5]["left"]["tool_point_m"] == pytest.approx([.05, .07, .34])
    too_low = profile(left_workspace=WorkspaceBounds(-.3, .3, .001, .3, .1, .32))
    assert build_dual_pen_cograsp_plan(candidate(), now_stamp_ns=200, profile=too_low)["reason"] == "left_pregrasp_workspace_out_of_bounds"


def test_object_basis_is_shared_orthonormal_right_handed_and_parallel_axis_is_rejected():
    plan = build_dual_pen_cograsp_plan(candidate(), now_stamp_ns=200, profile=profile())
    basis = plan["steps"][0]["left"]["object_basis_base_columns"]
    assert basis == plan["steps"][0]["right"]["object_basis_base_columns"]
    axis, lateral, outbound = basis
    dot = lambda first, second: sum(a * b for a, b in zip(first, second))
    cross = lambda first, second: [first[1] * second[2] - first[2] * second[1], first[2] * second[0] - first[0] * second[2], first[0] * second[1] - first[1] * second[0]]
    assert dot(axis, lateral) == pytest.approx(0) and dot(axis, outbound) == pytest.approx(0) and dot(lateral, outbound) == pytest.approx(0)
    assert cross(axis, lateral) == pytest.approx(outbound)
    assert "validated_tool_transform" in plan["steps"][0]["left"]["tcp_adapter_contract"]
    assert validate_dual_pen_cograsp_candidate(candidate(axis_base_unit=[0, 0, 1]), now_stamp_ns=200, profile=profile())["reason"] == "pen_axis_parallel_to_approach_normal"
    assert validate_dual_pen_cograsp_candidate(candidate(axis_base_unit=[2, 0, 0]), now_stamp_ns=200, profile=profile())["reason"] == "axis_base_unit_must_be_unit"


@pytest.mark.parametrize("changes,field", [
    ({"min_confidence": 1.1}, "min_confidence"),
    ({"max_candidate_age_ns": 0}, "max_candidate_age_ns"),
    ({"min_contact_distance_m": .2, "max_contact_distance_m": .1}, "contact_distance_range"),
    ({"approach_offset_m": .05}, "approach_offset_m"),
    ({"left_workspace": WorkspaceBounds(0, 0, .001, .3, .1, .6)}, "left_workspace_x"),
])
def test_invalid_site_profile_values_fail_closed_stably(changes, field):
    assert validate_dual_pen_cograsp_candidate(candidate(), now_stamp_ns=200, profile=profile(**changes))["reason"] == f"site_profile_invalid:{field}"


def test_zero_minimum_confidence_is_a_valid_configured_threshold():
    assert validate_dual_pen_cograsp_candidate(candidate(confidence=0), now_stamp_ns=200, profile=profile(min_confidence=0))["valid"] is True


def test_success_trace_is_serializable_and_has_stable_fields():
    trace = run_dual_pen_cograsp_simulation(candidate(), now_stamp_ns=200, profile=profile())
    assert trace["terminal_state"] == "succeeded"
    assert trace["event_sequence"] == ["pregrasp", "approach", "contact", "close", "confirm", "lift", "hold"]
    assert trace["commands_emitted"] is False
    assert json.loads(trace_json(trace))["schema"] == "dual_pen_cograsp_trace/v1"
    assert {"state_from", "state_to", "left", "right", "barrier_skew_sec", "deadline_sec", "failure_code"} <= set(trace["events"][0])


@pytest.mark.parametrize("phase", ["pregrasp", "approach", "contact", "close"])
def test_single_side_early_phase_failure_cancels_and_never_dispatches_later_phase(phase):
    adapter = FakeDualArmGripperAdapter({f"left:{phase}": "failed"})
    trace = run_dual_pen_cograsp_simulation(candidate(), now_stamp_ns=200, profile=profile(), adapter=adapter)
    assert trace["failed_phase"] == phase
    later = ["pregrasp", "approach", "contact", "close", "confirm", "lift", "hold"]
    assert not any(call.get("phase") in later[later.index(phase) + 1:] for call in adapter.calls)
    assert {call["side"] for call in adapter.calls if call["op"] == "cancel"} == {"left", "right"}


@pytest.mark.parametrize("phase", ["lift", "hold"])
def test_lift_or_hold_failure_locks_for_manual_intervention(phase):
    adapter = FakeDualArmGripperAdapter({f"right:{phase}": "failed"})
    trace = run_dual_pen_cograsp_simulation(candidate(), now_stamp_ns=200, profile=profile(), adapter=adapter)
    assert trace["terminal_state"] == "locked_manual_intervention"
    assert len([call for call in adapter.calls if call["op"] == "stop_unverified"]) == 2


def test_close_without_two_explicit_grasp_confirmations_never_lifts():
    adapter = FakeDualArmGripperAdapter({"left:close": {"result": "succeeded", "grasp_confirmed": False}})
    trace = run_dual_pen_cograsp_simulation(candidate(), now_stamp_ns=200, profile=profile(), adapter=adapter)
    assert trace["failure_code"] == "grasp_confirmation_missing"
    assert not any(call.get("phase") == "lift" for call in adapter.calls)


def test_timeout_cancel_race_and_skew_limit_are_visible():
    timeout = FakeDualArmGripperAdapter({"right:approach": {"delay_sec": 9}})
    timed = run_dual_pen_cograsp_simulation(candidate(), now_stamp_ns=200, profile=profile(), adapter=timeout)
    assert timed["failure_code"] == "approach_deadline_exceeded"
    raced = run_dual_pen_cograsp_simulation(candidate(), now_stamp_ns=200, profile=profile(), adapter=FakeDualArmGripperAdapter({"left:contact": "cancelled"}))
    assert raced["failure_code"] == "contact_cancelled"
    skewed = run_dual_pen_cograsp_simulation(candidate(), now_stamp_ns=200, profile=profile(), adapter=FakeDualArmGripperAdapter({"right:pregrasp": {"delay_sec": .2}}))
    assert skewed["failure_code"] == "barrier_skew_exceeded"


def test_adapter_has_manual_time_and_independent_side_acceptance_feedback_result():
    clock = ManualClock(2)
    adapter = FakeDualArmGripperAdapter({"left:pregrasp": {"accepted": False}, "right:pregrasp": {"feedback": "ready"}}, clock=clock)
    trace = run_dual_pen_cograsp_simulation(candidate(), now_stamp_ns=200, profile=profile(), adapter=adapter)
    assert trace["failure_code"] == "pregrasp_goal_rejected"
    assert any(c["op"] == "feedback" and c["side"] == "right" for c in adapter.calls)


def test_lost_lift_feedback_locks_and_stops_unverified_without_hold():
    adapter = FakeDualArmGripperAdapter({"left:lift": {"feedback": "missing"}})
    trace = run_dual_pen_cograsp_simulation(candidate(), now_stamp_ns=200, profile=profile(), adapter=adapter)
    assert trace["terminal_state"] == "locked_manual_intervention"
    assert trace["failure_code"] == "lift_feedback_missing"
    assert not any(call.get("phase") == "hold" for call in adapter.calls)
    assert len([call for call in adapter.calls if call["op"] == "stop_unverified"]) == 2


def test_hold_advances_manual_time_and_rejects_short_timeout_or_early_success():
    clock = ManualClock()
    trace = run_dual_pen_cograsp_simulation(candidate(), now_stamp_ns=200, profile=profile(hold_sec=2.5), adapter=FakeDualArmGripperAdapter(clock=clock))
    assert trace["terminal_state"] == "succeeded" and clock.now_sec >= 2.5
    timeout = run_dual_pen_cograsp_simulation(candidate(), now_stamp_ns=200, profile=profile(hold_sec=2), timeouts=SimulationTimeouts(hold_sec=1))
    assert timeout["terminal_state"] == "locked_manual_intervention" and timeout["failure_code"] == "hold_duration_not_met"
    early = run_dual_pen_cograsp_simulation(candidate(), now_stamp_ns=200, profile=profile(hold_sec=2), adapter=FakeDualArmGripperAdapter({"left:hold": {"delay_sec": .5}}))
    assert early["terminal_state"] == "locked_manual_intervention" and early["failure_code"] == "hold_duration_not_met"


def test_stop_unverified_always_locks_and_cancel_is_distinct_terminal_state():
    stopped_adapter = FakeDualArmGripperAdapter({"left:approach": "stop_unverified"})
    stopped = run_dual_pen_cograsp_simulation(candidate(), now_stamp_ns=200, profile=profile(), adapter=stopped_adapter)
    assert stopped["terminal_state"] == "locked_manual_intervention"
    assert len([call for call in stopped_adapter.calls if call["op"] == "stop_unverified"]) == 2
    assert not any(call.get("phase") == "contact" for call in stopped_adapter.calls)
    cancelled = run_dual_pen_cograsp_simulation(candidate(), now_stamp_ns=200, profile=profile(), adapter=FakeDualArmGripperAdapter({"right:contact": "cancelled"}))
    assert cancelled["terminal_state"] == "cancelled" and cancelled["failure_code"] == "contact_cancelled"


def test_cograsp_core_has_no_ros_vendor_or_motion_command_dependency():
    package = Path(__file__).resolve().parents[1] / "src" / "deyes_stereo" / "deyes_stereo"
    source = "\n".join((package / name).read_text(encoding="utf-8") for name in (
        "dual_pen_cograsp_contract.py", "dual_pen_cograsp_simulation.py",
    ))
    for forbidden in ("rclpy", "pymycobot", "send_angles", "joint_states", "ActionClient"):
        assert forbidden not in source
