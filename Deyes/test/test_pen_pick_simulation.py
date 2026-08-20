import json

import pytest

from deyes_stereo.pen_pick_dry_run_contract import PickPlanLimits
from deyes_stereo.pen_pick_simulation import FakeGripperAdapter, FakeMercuryDualArmAdapter, FakeNav2Adapter, SimulationTimeouts, TRACE_VERSION, run_pick_simulation, write_simulation_trace


NOW = 40_000_000_000
LIMITS = PickPlanLimits(workspace_min_base_m=(.1, -.4, .05), workspace_max_base_m=(.9, .4, .7))


def _candidate(**changes):
    candidate = {"valid": True, "trusted_for_grasp": True, "target_id": "synthetic-pen", "target_frame": "base_link", "grasp_point_base_m": [.45, .0, .2], "axis_base_unit": [1., 0., 0.], "approach_normal_base_unit": [0., 0., 1.], "quality": {"detection_confidence": .95, "depth_mad_m": .003, "mask_depth_valid_ratio": .9}}
    candidate.update(changes)
    return {"stamp_sec": 39, "stamp_nanosec": 800_000_000, "valid": True, "trusted_for_grasp": True, "candidate_count": 1, "candidates": [candidate]}


def _run(**kwargs):
    return run_pick_simulation(_candidate(), now_stamp_ns=NOW, limits=LIMITS, **kwargs)


def test_synthetic_candidate_replay_completes_and_emits_machine_readable_trace():
    trace = _run()
    assert trace["trace_version"] == TRACE_VERSION
    assert trace["terminal_state"] == "succeeded"
    assert trace["hardware_commands_emitted"] is False
    assert [item["step"] for item in trace["events"]] == ["pre_grasp", "approach", "grasp", "close_gripper", "lift", "safe_retreat"]
    assert json.loads(json.dumps(trace))["target_id"] == "synthetic-pen"


@pytest.mark.parametrize("nav_outcome,expected", [("rejected", "rejected"), ("timed_out", "timed_out")])
def test_navigation_failure_stops_before_any_arm_or_gripper_call(nav_outcome, expected):
    arm, gripper = FakeMercuryDualArmAdapter(), FakeGripperAdapter()
    trace = _run(nav=FakeNav2Adapter(nav_outcome), arm=arm, gripper=gripper, include_navigation_gate=True)
    assert trace["reason"] == expected
    assert [event["step"] for event in trace["events"]] == ["verify_navigation_arrival"]
    assert not arm.calls and not gripper.calls


def test_unreachable_arm_and_gripper_failure_stop_the_remaining_sequence():
    arm = FakeMercuryDualArmAdapter({"approach": "unreachable"})
    trace = _run(arm=arm)
    assert trace["failed_step"] == "approach"
    assert [event["step"] for event in trace["events"]] == ["pre_grasp", "approach"]
    trace = _run(gripper=FakeGripperAdapter("failed"))
    assert trace["failed_step"] == "close_gripper"
    assert [event["step"] for event in trace["events"]][-1] == "close_gripper"


def test_stale_calibration_confirmation_and_cancel_all_fail_closed():
    stale = run_pick_simulation(_candidate(), now_stamp_ns=NOW + 1_000_000_000, limits=LIMITS)
    assert stale["events"] == [] and stale["terminal_state"] == "failed"
    invalid = _candidate(trusted_for_grasp=False)
    assert run_pick_simulation(invalid, now_stamp_ns=NOW, limits=LIMITS)["events"] == []
    unapproved = _run(operator_approved=False)
    assert unapproved["reason"] == "operator_approval_missing" and unapproved["events"] == []
    cancelled = _run(cancel_at_step="approach")
    assert cancelled["terminal_state"] == "cancelled"
    assert [event["step"] for event in cancelled["events"]] == ["pre_grasp", "approach"]


def test_single_selected_arm_is_recorded_and_invalid_arm_stops_before_gripper():
    arm = FakeMercuryDualArmAdapter()
    trace = _run(arm=arm, selected_arm="right")
    assert trace["terminal_state"] == "succeeded"
    assert {call["selected_arm"] for call in arm.calls} == {"right"}
    invalid = _run(selected_arm="both")
    assert invalid["failed_step"] == "pre_grasp"


def test_invalid_timeout_fails_on_its_first_step():
    trace = _run(timeouts=SimulationTimeouts(arm_motion_sec=0.0))
    assert trace["reason"] == "timeout_invalid"
    assert [event["step"] for event in trace["events"]] == ["pre_grasp"]


def test_trace_writer_refuses_repository_paths_and_accepts_external_temp_root(tmp_path):
    trace = _run()
    with pytest.raises(ValueError, match="temp_deyes"):
        write_simulation_trace(trace, "E:/a_robot/rak-mercy/trace.json")
    output = write_simulation_trace(trace, tmp_path / "trace.json", temp_root=tmp_path)
    assert json.loads(output.read_text(encoding="utf-8"))["terminal_state"] == "succeeded"
