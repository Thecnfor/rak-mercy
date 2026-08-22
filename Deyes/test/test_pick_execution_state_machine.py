from deyes_stereo.pen_pick_dry_run_contract import PickPlanLimits, build_dry_run_plan
from deyes_stereo.pick_execution_state_machine import FakePickBackend, MercurySafetyPickBackend, PickTimeouts, run_pick_state_machine


NOW = 20_000_000_000
LIMITS = PickPlanLimits(workspace_min_base_m=(.1, -.4, .05), workspace_max_base_m=(.9, .4, .7))


def _plan():
    candidate = {"valid": True, "trusted_for_grasp": True, "target_id": "p", "target_frame": "base_link", "grasp_point_base_m": [.45, 0, .2], "axis_base_unit": [1, 0, 0], "approach_normal_base_unit": [0, 0, 1], "quality": {"detection_confidence": .9, "depth_mad_m": .002, "mask_depth_valid_ratio": .9}}
    payload = {"stamp_sec": 19, "stamp_nanosec": 800_000_000, "valid": True, "trusted_for_grasp": True, "candidate_count": 1, "candidates": [candidate]}
    return build_dry_run_plan(payload, now_stamp_ns=NOW, limits=LIMITS)


def test_fake_backend_runs_the_complete_pick_closed_loop_in_order():
    trace = run_pick_state_machine(_plan(), FakePickBackend(), calibration_verified=True)
    assert trace["state"] == "succeeded"
    assert [e["state"] for e in trace["events"]] == ["pre_grasp", "approach", "grasp", "close_gripper", "lift", "safe_retreat"]
    assert trace["hardware_commands_emitted"] is False


def test_calibration_cancel_timeout_tracking_and_gripper_failures_are_closed_and_recovered():
    assert run_pick_state_machine(_plan(), FakePickBackend(), calibration_verified=False)["reason"] == "coordinate_or_calibration_not_verified"
    cancelled = run_pick_state_machine(_plan(), FakePickBackend(), calibration_verified=True, cancel_at="approach")
    assert cancelled["state"] == "cancelled" and cancelled["recovery"]["code"] == "ok"
    timeout = run_pick_state_machine(_plan(), FakePickBackend({"lift": "timed_out"}), calibration_verified=True)
    assert timeout["reason"] == "timed_out" and timeout["recovery"]["code"] == "ok"
    tracking = run_pick_state_machine(_plan(), FakePickBackend({"approach": "tracking_error"}), calibration_verified=True)
    assert tracking["reason"] == "tracking_error" and tracking["recovery"]["code"] == "ok"
    gripper = run_pick_state_machine(_plan(), FakePickBackend({"close_gripper": "gripper_failed"}), calibration_verified=True)
    assert gripper["reason"] == "gripper_failed" and gripper["recovery"]["code"] == "ok"


def test_collision_workspace_and_serial_busy_stop_without_unsafe_recovery():
    for code in ("collision", "workspace_violation", "serial_busy"):
        trace = run_pick_state_machine(_plan(), FakePickBackend({"pre_grasp": code}), calibration_verified=True)
        assert trace["reason"] == code and "recovery" not in trace


def test_mercury_adapter_is_a_safety_preview_only_and_never_moves_hardware():
    backend = MercurySafetyPickBackend()
    trace = run_pick_state_machine(_plan(), backend, calibration_verified=True, timeouts=PickTimeouts())
    assert trace["state"] == "failed"
    assert trace["reason"] == "selected_arm_must_be_left_or_right"
    assert trace["hardware_commands_emitted"] is False
