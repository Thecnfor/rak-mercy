from deyes_stereo.pick_ros2_execution_contract import LiveExecutionGates, validate_execution_admission, validate_single_step


def _coordinate(): return {"trusted_for_execution": True, "frame_id": "base_link", "calibration_id": "measured", "candidate_id": "pen-1", "stamp_ns": 9}
def _plan(): return {"state": "dry_run_ready", "commands_emitted": False, "target_id": "pen-1", "candidate_stamp_ns": 9, "steps": [{"name": "pre_grasp", "kind": "cartesian_intent"}, {"name": "close_gripper", "kind": "gripper_intent"}]}
def _gates(**changes):
    result = LiveExecutionGates(dry_run=False, enable_live_execution=True, operator_confirmed=True, validated_calibration=True, action_server_available=True, joint_state_stamp_ns=10_000_000_000, now_stamp_ns=10_100_000_000)
    return LiveExecutionGates(**{**result.__dict__, **changes})


def test_admission_requires_dry_run_off_all_three_gates_trusted_tf_server_and_fresh_feedback():
    assert validate_execution_admission(_coordinate(), _plan(), _gates()) == (True, "ok")
    for change, expected in (({"dry_run": True}, "dry_run_enabled"), ({"enable_live_execution": False}, "enable_live_execution_false"), ({"operator_confirmed": False}, "operator_confirmation_missing"), ({"validated_calibration": False}, "validated_calibration_missing"), ({"action_server_available": False}, "follow_joint_trajectory_server_unavailable"), ({"joint_state_stamp_ns": 1}, "joint_state_stale_or_missing")):
        assert validate_execution_admission(_coordinate(), _plan(), _gates(**change))[1] == expected
    assert validate_execution_admission({"trusted_for_execution": False}, _plan(), _gates())[1] == "coordinate_result_not_trusted_for_execution"
    assert validate_execution_admission({**_coordinate(), "candidate_id": "other"}, _plan(), _gates())[1] == "coordinate_plan_target_mismatch"
    assert validate_execution_admission({**_coordinate(), "stamp_ns": 10}, _plan(), _gates())[1] == "coordinate_plan_stamp_mismatch"


def test_every_motion_stage_requires_individual_confirmation_and_approved_joint_trajectory():
    assert validate_single_step(_plan(), "pre_grasp", operator_step_confirmed=False)[1] == "single_step_confirmation_missing"
    assert validate_single_step(_plan(), "pre_grasp", operator_step_confirmed=True)[1] == "approved_joint_trajectory_missing"
    plan = _plan(); plan["steps"][0]["approved_joint_trajectory"] = {"joint_names": ["left_j1"], "positions": [.1]}
    assert validate_single_step(plan, "pre_grasp", operator_step_confirmed=True) == (True, "ok")
    assert validate_single_step(plan, "close_gripper", operator_step_confirmed=True) == (True, "ok")
