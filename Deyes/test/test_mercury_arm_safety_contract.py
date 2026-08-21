from deyes_stereo.mercury_arm_safety_contract import (
    MercuryArmSafetyProfile, build_single_joint_jog_preview,
    validate_low_speed_motion_request,
)


PROFILE = MercuryArmSafetyProfile(
    arm_side="left", joint_min_deg=(-90., -80., -70., -60., -50., -40.),
    joint_max_deg=(90., 80., 70., 60., 50., 40.),
    workspace_min_base_m=(.10, -.40, .10), workspace_max_base_m=(.80, .40, .70),
)


def test_default_profile_is_dry_run_and_rejects_missing_measured_arm_selection():
    result = validate_low_speed_motion_request({"kind": "gripper", "action": "open"})
    assert result["commands_emitted"] is False
    assert result["execution_permitted"] is False
    assert result["reason"] == "selected_arm_must_be_left_or_right"


def test_joint_preview_requires_six_values_measured_limits_and_low_speed():
    good = validate_low_speed_motion_request({"kind": "joint_position", "positions_deg": [0.] * 6, "speed_deg_s": 5.0}, PROFILE)
    assert good["state"] == "dry_run_ready"
    assert good["motion_command_emitted"] is False
    assert validate_low_speed_motion_request({"kind": "joint_position", "positions_deg": [0.] * 6, "speed_deg_s": 5.01}, PROFILE)["reason"] == "joint_speed_exceeds_low_speed_limit"
    assert validate_low_speed_motion_request({"kind": "joint_position", "positions_deg": [91., 0., 0., 0., 0., 0.]}, PROFILE)["reason"] == "joint_target_outside_configured_limits"


def test_cartesian_preview_checks_base_frame_workspace_speed_and_acceleration():
    request = {"kind": "cartesian_pose", "frame_id": "base_link", "position_m": [.40, 0., .30], "speed_m_s": .02, "acceleration_m_s2": .05}
    assert validate_low_speed_motion_request(request, PROFILE)["state"] == "dry_run_ready"
    assert validate_low_speed_motion_request({**request, "frame_id": "camera"}, PROFILE)["reason"] == "cartesian_frame_must_be_base_link"
    assert validate_low_speed_motion_request({**request, "position_m": [.81, 0., .30]}, PROFILE)["reason"] == "cartesian_target_outside_configured_workspace"
    assert validate_low_speed_motion_request({**request, "speed_m_s": .021}, PROFILE)["reason"] == "cartesian_speed_exceeds_low_speed_limit"


def test_execution_request_and_gripper_effort_are_fail_closed():
    assert validate_low_speed_motion_request({"kind": "gripper", "action": "close", "request_execution": True}, PROFILE)["reason"] == "live_execution_not_implemented"
    assert validate_low_speed_motion_request({"kind": "gripper", "action": "close", "effort_percent": 20.1}, PROFILE)["reason"] == "gripper_effort_exceeds_low_speed_limit"


def test_first_joint_jog_preview_requires_feedback_and_is_limited_to_one_degree_at_2_deg_s():
    preview = build_single_joint_jog_preview(current_positions_deg=[0.] * 6, joint_index=2, delta_deg=.5, profile=PROFILE)
    assert preview["state"] == "dry_run_ready"
    assert preview["preview"]["positions_deg"] == [0., 0., .5, 0., 0., 0.]
    assert preview["commands_emitted"] is False
    assert build_single_joint_jog_preview(current_positions_deg=[0.] * 6, joint_index=2, delta_deg=1.01, profile=PROFILE)["reason"] == "single_joint_delta_must_be_within_0_to_1_deg"
    assert build_single_joint_jog_preview(current_positions_deg=[0.] * 6, joint_index=2, delta_deg=.5, speed_deg_s=2.1, profile=PROFILE)["reason"] == "single_joint_jog_speed_must_not_exceed_2_deg_s"
