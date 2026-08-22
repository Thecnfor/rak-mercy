from deyes_stereo.motion_adapter_contract import (
    NAV2_ACTION_TYPE,
    inspect_ros_interface_presence,
    required_motion_adapter_contract,
    validate_dual_arm_adapter_contract,
)


def test_nav2_probe_accepts_only_the_standard_navigate_to_pose_action_type():
    present = inspect_ros_interface_presence(
        action_names_and_types=[("navigate_to_pose", [NAV2_ACTION_TYPE])],
        topic_names_and_types=[],
    )
    assert present["nav2"]["state"] == "interface_present"
    assert present["execution_permitted"] is False
    wrong = inspect_ros_interface_presence(
        action_names_and_types=[("/navigate_to_pose", ["example/action/Other"])],
        topic_names_and_types=[],
    )
    assert wrong["nav2"]["state"] == "type_mismatch"


def test_joint_states_evidence_is_reported_but_never_accepted_as_an_arm_adapter():
    result = inspect_ros_interface_presence(
        action_names_and_types=[], topic_names_and_types=[("joint_states", ["sensor_msgs/msg/JointState"])],
    )
    arm = result["mercury_turing_dual_arm"]
    assert arm["state"] == "unimplemented_fail_closed"
    assert arm["joint_states_observed_types"] == ["sensor_msgs/msg/JointState"]


def test_dual_arm_contract_requires_feedback_cancel_timeout_collision_and_gripper_evidence():
    incomplete = validate_dual_arm_adapter_contract({"goal_acceptance": True})
    assert incomplete["contract_valid"] is False
    assert "cancellation" in incomplete["missing_capabilities"]
    complete = validate_dual_arm_adapter_contract({name: True for name in required_motion_adapter_contract()["mercury_turing_dual_arm"]["required_capabilities"]})
    assert complete["contract_valid"] is True
    assert complete["execution_permitted"] is False
