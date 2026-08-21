from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_isaac_motion_launch_is_simulation_only_and_has_no_serial_path():
    launch = (ROOT / "src/deyes_bringup/launch/isaac_single_pen_motion.launch.py").read_text(encoding="utf-8")
    config = (ROOT / "config/stereo/isaac_right_arm_stage_executor.defaults.yaml").read_text(encoding="utf-8")
    assert "isaac_right_arm_stage_executor" in launch
    assert "enable_execution" in launch and "simulation_only" in launch and "motion_enabled" in launch
    assert "enable_execution: false" in config and "simulation_only: false" in config
    assert "ros_domain_id: 46" in config
    assert "/x1_sim/joint_command" in config and "/x1_sim/gripper_command" in config
    assert "joint1_R" in config and "right_gripper_left_finger_joint" in config
    assert ': "/joint_command"' not in config and ': "/gripper_command"' not in config
    assert "/dev/right_arm" not in launch + config
