from pathlib import Path


def test_single_shot_launch_is_right_arm_one_shot_and_dry_run_by_default():
    root=Path(__file__).resolve().parents[1]
    launch=(root/"src/deyes_bringup/launch/single_shot_pick.launch.py").read_text(encoding="utf-8")
    config=(root/"config/stereo/single_shot_pick.defaults.yaml").read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("arm_side",default_value="right")' in launch
    assert 'DeclareLaunchArgument("dry_run",default_value="true")' in launch
    assert 'DeclareLaunchArgument("enable_live_execution",default_value="false")' in launch
    assert 'run_mode: "one_shot"' in config and 'expected_max_targets: 1' in config
    assert 'image_topic: "/x1/snapshot/left_rect"' in config


def test_action_interfaces_expose_feedback_cancelable_stage_boundaries():
    root=Path(__file__).resolve().parents[1]/"src/deyes_interfaces/action"
    cartesian=(root/"ExecuteCartesianStage.action").read_text(encoding="utf-8")
    gripper=(root/"ExecuteGripper.action").read_text(encoding="utf-8")
    assert "transaction_id" in cartesian and "tracking_error_m" in cartesian and "failure_code" in cartesian
    assert "transaction_id" in gripper and "current_value" in gripper and "failure_code" in gripper


def test_executor_status_exposes_navigation_and_execution_trust_identity():
    root=Path(__file__).resolve().parents[1]
    source=(root/"src/deyes_stereo/deyes_stereo/single_shot_pick_executor_node.py").read_text(encoding="utf-8")
    for field in ('"mission_id"', '"nav_epoch"', '"calibration_id"', '"dry_run"', '"trusted_for_execution"'):
        assert field in source
