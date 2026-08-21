from types import SimpleNamespace

import pytest

from deyes_stereo.right_arm_execution_contract import RightArmExecutionProfile, basis_to_xyz_euler_deg, build_action_steps, validate_cartesian_goal, validate_profile


PROFILE=RightArmExecutionProfile(validated=True,joint_min_deg=(-90.,)*6,joint_max_deg=(90.,)*6,workspace_min_base_m=(.1,-.5,.05),workspace_max_base_m=(.8,.5,.8),gripper_direction_validated=True,orientation_convention_validated=True)


def test_default_profile_and_unvalidated_tool_conventions_fail_closed():
    assert validate_profile(RightArmExecutionProfile())[1]=="right_arm_site_profile_not_validated"
    assert validate_profile(PROFILE)==(True,"ok")


def test_cartesian_goal_is_right_arm_identity_and_workspace_bound():
    goal=SimpleNamespace(arm_side="right",stage="pre_grasp",transaction_id="pick-42",calibration_id="cal-1",pose_base=[.4,0,.3,0,0,0],max_speed_m_s=.02,timeout_sec=8.)
    assert validate_cartesian_goal(goal,PROFILE,calibration_id="cal-1")[:2]==(True,"ok")
    goal.pose_base=[2.,0,.3,0,0,0]
    assert validate_cartesian_goal(goal,PROFILE,calibration_id="cal-1")[1]=="cartesian_target_outside_workspace"


def test_plan_expands_to_open_pick_lift_hold_return_release_retreat():
    pose=lambda position:{"frame_id":"base_link","position_m":position,"tool_basis_columns_base":[[1,0,0],[0,1,0],[0,0,1]]}
    plan={"state":"dry_run_ready","transaction_id":"pick-42","calibration_id":"cal-1","steps":[{"name":"pre_grasp","pose":pose([.4,0,.32])},{"name":"approach","pose":pose([.4,0,.28])},{"name":"grasp","pose":pose([.4,0,.2])},{"name":"close_gripper"},{"name":"lift","pose":pose([.4,0,.3])},{"name":"safe_retreat","pose":pose([.4,0,.32])}]}
    steps,reason=build_action_steps(plan)
    assert reason=="ok"
    assert [(s["kind"],s.get("stage",s.get("action"))) for s in steps]==[("gripper","open"),("cartesian","pre_grasp"),("cartesian","approach"),("cartesian","grasp"),("gripper","close"),("cartesian","lift"),("hold",None),("cartesian","return_to_grasp"),("gripper","open"),("cartesian","safe_retreat")]
    assert basis_to_xyz_euler_deg([[1,0,0],[0,1,0],[0,0,1]])==pytest.approx([0,0,0])
