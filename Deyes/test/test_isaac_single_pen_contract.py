from deyes_stereo.isaac_single_pen_contract import build_single_pen_motion_plan, evaluate_pen_lift, stage_feedback_converged, validate_single_pen_candidate


SCENE = "a" * 64


def candidate(**changes):
    value = {"source": "isaac_sim", "simulation_validated": True, "physical_validated": False, "physical_execution_eligible": False, "candidate_count": 1, "stamp_ns": 100, "scene_sha256": SCENE, "candidates": [{"target_id": "pen-1", "target_frame": "base_link", "grasp_point_base_m": [.4, 0., .2]}]}
    value.update(changes); return value


def ik():
    return {phase: ({"right_gripper": [.1]} if phase in {"close", "release"} else {"right_arm_rad": [0.] * 6}) for phase in ("pregrasp", "approach", "close", "lift", "return", "release")}


def test_single_candidate_requires_scene_and_fresh_observation():
    assert validate_single_pen_candidate(candidate(), expected_scene_sha256=SCENE, now_stamp_ns=200) == (True, "ok")
    assert validate_single_pen_candidate(candidate(candidate_count=2), expected_scene_sha256=SCENE, now_stamp_ns=200)[1] == "exactly_one_candidate_required"
    assert validate_single_pen_candidate(candidate(scene_sha256="b" * 64), expected_scene_sha256=SCENE, now_stamp_ns=200)[1] == "scene_sha256_mismatch"


def test_plan_rejects_missing_ik_and_binds_identity():
    assert build_single_pen_motion_plan(candidate(), scene_sha256=SCENE, now_stamp_ns=200, ik_targets={})["reason"].startswith("ik_targets_missing")
    plan = build_single_pen_motion_plan(candidate(), scene_sha256=SCENE, now_stamp_ns=200, ik_targets=ik())
    assert plan["state"] == "ready" and plan["target_id"] == "pen-1" and plan["commands_emitted"] is False


def test_lift_requires_observed_rigid_body_motion():
    assert evaluate_pen_lift(before_z_m=.2, after_z_m=.24, threshold_m=.03) == (True, "ok")
    assert evaluate_pen_lift(before_z_m=.2, after_z_m=.21, threshold_m=.03)[0] is False
    assert evaluate_pen_lift(before_z_m=None, after_z_m=.24)[1] == "pen_pose_observation_missing"


def test_stage_barrier_requires_fresh_converged_feedback():
    assert stage_feedback_converged(phase="approach", target_arm_rad=[0.] * 6, observed_arm_rad=[.01] * 6, feedback_age_sec=.1)[0]
    assert stage_feedback_converged(phase="approach", target_arm_rad=[0.] * 6, observed_arm_rad=[.1] * 6, feedback_age_sec=.1)[1] == "arm_not_converged"
    assert stage_feedback_converged(phase="close", target_gripper=.2, observed_gripper=.2, feedback_age_sec=2.)[1] == "feedback_stale_or_timeout"
