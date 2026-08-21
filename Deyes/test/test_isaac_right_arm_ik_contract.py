import math

from deyes_stereo.isaac_right_arm_ik_contract import (
    RIGHT_ARM_JOINT_LIMITS_RAD,
    RIGHT_ARM_JOINT_NAMES,
    RIGHT_GRIPPER_ROOT_JOINT_NAMES,
    build_ik_request,
    build_sparse_right_arm_command,
    build_sparse_right_gripper_command,
    validate_injected_phase_targets,
    validate_sparse_right_command,
)


NAMES = [f"joint{i}_R" for i in range(1, 7)]
LIMITS = {name: [-3.14, 3.14] for name in NAMES}
PHASES = ("pregrasp", "approach", "close", "lift", "return", "release")


def request():
    return build_ik_request(target_base_m=[.4, 0., .2], approach_normal_base_unit=[0., 0., 1.], tcp_frame="right_tool", target_id="pen-1", stamp_ns=100, scene_sha256="a" * 64)


def targets():
    return {phase: ({"right_gripper": [.2]} if phase in {"close", "release"} else {"right_arm_rad": [0.] * 6}) for phase in PHASES}


def test_request_is_explicitly_ik_required_and_never_solver_ready():
    result = request()
    assert result["state"] == "ik_required" and result["solver"] is None and result["commands_emitted"] is False


def test_injected_targets_are_checked_for_all_phases_and_limits():
    ok, reason, plan = validate_injected_phase_targets(request(), targets(), joint_names=NAMES, joint_limits_rad=LIMITS)
    assert ok and reason == "ok" and plan["state"] == "ready"
    bad = targets(); bad["lift"] = {"right_arm_rad": [4.] * 6}
    assert validate_injected_phase_targets(request(), bad, joint_names=NAMES, joint_limits_rad=LIMITS)[1].startswith("ik_joint_out_of_limits")


def test_missing_model_or_phase_data_fails_closed():
    assert validate_injected_phase_targets(request(), {}, joint_names=NAMES, joint_limits_rad=LIMITS)[1].startswith("ik_phase_targets_missing")
    assert validate_injected_phase_targets(request(), targets(), joint_names=[], joint_limits_rad={})[1] == "right_arm_joint_names_invalid"


def test_verified_right_arm_order_and_usd_hard_limits_are_enforced():
    assert RIGHT_ARM_JOINT_NAMES == tuple(f"joint{i}_R" for i in range(1, 7))
    assert math.isclose(RIGHT_ARM_JOINT_LIMITS_RAD["joint3_R"][1], math.radians(5.0))
    ok, reason, payload = build_sparse_right_arm_command([0.0] * 6)
    assert (ok, reason) == (True, "ok")
    assert payload["name"] == list(RIGHT_ARM_JOINT_NAMES)
    assert build_sparse_right_arm_command([0.0, 0.0, .2, 0.0, 0.0, 0.0])[1] == "right_arm_joint_out_of_limits:joint3_R"


def test_sparse_command_rejects_unknown_duplicate_or_reordered_names():
    ok, _, payload = build_sparse_right_arm_command([0.0] * 6)
    assert ok and payload is not None
    assert validate_sparse_right_command(payload, kind="arm") == (True, "ok")
    altered = dict(payload); altered["name"] = list(RIGHT_ARM_JOINT_NAMES[:-1]) + ["joint7_R"]
    assert validate_sparse_right_command(altered, kind="arm")[1] == "sparse_command_names_not_exact"
    altered = dict(payload); altered["name"] = ["joint1_R"] * 6
    assert validate_sparse_right_command(altered, kind="arm")[1] == "sparse_command_names_not_exact"


def test_right_gripper_uses_four_verified_roots_and_signed_aperture_mapping():
    ok, reason, payload = build_sparse_right_gripper_command(.5)
    assert (ok, reason) == (True, "ok")
    assert payload["name"] == list(RIGHT_GRIPPER_ROOT_JOINT_NAMES)
    assert payload["position"] == [math.radians(20.0), -math.radians(20.0), -math.radians(20.0), math.radians(20.0)]
    assert validate_sparse_right_command(payload, kind="gripper") == (True, "ok")
    assert build_sparse_right_gripper_command(1.1)[1] == "right_gripper_aperture_invalid"
