from dataclasses import replace
from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "src" / "deyes_stereo"
sys.path.insert(0, str(PACKAGE_ROOT))

from deyes_stereo.socl_ous_jazzy_adapter_contract import (  # noqa: E402
    GRIPPER_SIGNS,
    JOINT_STATE_TYPE,
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
    JointFeedbackSnapshot,
    SoclOusExecutionProfile,
    SoclOusInterfaceEvidence,
    SoclOusPhaseAuthorization,
    SoclOusPhaseTargets,
    audit_socl_ous_phase,
    prepare_socl_ous_phase_command,
)


NOW = 10_000_000_000
LEFT_GRIPPER = tuple(f"left_gripper_{index}" for index in range(1, 5))
RIGHT_GRIPPER = tuple(f"right_gripper_{index}" for index in range(1, 5))
ALL_JOINTS = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + LEFT_GRIPPER + RIGHT_GRIPPER


def profile():
    return SoclOusExecutionProfile(
        validated=True,
        profile_id="socl-x1-validated-001",
        source="validated_sim_joint_targets",
        left_gripper_joint_names=LEFT_GRIPPER,
        right_gripper_joint_names=RIGHT_GRIPPER,
        joint_limits_rad={name: (-3.2, 3.2) for name in ALL_JOINTS},
    )


def interface():
    return SoclOusInterfaceEvidence(
        graph_checked=True,
        checked_at_ns=NOW - 10_000_000,
        ros_domain_id=45,
        joint_command_type=JOINT_STATE_TYPE,
        gripper_command_type=JOINT_STATE_TYPE,
        joint_states_type=JOINT_STATE_TYPE,
        joint_states_seen=True,
    )


def feedback():
    return JointFeedbackSnapshot(
        stamp_ns=NOW - 5_000_000,
        names=ALL_JOINTS,
        positions_rad=(0.0,) * len(ALL_JOINTS),
    )


def plan():
    phases = ("pregrasp", "approach", "contact", "close", "confirm", "lift", "hold")
    return {
        "schema": "dual_pen_cograsp_plan/v1",
        "state": "ready",
        "reason": "ok",
        "target_id": "pen-1",
        "navigation_included": False,
        "commands_emitted": False,
        "candidate": {
            "valid": True,
            "reason": "ok",
            "failure_code": None,
            "target_id": "pen-1",
            "source_stamp_ns": NOW - 20_000_000,
            "commands_emitted": False,
        },
        "steps": [{"phase": phase, "barrier": True, "commands_emitted": False} for phase in phases],
    }


def arm_targets(phase="pregrasp"):
    return SoclOusPhaseTargets(
        phase=phase,
        profile_id="socl-x1-validated-001",
        target_id="pen-1",
        validated=True,
        source="validated_sim_joint_targets",
        left_arm_rad=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
        right_arm_rad=(-0.1, -0.2, -0.3, -0.4, -0.5, -0.6),
    )


def close_targets(left=0.12, right=0.08):
    return SoclOusPhaseTargets(
        phase="close",
        profile_id="socl-x1-validated-001",
        target_id="pen-1",
        validated=True,
        source="validated_sim_joint_targets",
        left_gripper_rad=left,
        right_gripper_rad=right,
    )


def authorization(phase="pregrasp", *, both_grippers_confirmed=False):
    phase_order = ("pregrasp", "approach", "contact", "close", "confirm", "lift", "hold")
    return SoclOusPhaseAuthorization(
        validated=True,
        source="dual_pen_cograsp_executor_barrier",
        issued_at_ns=NOW - 2_000_000,
        profile_id=profile().profile_id,
        target_id="pen-1",
        phase=phase,
        phase_index=phase_order.index(phase),
        prior_barrier_confirmed=True,
        both_grippers_confirmed=both_grippers_confirmed,
    )


def prepare(targets, **changes):
    arguments = dict(
        plan=plan(), targets=targets, now_stamp_ns=NOW, enable_execution=True,
        profile=profile(), interface=interface(), feedback=feedback(), authorization=authorization(targets.phase, both_grippers_confirmed=targets.phase in {"lift", "hold"}),
    )
    arguments.update(changes)
    return prepare_socl_ous_phase_command(**arguments)


def test_default_is_fail_closed_and_constructs_no_command():
    result = prepare_socl_ous_phase_command(plan(), arm_targets(), now_stamp_ns=NOW)
    assert result["valid"] is False
    assert result["publish_allowed"] is False
    assert result["commands_emitted"] is False
    assert result["commands"] == []
    assert "command" not in result


def test_execution_disabled_constructs_no_payload_even_after_successful_audit():
    result = prepare(arm_targets(), enable_execution=False)
    assert result["reason"] == "execution_disabled"
    assert result["commands"] == []
    assert "command" not in result


def test_arm_command_uses_exact_sparse_names_and_no_unnamed_joint_slots():
    result = prepare(arm_targets())
    assert result["valid"] is True and result["publish_allowed"] is True
    assert result["command"]["topic"] == "/joint_command"
    assert result["command"]["name"] == list(LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS)
    assert len(result["command"]["position"]) == 12
    assert result["command"]["sparse_name_addressing"] is True
    assert result["commands_emitted"] is False


def test_gripper_keeps_independent_side_targets_and_required_sign_pattern():
    result = prepare(close_targets(0.12, 0.08))
    assert result["command"]["topic"] == "/gripper_command"
    assert result["command"]["name"] == list(LEFT_GRIPPER + RIGHT_GRIPPER)
    expected = [0.12 * sign for sign in GRIPPER_SIGNS] + [0.08 * sign for sign in GRIPPER_SIGNS]
    assert result["command"]["position"] == expected
    assert result["independent_gripper_targets_rad"] == {"left": 0.12, "right": 0.08}


def test_graph_domain_type_and_observation_are_mandatory():
    bad = replace(interface(), ros_domain_id=0)
    assert prepare(arm_targets(), interface=bad)["reason"] == "ros_domain_id_must_be_45"
    bad = replace(interface(), joint_command_type="trajectory_msgs/msg/JointTrajectory")
    assert prepare(arm_targets(), interface=bad)["reason"] == "live_interface_type_mismatch"
    bad = replace(interface(), joint_states_seen=False)
    assert prepare(arm_targets(), interface=bad)["reason"] == "joint_states_not_observed"


def test_stale_or_incomplete_feedback_fails_closed():
    stale = replace(feedback(), stamp_ns=NOW - profile().max_feedback_age_ns - 1)
    assert prepare(arm_targets(), feedback=stale)["reason"] == "joint_feedback_stamp_stale_or_invalid"
    incomplete = replace(feedback(), names=feedback().names[:-1], positions_rad=feedback().positions_rad[:-1])
    assert prepare(arm_targets(), feedback=incomplete)["reason"].startswith("joint_feedback_missing:")
    duplicate = replace(feedback(), names=(feedback().names[0],) + feedback().names, positions_rad=(0.0,) + feedback().positions_rad)
    assert prepare(arm_targets(), feedback=duplicate)["reason"] == "joint_feedback_duplicate_names"


def test_plan_must_be_current_ready_and_untampered():
    stale_plan = plan()
    stale_plan["candidate"]["source_stamp_ns"] = NOW - profile().max_plan_age_ns - 1
    assert prepare(arm_targets(), plan=stale_plan)["reason"] == "plan_stamp_stale_or_invalid"
    emitted = plan()
    emitted["commands_emitted"] = True
    assert prepare(arm_targets(), plan=emitted)["reason"] == "plan_contract_not_fail_closed"
    untrusted = plan()
    untrusted["candidate"]["valid"] = False
    assert prepare(arm_targets(), plan=untrusted)["reason"] == "plan_candidate_not_trusted"


def test_profile_and_targets_require_explicit_validation_and_binding():
    assert prepare(arm_targets(), profile=replace(profile(), validated=False))["reason"] == "execution_profile_not_validated"
    assert prepare(replace(arm_targets(), validated=False))["reason"] == "phase_targets_not_validated"
    assert prepare(replace(arm_targets(), profile_id="other"))["reason"] == "phase_target_profile_mismatch"
    assert prepare(replace(arm_targets(), target_id="other"))["reason"] == "phase_target_id_mismatch"


def test_fresh_barrier_authorization_and_gripper_confirmation_are_mandatory():
    assert prepare(arm_targets(), authorization=SoclOusPhaseAuthorization())["reason"] == "phase_not_authorized_by_barrier"
    wrong_phase = authorization("approach")
    assert prepare(arm_targets(), authorization=wrong_phase)["reason"] == "phase_authorization_phase_mismatch"
    lift = arm_targets("lift")
    assert prepare(lift, authorization=authorization("lift", both_grippers_confirmed=False))["reason"] == "both_grippers_not_confirmed"


def test_exact_six_joint_targets_and_limits_are_required_without_ik_inference():
    short = replace(arm_targets(), left_arm_rad=(0.1,) * 5)
    assert prepare(short)["reason"] == "left_arm_rad_must_have_6_values"
    limited = profile()
    limits = dict(limited.joint_limits_rad)
    limits["joint1_L"] = (-0.05, 0.05)
    assert prepare(arm_targets(), profile=replace(limited, joint_limits_rad=limits))["reason"] == "joint_target_out_of_bounds:joint1_L"


def test_confirm_and_hold_are_feedback_only_and_never_construct_publish_payload():
    target = SoclOusPhaseTargets(
        phase="confirm", profile_id=profile().profile_id, target_id="pen-1",
        validated=True, source="validated_sim_joint_targets",
    )
    result = prepare(target)
    assert result["valid"] is True
    assert result["reason"] == "phase_requires_feedback_only"
    assert result["publish_allowed"] is False
    assert result["commands"] == []


def test_audit_never_constructs_a_command_payload():
    result = audit_socl_ous_phase(
        plan(), arm_targets(), now_stamp_ns=NOW, profile=profile(),
        interface=interface(), feedback=feedback(), authorization=authorization(),
    )
    assert result["valid"] is True
    assert result["publish_required"] is True
    assert result["commands"] == []
    assert "command" not in result


def test_module_is_ros_free():
    source = (PACKAGE_ROOT / "deyes_stereo" / "socl_ous_jazzy_adapter_contract.py").read_text(encoding="utf-8")
    assert "import rclpy" not in source
    assert "from sensor_msgs" not in source
    assert "trajectory_msgs" not in source
