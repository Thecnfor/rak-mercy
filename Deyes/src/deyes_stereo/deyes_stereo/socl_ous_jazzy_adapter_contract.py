"""Fail-closed, ROS-free contract for the Socl_ous X1 Isaac Sim bridge.

This module never imports ROS and never publishes.  It only creates a sparse
``sensor_msgs/JointState``-shaped payload after every execution interlock has
passed.  In particular, it does not implement IK/FK or infer joint targets from
the Cartesian points in a dual-pen co-grasp plan.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite
from typing import Any, Mapping


ROS_DOMAIN_ID = 45
JOINT_COMMAND_TOPIC = "/joint_command"
GRIPPER_COMMAND_TOPIC = "/gripper_command"
JOINT_STATES_TOPIC = "/joint_states"
JOINT_STATE_TYPE = "sensor_msgs/msg/JointState"
LEFT_ARM_JOINTS = tuple(f"joint{index}_L" for index in range(1, 7))
RIGHT_ARM_JOINTS = tuple(f"joint{index}_R" for index in range(1, 7))
GRIPPER_SIGNS = (1.0, -1.0, -1.0, 1.0)
ARM_PHASES = frozenset({"pregrasp", "approach", "contact", "lift"})
GRIPPER_PHASES = frozenset({"close"})
NO_PUBLISH_PHASES = frozenset({"confirm", "hold"})


@dataclass(frozen=True)
class SoclOusInterfaceEvidence:
    """Read-only ROS graph evidence captured immediately before execution."""

    graph_checked: bool = False
    checked_at_ns: int = 0
    ros_domain_id: int = 0
    joint_command_type: str = ""
    gripper_command_type: str = ""
    joint_states_type: str = ""
    joint_states_seen: bool = False


@dataclass(frozen=True)
class JointFeedbackSnapshot:
    """One directly observed ``/joint_states`` sample."""

    stamp_ns: int = 0
    names: tuple[str, ...] = ()
    positions_rad: tuple[float, ...] = ()


@dataclass(frozen=True)
class SoclOusExecutionProfile:
    """Site-owned limits and exact simulated gripper joint names.

    No gripper names or limits are guessed.  A profile is unusable until it is
    explicitly validated from the live X1 USD/ROS graph.
    """

    validated: bool = False
    profile_id: str = ""
    source: str = ""
    left_gripper_joint_names: tuple[str, ...] = ()
    right_gripper_joint_names: tuple[str, ...] = ()
    joint_limits_rad: Mapping[str, tuple[float, float]] = field(default_factory=dict)
    max_plan_age_ns: int = 250_000_000
    max_feedback_age_ns: int = 100_000_000
    max_interface_age_ns: int = 5_000_000_000


@dataclass(frozen=True)
class SoclOusPhaseTargets:
    """Prevalidated joint targets for exactly one co-grasp phase.

    Cartesian tool points are deliberately absent.  Arm phases require two
    independent six-joint vectors.  Close requires two independent gripper
    scalars; even equal values remain separately sourced and checked.
    """

    phase: str
    profile_id: str
    target_id: str
    validated: bool = False
    source: str = ""
    left_arm_rad: tuple[float, ...] | None = None
    right_arm_rad: tuple[float, ...] | None = None
    left_gripper_rad: float | None = None
    right_gripper_rad: float | None = None


@dataclass(frozen=True)
class SoclOusPhaseAuthorization:
    """Fresh authorization emitted by the barrier/sequencing layer."""

    validated: bool = False
    source: str = ""
    issued_at_ns: int = 0
    profile_id: str = ""
    target_id: str = ""
    phase: str = ""
    phase_index: int = -1
    prior_barrier_confirmed: bool = False
    both_grippers_confirmed: bool = False


def _result(reason: str, *, valid: bool = False, phase: str | None = None, publish_required: bool = False) -> dict[str, Any]:
    return {
        "schema": "socl_ous_jazzy_phase_command/v1",
        "valid": valid,
        "reason": reason,
        "failure_code": None if valid else reason,
        "phase": phase,
        "publish_required": publish_required,
        "publish_allowed": False,
        "commands_emitted": False,
        "commands": [],
    }


def _positive_age_failure(stamp_ns: Any, now_stamp_ns: int, maximum_age_ns: int, field: str) -> str | None:
    try:
        stamp = int(stamp_ns)
        now = int(now_stamp_ns)
        maximum = int(maximum_age_ns)
    except (TypeError, ValueError):
        return f"{field}_invalid"
    if stamp <= 0 or now <= 0 or maximum <= 0 or stamp > now or now - stamp > maximum:
        return f"{field}_stale_or_invalid"
    return None


def _finite_vector(value: Any, size: int, field: str) -> tuple[tuple[float, ...] | None, str | None]:
    if not isinstance(value, (list, tuple)) or len(value) != size:
        return None, f"{field}_must_have_{size}_values"
    try:
        converted = tuple(float(item) for item in value)
    except (TypeError, ValueError):
        return None, f"{field}_must_be_finite"
    if not all(isfinite(item) for item in converted):
        return None, f"{field}_must_be_finite"
    return converted, None


def _plan_failure(plan: Any, phase: str, now_stamp_ns: int, profile: SoclOusExecutionProfile) -> str | None:
    if not isinstance(plan, dict):
        return "plan_not_mapping"
    if plan.get("schema") != "dual_pen_cograsp_plan/v1":
        return "plan_schema_mismatch"
    if plan.get("state") != "ready" or plan.get("reason") != "ok":
        return "plan_not_ready"
    if plan.get("commands_emitted") is not False or plan.get("navigation_included") is not False:
        return "plan_contract_not_fail_closed"
    candidate = plan.get("candidate")
    if not isinstance(candidate, dict) or candidate.get("valid") is not True or candidate.get("reason") != "ok":
        return "plan_candidate_not_trusted"
    if candidate.get("commands_emitted") is not False or candidate.get("failure_code") is not None:
        return "plan_candidate_not_trusted"
    if str(plan.get("target_id") or "") != str(candidate.get("target_id") or ""):
        return "plan_target_id_mismatch"
    failure = _positive_age_failure(candidate.get("source_stamp_ns"), now_stamp_ns, profile.max_plan_age_ns, "plan_stamp")
    if failure:
        return failure
    steps = plan.get("steps")
    if not isinstance(steps, list):
        return "plan_steps_invalid"
    matches = [step for step in steps if isinstance(step, dict) and step.get("phase") == phase]
    if len(matches) != 1:
        return "plan_phase_missing_or_ambiguous"
    step = matches[0]
    if step.get("barrier") is not True or step.get("commands_emitted") is not False:
        return "plan_phase_not_fail_closed_barrier"
    return None


def _profile_failure(profile: SoclOusExecutionProfile) -> str | None:
    if not profile.validated:
        return "execution_profile_not_validated"
    if not profile.profile_id:
        return "execution_profile_id_missing"
    if profile.source != "validated_sim_joint_targets":
        return "execution_profile_source_untrusted"
    left, right = profile.left_gripper_joint_names, profile.right_gripper_joint_names
    if len(left) != 4 or len(right) != 4:
        return "gripper_joint_names_must_be_four_per_side"
    all_names = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + tuple(left) + tuple(right)
    if any(not isinstance(name, str) or not name for name in all_names) or len(set(all_names)) != len(all_names):
        return "joint_names_empty_or_not_unique"
    for name in all_names:
        limit = profile.joint_limits_rad.get(name)
        if not isinstance(limit, (list, tuple)) or len(limit) != 2:
            return f"joint_limit_missing:{name}"
        try:
            lower, upper = float(limit[0]), float(limit[1])
        except (TypeError, ValueError):
            return f"joint_limit_invalid:{name}"
        if not isfinite(lower) or not isfinite(upper) or lower >= upper:
            return f"joint_limit_invalid:{name}"
    for name in ("max_plan_age_ns", "max_feedback_age_ns", "max_interface_age_ns"):
        try:
            if int(getattr(profile, name)) <= 0:
                return f"execution_profile_invalid:{name}"
        except (TypeError, ValueError):
            return f"execution_profile_invalid:{name}"
    return None


def _interface_failure(evidence: SoclOusInterfaceEvidence, now_stamp_ns: int, profile: SoclOusExecutionProfile) -> str | None:
    if not evidence.graph_checked:
        return "live_ros_graph_not_checked"
    if evidence.ros_domain_id != ROS_DOMAIN_ID:
        return "ros_domain_id_must_be_45"
    expected = (evidence.joint_command_type, evidence.gripper_command_type, evidence.joint_states_type)
    if expected != (JOINT_STATE_TYPE, JOINT_STATE_TYPE, JOINT_STATE_TYPE):
        return "live_interface_type_mismatch"
    if not evidence.joint_states_seen:
        return "joint_states_not_observed"
    return _positive_age_failure(evidence.checked_at_ns, now_stamp_ns, profile.max_interface_age_ns, "interface_check")


def _feedback_failure(feedback: JointFeedbackSnapshot, now_stamp_ns: int, profile: SoclOusExecutionProfile) -> str | None:
    failure = _positive_age_failure(feedback.stamp_ns, now_stamp_ns, profile.max_feedback_age_ns, "joint_feedback_stamp")
    if failure:
        return failure
    if len(feedback.names) != len(feedback.positions_rad):
        return "joint_feedback_name_position_length_mismatch"
    if len(set(feedback.names)) != len(feedback.names):
        return "joint_feedback_duplicate_names"
    try:
        positions = tuple(float(value) for value in feedback.positions_rad)
    except (TypeError, ValueError):
        return "joint_feedback_nonfinite"
    if not all(isfinite(value) for value in positions):
        return "joint_feedback_nonfinite"
    required = set(LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + profile.left_gripper_joint_names + profile.right_gripper_joint_names)
    missing = sorted(required.difference(feedback.names))
    if missing:
        return f"joint_feedback_missing:{','.join(missing)}"
    return None


def _authorization_failure(
    authorization: SoclOusPhaseAuthorization,
    plan: dict[str, Any],
    targets: SoclOusPhaseTargets,
    now_stamp_ns: int,
    profile: SoclOusExecutionProfile,
) -> str | None:
    if not authorization.validated or authorization.source != "dual_pen_cograsp_executor_barrier":
        return "phase_not_authorized_by_barrier"
    if authorization.profile_id != profile.profile_id or authorization.target_id != targets.target_id:
        return "phase_authorization_binding_mismatch"
    if authorization.phase != targets.phase:
        return "phase_authorization_phase_mismatch"
    steps = plan.get("steps", [])
    matching_indexes = [index for index, step in enumerate(steps) if isinstance(step, dict) and step.get("phase") == targets.phase]
    if len(matching_indexes) != 1 or authorization.phase_index != matching_indexes[0]:
        return "phase_authorization_index_mismatch"
    failure = _positive_age_failure(authorization.issued_at_ns, now_stamp_ns, profile.max_feedback_age_ns, "phase_authorization_stamp")
    if failure:
        return failure
    if not authorization.prior_barrier_confirmed:
        return "prior_barrier_not_confirmed"
    if targets.phase in {"lift", "hold"} and not authorization.both_grippers_confirmed:
        return "both_grippers_not_confirmed"
    return None


def _target_values(targets: SoclOusPhaseTargets) -> tuple[tuple[str, ...], tuple[float, ...], str, dict[str, float]] | tuple[None, None, str, dict[str, float]]:
    phase = targets.phase
    if phase in ARM_PHASES:
        left, failure = _finite_vector(targets.left_arm_rad, 6, "left_arm_rad")
        if failure:
            return None, None, failure, {}
        right, failure = _finite_vector(targets.right_arm_rad, 6, "right_arm_rad")
        if failure:
            return None, None, failure, {}
        if targets.left_gripper_rad is not None or targets.right_gripper_rad is not None:
            return None, None, "arm_phase_must_not_include_gripper_targets", {}
        assert left is not None and right is not None
        return LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS, left + right, "ok", {}
    if phase in GRIPPER_PHASES:
        if targets.left_arm_rad is not None or targets.right_arm_rad is not None:
            return None, None, "gripper_phase_must_not_include_arm_targets", {}
        try:
            left_scalar, right_scalar = float(targets.left_gripper_rad), float(targets.right_gripper_rad)
        except (TypeError, ValueError):
            return None, None, "independent_gripper_targets_required", {}
        if not isfinite(left_scalar) or not isfinite(right_scalar):
            return None, None, "independent_gripper_targets_required", {}
        # The two scalars remain independent evidence even when numerically equal.
        return (), tuple(left_scalar * sign for sign in GRIPPER_SIGNS) + tuple(right_scalar * sign for sign in GRIPPER_SIGNS), "ok", {"left": left_scalar, "right": right_scalar}
    if phase in NO_PUBLISH_PHASES:
        if any(value is not None for value in (targets.left_arm_rad, targets.right_arm_rad, targets.left_gripper_rad, targets.right_gripper_rad)):
            return None, None, "no_publish_phase_must_not_have_targets", {}
        return (), (), "ok", {}
    return None, None, "unsupported_phase", {}


def audit_socl_ous_phase(
    plan: dict[str, Any],
    targets: SoclOusPhaseTargets,
    *,
    now_stamp_ns: int,
    profile: SoclOusExecutionProfile = SoclOusExecutionProfile(),
    interface: SoclOusInterfaceEvidence = SoclOusInterfaceEvidence(),
    feedback: JointFeedbackSnapshot = JointFeedbackSnapshot(),
    authorization: SoclOusPhaseAuthorization = SoclOusPhaseAuthorization(),
) -> dict[str, Any]:
    """Audit all live execution gates without constructing a command payload."""

    phase = targets.phase
    publish_required = phase in ARM_PHASES or phase in GRIPPER_PHASES
    for failure in (
        _profile_failure(profile),
        _plan_failure(plan, phase, now_stamp_ns, profile),
        _interface_failure(interface, now_stamp_ns, profile),
        _feedback_failure(feedback, now_stamp_ns, profile),
    ):
        if failure:
            return _result(failure, phase=phase, publish_required=publish_required)
    if not targets.validated or targets.source != "validated_sim_joint_targets":
        return _result("phase_targets_not_validated", phase=phase, publish_required=publish_required)
    if targets.profile_id != profile.profile_id:
        return _result("phase_target_profile_mismatch", phase=phase, publish_required=publish_required)
    if targets.target_id != str(plan.get("target_id") or ""):
        return _result("phase_target_id_mismatch", phase=phase, publish_required=publish_required)
    authorization_failure = _authorization_failure(authorization, plan, targets, now_stamp_ns, profile)
    if authorization_failure:
        return _result(authorization_failure, phase=phase, publish_required=publish_required)
    names, positions, failure, _ = _target_values(targets)
    if failure != "ok":
        return _result(failure, phase=phase, publish_required=publish_required)
    if names is None or positions is None:
        return _result("phase_target_invalid", phase=phase, publish_required=publish_required)
    command_names = names
    if phase in GRIPPER_PHASES:
        command_names = profile.left_gripper_joint_names + profile.right_gripper_joint_names
    for name, position in zip(command_names, positions):
        lower, upper = profile.joint_limits_rad[name]
        if not float(lower) <= position <= float(upper):
            return _result(f"joint_target_out_of_bounds:{name}", phase=phase, publish_required=publish_required)
    return _result("ok", valid=True, phase=phase, publish_required=publish_required)


def prepare_socl_ous_phase_command(
    plan: dict[str, Any],
    targets: SoclOusPhaseTargets,
    *,
    now_stamp_ns: int,
    enable_execution: bool = False,
    profile: SoclOusExecutionProfile = SoclOusExecutionProfile(),
    interface: SoclOusInterfaceEvidence = SoclOusInterfaceEvidence(),
    feedback: JointFeedbackSnapshot = JointFeedbackSnapshot(),
    authorization: SoclOusPhaseAuthorization = SoclOusPhaseAuthorization(),
) -> dict[str, Any]:
    """Create one sparse publish payload only after every interlock passes.

    ``commands_emitted`` remains false: only the eventual ROS publisher may set
    it true after a successful ``publish()`` call.
    """

    audited = audit_socl_ous_phase(
        plan, targets, now_stamp_ns=now_stamp_ns, profile=profile,
        interface=interface, feedback=feedback, authorization=authorization,
    )
    if not audited["valid"]:
        return audited
    if not enable_execution:
        return _result("execution_disabled", phase=targets.phase, publish_required=audited["publish_required"])
    if not audited["publish_required"]:
        return _result("phase_requires_feedback_only", valid=True, phase=targets.phase, publish_required=False)
    names, positions, failure, independent = _target_values(targets)
    assert failure == "ok" and names is not None and positions is not None
    topic = JOINT_COMMAND_TOPIC
    if targets.phase in GRIPPER_PHASES:
        names = profile.left_gripper_joint_names + profile.right_gripper_joint_names
        topic = GRIPPER_COMMAND_TOPIC
    result = _result("ok", valid=True, phase=targets.phase, publish_required=True)
    command = {
        "topic": topic,
        "type": JOINT_STATE_TYPE,
        "header_stamp_ns": int(now_stamp_ns),
        "name": list(names),
        "position": list(positions),
        "velocity": [],
        "effort": [],
        "sparse_name_addressing": True,
    }
    result.update({
        "publish_allowed": True,
        "command": command,
        "commands": [command],
    })
    if independent:
        result["independent_gripper_targets_rad"] = independent
    return result
