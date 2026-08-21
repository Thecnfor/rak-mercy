"""ROS-free boundary for an Isaac right-arm IK provider.

No solver is bundled here: the Socl_ous checkout currently exposes no Lula
module and only an ``x1_left_arm.urdf``.  This contract accepts externally
computed, reviewed six-axis targets and rejects incomplete/model-free claims.
"""
from __future__ import annotations

import math
from typing import Any, Mapping

from .isaac_single_pen_contract import PHASES

IK_SCHEMA = "isaac_right_arm_ik_request/v1"
MODEL_MISSING_REASON = "right_arm_ik_model_missing"

# Evidence source (read-only, 2026-08-21):
# ``scenes/active/robots/mercury_x1.usd`` plus live
# ``/x1_sim/joint_states`` on Socl_ous, ROS domain 46.  These are the Isaac
# articulation names, not a physical-robot or placeholder-URDF convention.
RIGHT_ARM_JOINT_NAMES = tuple(f"joint{index}_R" for index in range(1, 7))
RIGHT_ARM_JOINT_LIMITS_RAD = {
    "joint1_R": (math.radians(-165.0), math.radians(165.0)),
    "joint2_R": (math.radians(-55.0), math.radians(95.0)),
    "joint3_R": (math.radians(-173.0), math.radians(5.0)),
    "joint4_R": (math.radians(-165.0), math.radians(165.0)),
    "joint5_R": (math.radians(-20.0), math.radians(265.0)),
    "joint6_R": (math.radians(-180.0), math.radians(180.0)),
}
RIGHT_GRIPPER_ROOT_JOINT_NAMES = (
    "right_gripper_left_finger_joint",
    "right_gripper_right_finger_joint",
    "right_gripper_left_joint2",
    "right_gripper_right_join2",
)
RIGHT_GRIPPER_ROOT_SIGNS = (1.0, -1.0, -1.0, 1.0)
RIGHT_GRIPPER_SAFE_TRAVEL_RAD = math.radians(40.0)
RIGHT_GRIPPER_TCP_FRAME = "right_gripper_base"


def _exact_names(value: Any, expected: tuple[str, ...]) -> bool:
    """Require the exact sparse articulation sequence; reject aliases/dupes."""
    try:
        names = tuple(str(item) for item in value)
    except TypeError:
        return False
    return names == expected and len(set(names)) == len(names)


def build_sparse_right_arm_command(values_rad: Any) -> tuple[bool, str, dict[str, list[Any]] | None]:
    """Return a ROS ``JointState``-shaped *sparse* right-arm payload only."""
    values = _vec(values_rad, 6)
    if values is None:
        return False, "right_arm_values_invalid", None
    for name, value in zip(RIGHT_ARM_JOINT_NAMES, values):
        lower, upper = RIGHT_ARM_JOINT_LIMITS_RAD[name]
        if not lower <= value <= upper:
            return False, f"right_arm_joint_out_of_limits:{name}", None
    return True, "ok", {"name": list(RIGHT_ARM_JOINT_NAMES), "position": list(values)}


def build_sparse_right_gripper_command(aperture: Any) -> tuple[bool, str, dict[str, list[Any]] | None]:
    """Map aperture ``[0,1]`` to the four verified right-gripper roots."""
    try:
        opening = float(aperture)
    except (TypeError, ValueError):
        return False, "right_gripper_aperture_invalid", None
    if not math.isfinite(opening) or not 0.0 <= opening <= 1.0:
        return False, "right_gripper_aperture_invalid", None
    values = [sign * opening * RIGHT_GRIPPER_SAFE_TRAVEL_RAD for sign in RIGHT_GRIPPER_ROOT_SIGNS]
    return True, "ok", {"name": list(RIGHT_GRIPPER_ROOT_JOINT_NAMES), "position": values}


def validate_sparse_right_command(payload: Mapping[str, Any], *, kind: str) -> tuple[bool, str]:
    """Reject unknown, duplicate, reordered, or full-articulation commands."""
    if not isinstance(payload, Mapping):
        return False, "sparse_command_not_mapping"
    expected = RIGHT_ARM_JOINT_NAMES if kind == "arm" else RIGHT_GRIPPER_ROOT_JOINT_NAMES if kind == "gripper" else ()
    if not expected:
        return False, "sparse_command_kind_invalid"
    if not _exact_names(payload.get("name"), expected):
        return False, "sparse_command_names_not_exact"
    values = _vec(payload.get("position"), len(expected))
    if values is None:
        return False, "sparse_command_positions_invalid"
    return True, "ok"


def _vec(value: Any, size: int) -> tuple[float, ...] | None:
    try: values = tuple(float(item) for item in value)
    except (TypeError, ValueError): return None
    return values if len(values) == size and all(math.isfinite(v) for v in values) else None


def build_ik_request(*, target_base_m: Any, approach_normal_base_unit: Any, tcp_frame: str, target_id: str, stamp_ns: int, scene_sha256: str) -> dict[str, Any]:
    point = _vec(target_base_m, 3); normal = _vec(approach_normal_base_unit, 3)
    if point is None or normal is None or not tcp_frame or not target_id or int(stamp_ns) <= 0 or len(scene_sha256) != 64:
        return {"schema": IK_SCHEMA, "state": "rejected", "reason": "ik_request_geometry_or_identity_invalid", "commands_emitted": False}
    norm = math.sqrt(sum(v * v for v in normal))
    if abs(norm - 1.0) > .05: return {"schema": IK_SCHEMA, "state": "rejected", "reason": "approach_normal_not_unit", "commands_emitted": False}
    return {"schema": IK_SCHEMA, "state": "ik_required", "target_base_m": list(point), "approach_normal_base_unit": list(normal), "tcp_frame": tcp_frame, "target_id": target_id, "stamp_ns": int(stamp_ns), "scene_sha256": scene_sha256, "solver": None, "commands_emitted": False}


def validate_injected_phase_targets(request: Mapping[str, Any], phase_targets: Mapping[str, Any], *, joint_names: Any, joint_limits_rad: Mapping[str, Any]) -> tuple[bool, str, dict[str, Any] | None]:
    """Validate externally supplied IK; never infer angles from Cartesian input."""
    if request.get("schema") != IK_SCHEMA or request.get("state") != "ik_required": return False, "ik_request_invalid", None
    names = tuple(str(n) for n in joint_names)
    if not _exact_names(names, RIGHT_ARM_JOINT_NAMES): return False, "right_arm_joint_names_invalid", None
    if any(name not in joint_limits_rad for name in names): return False, "right_arm_joint_limits_missing", None
    missing = [phase for phase in PHASES if phase not in phase_targets]
    if missing: return False, "ik_phase_targets_missing:" + ",".join(missing), None
    output = []
    previous: tuple[float, ...] | None = None
    for phase in PHASES:
        item = phase_targets[phase]
        if not isinstance(item, Mapping): return False, f"ik_phase_invalid:{phase}", None
        if phase in {"close", "release"}:
            values = _vec(item.get("right_gripper"), 1)
            if values is None: return False, f"ik_gripper_target_invalid:{phase}", None
            output.append({"phase": phase, "right_gripper": list(values)})
            continue
        values = _vec(item.get("right_arm_rad"), 6)
        if values is None: return False, f"ik_arm_target_invalid:{phase}", None
        for name, value in zip(names, values):
            # The caller may tighten a site limit but can never widen the
            # verified USD hard limit.
            limit = _vec(joint_limits_rad[name], 2)
            hard_limit = RIGHT_ARM_JOINT_LIMITS_RAD[name]
            if (limit is None or limit[0] > limit[1] or not limit[0] <= value <= limit[1]
                    or not hard_limit[0] <= value <= hard_limit[1]):
                return False, f"ik_joint_out_of_limits:{name}:{phase}", None
        if previous is not None and max(abs(a - b) for a, b in zip(previous, values)) > math.pi: return False, f"ik_discontinuous:{phase}", None
        previous = values; output.append({"phase": phase, "right_arm_rad": list(values), "right_joint_names": list(names)})
    return True, "ok", {"schema": "isaac_single_pen_plan/v1", "state": "ready", "simulation_only": True, "commands_emitted": False, "target_id": request["target_id"], "stamp_ns": request["stamp_ns"], "scene_sha256": request["scene_sha256"], "steps": output}
