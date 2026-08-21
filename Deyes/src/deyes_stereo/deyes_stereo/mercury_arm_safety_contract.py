"""Fail-closed low-speed commissioning boundary for Mercury X1 arms.

This module is deliberately ROS- and SDK-free.  It validates a *proposed*
single-arm joint or Cartesian command and returns a preview only.  It has no
code path that can open a serial device, call pymycobot, publish JointState,
or issue a ROS action goal.  A future live adapter must separately prove the
capabilities declared in :mod:`motion_adapter_contract`.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Any


ARM_SIDES = ("left", "right")
COMMAND_KINDS = ("joint_position", "cartesian_pose", "gripper")


@dataclass(frozen=True)
class MercuryArmSafetyProfile:
    """Measured commissioning limits for exactly one physical arm.

    Defaults intentionally contain no work envelope or joint bounds.  They
    therefore cannot authorize even a dry-run preview until an operator has
    entered locally measured values.  Speed defaults are deliberately slow;
    they are limits for a future adapter, never a statement about vendor SDK
    units or robot capability.
    """

    arm_side: str = ""
    joint_min_deg: tuple[float, float, float, float, float, float] | None = None
    joint_max_deg: tuple[float, float, float, float, float, float] | None = None
    workspace_min_base_m: tuple[float, float, float] | None = None
    workspace_max_base_m: tuple[float, float, float] | None = None
    max_joint_speed_deg_s: float = 5.0
    max_cartesian_speed_m_s: float = 0.02
    max_cartesian_accel_m_s2: float = 0.05
    max_gripper_effort_percent: float = 20.0
    dry_run: bool = True


def _finite_vector(value: Any, *, size: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f"{field}_must_be_{size}_finite_values")
    try:
        values = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field}_must_be_{size}_finite_values") from exc
    if not all(isfinite(item) for item in values):
        raise ValueError(f"{field}_must_be_{size}_finite_values")
    return values


def _profile_failure(profile: MercuryArmSafetyProfile, *, kind: str) -> str | None:
    if profile.arm_side not in ARM_SIDES:
        return "selected_arm_must_be_left_or_right"
    for name, positive in (
        ("max_joint_speed_deg_s", True),
        ("max_cartesian_speed_m_s", True),
        ("max_cartesian_accel_m_s2", True),
        ("max_gripper_effort_percent", False),
    ):
        try:
            value = float(getattr(profile, name))
        except (TypeError, ValueError):
            return f"profile_{name}_invalid"
        if not isfinite(value) or (positive and value <= 0.0) or (not positive and value < 0.0):
            return f"profile_{name}_invalid"
    if kind == "joint_position":
        if profile.joint_min_deg is None or profile.joint_max_deg is None:
            return "joint_limits_not_configured"
        try:
            lower = _finite_vector(profile.joint_min_deg, size=6, field="joint_min_deg")
            upper = _finite_vector(profile.joint_max_deg, size=6, field="joint_max_deg")
        except ValueError as exc:
            return str(exc)
        if any(low >= high for low, high in zip(lower, upper)):
            return "joint_limits_invalid"
    if kind == "cartesian_pose":
        if profile.workspace_min_base_m is None or profile.workspace_max_base_m is None:
            return "cartesian_workspace_not_configured"
        try:
            lower = _finite_vector(profile.workspace_min_base_m, size=3, field="workspace_min_base_m")
            upper = _finite_vector(profile.workspace_max_base_m, size=3, field="workspace_max_base_m")
        except ValueError as exc:
            return str(exc)
        if any(low >= high for low, high in zip(lower, upper)):
            return "cartesian_workspace_invalid"
    return None


def _result(*, state: str, reason: str, profile: MercuryArmSafetyProfile, kind: str, preview: dict[str, Any] | None = None) -> dict[str, Any]:
    return {
        "state": state,
        "reason": reason,
        "failure_code": "" if reason == "ok" else reason,
        "command_kind": kind,
        "selected_arm": profile.arm_side,
        "dry_run": bool(profile.dry_run),
        "execution_permitted": False,
        "commands_emitted": False,
        "motion_command_emitted": False,
        "gripper_command_emitted": False,
        "limits": {
            "max_joint_speed_deg_s": profile.max_joint_speed_deg_s,
            "max_cartesian_speed_m_s": profile.max_cartesian_speed_m_s,
            "max_cartesian_accel_m_s2": profile.max_cartesian_accel_m_s2,
            "max_gripper_effort_percent": profile.max_gripper_effort_percent,
        },
        "preview": preview,
    }


def validate_low_speed_motion_request(request: dict[str, Any], profile: MercuryArmSafetyProfile = MercuryArmSafetyProfile()) -> dict[str, Any]:
    """Validate one intent and produce a non-executable preview.

    ``request_execution`` is deliberately rejected even when supplied.  This
    prevents a parameter typo or an optimistic caller from converting this
    commissioning layer into a live motion path.
    """
    if not isinstance(request, dict):
        return _result(state="rejected", reason="motion_request_invalid", profile=profile, kind="")
    kind = str(request.get("kind") or "")
    if kind not in COMMAND_KINDS:
        return _result(state="rejected", reason="motion_command_kind_invalid", profile=profile, kind=kind)
    if request.get("request_execution") is True:
        return _result(state="rejected", reason="live_execution_not_implemented", profile=profile, kind=kind)
    if profile.dry_run is not True:
        return _result(state="rejected", reason="dry_run_must_remain_true", profile=profile, kind=kind)
    failure = _profile_failure(profile, kind=kind)
    if failure:
        return _result(state="rejected", reason=failure, profile=profile, kind=kind)

    try:
        if kind == "joint_position":
            positions = _finite_vector(request.get("positions_deg"), size=6, field="positions_deg")
            speed = float(request.get("speed_deg_s", profile.max_joint_speed_deg_s))
            lower = _finite_vector(profile.joint_min_deg, size=6, field="joint_min_deg")  # type: ignore[arg-type]
            upper = _finite_vector(profile.joint_max_deg, size=6, field="joint_max_deg")  # type: ignore[arg-type]
            if not isfinite(speed) or speed <= 0.0 or speed > profile.max_joint_speed_deg_s:
                raise ValueError("joint_speed_exceeds_low_speed_limit")
            if any(value < low or value > high for value, low, high in zip(positions, lower, upper)):
                raise ValueError("joint_target_outside_configured_limits")
            preview = {"positions_deg": list(positions), "speed_deg_s": speed}
        elif kind == "cartesian_pose":
            if request.get("frame_id") != "base_link":
                raise ValueError("cartesian_frame_must_be_base_link")
            position = _finite_vector(request.get("position_m"), size=3, field="position_m")
            speed = float(request.get("speed_m_s", profile.max_cartesian_speed_m_s))
            acceleration = float(request.get("acceleration_m_s2", profile.max_cartesian_accel_m_s2))
            lower = _finite_vector(profile.workspace_min_base_m, size=3, field="workspace_min_base_m")  # type: ignore[arg-type]
            upper = _finite_vector(profile.workspace_max_base_m, size=3, field="workspace_max_base_m")  # type: ignore[arg-type]
            if not isfinite(speed) or speed <= 0.0 or speed > profile.max_cartesian_speed_m_s:
                raise ValueError("cartesian_speed_exceeds_low_speed_limit")
            if not isfinite(acceleration) or acceleration <= 0.0 or acceleration > profile.max_cartesian_accel_m_s2:
                raise ValueError("cartesian_acceleration_exceeds_low_speed_limit")
            if any(value < low or value > high for value, low, high in zip(position, lower, upper)):
                raise ValueError("cartesian_target_outside_configured_workspace")
            preview = {"frame_id": "base_link", "position_m": list(position), "speed_m_s": speed, "acceleration_m_s2": acceleration}
        else:
            action = str(request.get("action") or "")
            effort = float(request.get("effort_percent", profile.max_gripper_effort_percent))
            if action not in {"open", "close"}:
                raise ValueError("gripper_action_must_be_open_or_close")
            if not isfinite(effort) or effort < 0.0 or effort > profile.max_gripper_effort_percent:
                raise ValueError("gripper_effort_exceeds_low_speed_limit")
            preview = {"action": action, "effort_percent": effort}
    except (TypeError, ValueError) as exc:
        return _result(state="rejected", reason=str(exc), profile=profile, kind=kind)
    return _result(state="dry_run_ready", reason="ok", profile=profile, kind=kind, preview=preview)


def build_single_joint_jog_preview(
    *, current_positions_deg: Any, joint_index: Any, delta_deg: Any,
    speed_deg_s: Any = 2.0, profile: MercuryArmSafetyProfile = MercuryArmSafetyProfile(),
) -> dict[str, Any]:
    """Build the sole permitted first-motion *preview* for one arm.

    It requires actual current feedback supplied by a future adapter and
    changes one of the six arm joints by at most one degree.  The returned
    structure still has ``commands_emitted:false`` and cannot move hardware.
    """
    try:
        current = _finite_vector(current_positions_deg, size=6, field="current_positions_deg")
        index = int(joint_index)
        delta = float(delta_deg)
        speed = float(speed_deg_s)
        if index < 0 or index >= 6:
            raise ValueError("joint_index_must_be_0_through_5")
        if not isfinite(delta) or delta == 0.0 or abs(delta) > 1.0:
            raise ValueError("single_joint_delta_must_be_within_0_to_1_deg")
        if not isfinite(speed) or speed <= 0.0 or speed > 2.0:
            raise ValueError("single_joint_jog_speed_must_not_exceed_2_deg_s")
    except (TypeError, ValueError) as exc:
        return _result(state="rejected", reason=str(exc), profile=profile, kind="joint_position")
    target = list(current)
    target[index] += delta
    result = validate_low_speed_motion_request(
        {"kind": "joint_position", "positions_deg": target, "speed_deg_s": speed}, profile,
    )
    result["jog"] = {
        "joint_index": index, "delta_deg": delta, "requires_current_joint_feedback": True,
        "requires_operator_deadman": True, "requires_e_stop_accessible": True,
    }
    return result
