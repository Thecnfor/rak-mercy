"""Pure validation and geometry helpers for the one-shot right-arm executor."""

from __future__ import annotations

from dataclasses import dataclass
from math import asin, atan2, degrees, isfinite
from typing import Any

import numpy as np


ALLOWED_CARTESIAN_STAGES = {"pre_grasp", "approach", "grasp", "lift", "return_to_grasp", "safe_retreat"}


@dataclass(frozen=True)
class RightArmExecutionProfile:
    validated: bool = False
    arm_side: str = "right"
    serial_port: str = "/dev/right_arm"
    joint_min_deg: tuple[float, ...] = ()
    joint_max_deg: tuple[float, ...] = ()
    workspace_min_base_m: tuple[float, ...] = ()
    workspace_max_base_m: tuple[float, ...] = ()
    max_cartesian_speed_m_s: float = .02
    max_tracking_error_m: float = .01
    max_feedback_gap_sec: float = .25
    vendor_speed: int = 5
    gripper_open_value: int = 100
    gripper_closed_value: int = 0
    gripper_speed: int = 10
    gripper_direction_validated: bool = False
    orientation_convention_validated: bool = False


def profile_from_mapping(value: Any) -> RightArmExecutionProfile:
    if not isinstance(value, dict):
        raise ValueError("right_arm_profile_must_be_mapping")
    fields = RightArmExecutionProfile.__dataclass_fields__
    data = {name: value[name] for name in fields if name in value}
    for name in ("joint_min_deg", "joint_max_deg", "workspace_min_base_m", "workspace_max_base_m"):
        if name in data: data[name] = tuple(float(item) for item in data[name])
    return RightArmExecutionProfile(**data)


def validate_profile(profile: RightArmExecutionProfile) -> tuple[bool, str]:
    if not profile.validated: return False, "right_arm_site_profile_not_validated"
    if profile.arm_side != "right": return False, "arm_side_must_be_right"
    if profile.serial_port != "/dev/right_arm": return False, "serial_port_must_be_dev_right_arm"
    if len(profile.joint_min_deg) != 6 or len(profile.joint_max_deg) != 6: return False, "joint_limits_must_have_six_values"
    if len(profile.workspace_min_base_m) != 3 or len(profile.workspace_max_base_m) != 3: return False, "workspace_bounds_must_have_three_values"
    values = (*profile.joint_min_deg,*profile.joint_max_deg,*profile.workspace_min_base_m,*profile.workspace_max_base_m,profile.max_cartesian_speed_m_s,profile.max_tracking_error_m,profile.max_feedback_gap_sec)
    if not all(isfinite(float(item)) for item in values): return False, "profile_values_must_be_finite"
    if any(a >= b for a,b in zip(profile.joint_min_deg,profile.joint_max_deg)): return False, "joint_limits_invalid"
    if any(a >= b for a,b in zip(profile.workspace_min_base_m,profile.workspace_max_base_m)): return False, "workspace_bounds_invalid"
    if not 0 < profile.max_cartesian_speed_m_s <= .02: return False, "cartesian_speed_limit_invalid"
    if not 0 < profile.max_tracking_error_m <= .01: return False, "tracking_error_limit_invalid"
    if not 0 < profile.max_feedback_gap_sec <= .25: return False, "feedback_gap_limit_invalid"
    if not 1 <= profile.vendor_speed <= 5: return False, "vendor_speed_limit_invalid"
    if not profile.gripper_direction_validated: return False, "gripper_direction_not_validated"
    if not profile.orientation_convention_validated: return False, "orientation_convention_not_validated"
    return True, "ok"


def basis_to_xyz_euler_deg(basis_rows: Any) -> list[float]:
    """Convert a right-handed rotation matrix to intrinsic XYZ Euler degrees."""
    matrix = np.asarray(basis_rows,dtype=float)
    if matrix.shape != (3,3) or not np.all(np.isfinite(matrix)):
        raise ValueError("tool_basis_must_be_3x3_finite")
    if not np.allclose(matrix.T@matrix,np.eye(3),atol=2e-3) or np.linalg.det(matrix) < .99:
        raise ValueError("tool_basis_must_be_right_handed_orthonormal")
    sy = float(np.clip(matrix[0,2],-1.,1.))
    ry = asin(sy)
    if abs(abs(sy)-1.) > 1e-6:
        rx = atan2(-matrix[1,2],matrix[2,2]); rz = atan2(-matrix[0,1],matrix[0,0])
    else:
        rx = atan2(matrix[2,1],matrix[1,1]); rz = 0.
    return [degrees(rx),degrees(ry),degrees(rz)]


def validate_cartesian_goal(goal: Any, profile: RightArmExecutionProfile, *, calibration_id: str) -> tuple[bool,str,list[float]]:
    valid,reason=validate_profile(profile)
    if not valid:return False,reason,[]
    if str(getattr(goal,"arm_side","")) != "right":return False,"goal_arm_side_must_be_right",[]
    if str(getattr(goal,"stage","")) not in ALLOWED_CARTESIAN_STAGES:return False,"cartesian_stage_invalid",[]
    transaction=str(getattr(goal,"transaction_id","")); expected_calibration=str(getattr(goal,"calibration_id",""))
    if not transaction.startswith("pick-"):return False,"transaction_id_invalid",[]
    if not calibration_id or expected_calibration != calibration_id:return False,"calibration_identity_mismatch",[]
    try: pose=[float(item) for item in goal.pose_base]
    except (TypeError,ValueError):return False,"pose_base_invalid",[]
    if len(pose)!=6 or not all(isfinite(item) for item in pose):return False,"pose_base_invalid",[]
    if any(value<low or value>high for value,low,high in zip(pose[:3],profile.workspace_min_base_m,profile.workspace_max_base_m)):return False,"cartesian_target_outside_workspace",[]
    speed=float(getattr(goal,"max_speed_m_s",0.)); timeout=float(getattr(goal,"timeout_sec",0.))
    if not 0<speed<=profile.max_cartesian_speed_m_s:return False,"cartesian_speed_exceeds_limit",[]
    if not 0<timeout<=15.:return False,"stage_timeout_invalid",[]
    return True,"ok",pose


def build_action_steps(plan: Any) -> tuple[list[dict[str,Any]],str]:
    if not isinstance(plan,dict) or plan.get("state")!="dry_run_ready":return [],"dry_run_plan_invalid"
    transaction=str(plan.get("transaction_id") or ""); calibration=str(plan.get("calibration_id") or "")
    if not transaction.startswith("pick-") or not calibration:return [],"plan_identity_invalid"
    raw={str(item.get("name")):item for item in plan.get("steps",[]) if isinstance(item,dict)}
    required=("pre_grasp","approach","grasp","close_gripper","lift","safe_retreat")
    if any(name not in raw for name in required):return [],"plan_steps_incomplete"
    def cartesian(name:str,source:str|None=None)->dict[str,Any]:
        item=raw[source or name];pose=item.get("pose") or {};basis=pose.get("tool_basis_columns_base")
        position=pose.get("position_m")
        if not isinstance(position,list) or len(position)!=3:raise ValueError("plan_position_invalid")
        return {"kind":"cartesian","stage":name,"pose_base":[*map(float,position),*basis_to_xyz_euler_deg(basis)]}
    try:
        steps=[{"kind":"gripper","action":"open"},cartesian("pre_grasp"),cartesian("approach"),cartesian("grasp"),{"kind":"gripper","action":"close"},cartesian("lift"),{"kind":"hold","duration_sec":2.0},cartesian("return_to_grasp","grasp"),{"kind":"gripper","action":"open"},cartesian("safe_retreat")]
    except (TypeError,ValueError) as exc:return [],str(exc)
    for step in steps:step.update({"transaction_id":transaction,"calibration_id":calibration,"arm_side":"right"})
    return steps,"ok"
