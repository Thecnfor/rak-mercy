"""Fail-closed admission rules for the ROS2 Mercury pick adapter.

The adapter consumes a trusted TF2 coordinate result and a dry-run plan, but
never infers joint angles from Cartesian poses.  A reviewed IK/trajectory
producer must attach a named trajectory before FollowJointTrajectory may be
used.  These rules are ROS-free so simulation exercises the same gates.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Any


@dataclass(frozen=True)
class LiveExecutionGates:
    dry_run: bool = True
    enable_live_execution: bool = False
    operator_confirmed: bool = False
    validated_calibration: bool = False
    action_server_available: bool = False
    joint_state_stamp_ns: int = 0
    now_stamp_ns: int = 0
    max_joint_state_age_sec: float = 0.25


def _joint_state_fresh(gates: LiveExecutionGates) -> bool:
    age = gates.now_stamp_ns - gates.joint_state_stamp_ns
    return gates.joint_state_stamp_ns > 0 and age >= 0 and age <= int(gates.max_joint_state_age_sec * 1e9)


def validate_execution_admission(coordinate_result: Any, plan: Any, gates: LiveExecutionGates) -> tuple[bool, str]:
    """Return live eligibility only after all gates have passed.

    ``dry_run`` is intentionally first: a default node therefore has no
    possible transition into a hardware command path.
    """
    if gates.dry_run:
        return False, "dry_run_enabled"
    if not gates.enable_live_execution:
        return False, "enable_live_execution_false"
    if not gates.operator_confirmed:
        return False, "operator_confirmation_missing"
    if not gates.validated_calibration:
        return False, "validated_calibration_missing"
    if not isinstance(coordinate_result, dict) or coordinate_result.get("trusted_for_execution") is not True:
        return False, "coordinate_result_not_trusted_for_execution"
    if coordinate_result.get("frame_id") != "base_link":
        return False, "coordinate_result_frame_must_be_base_link"
    if not isinstance(plan, dict) or plan.get("state") != "dry_run_ready" or plan.get("commands_emitted") is not False:
        return False, "dry_run_plan_invalid"
    if str(coordinate_result.get("candidate_id") or "") != str(plan.get("target_id") or ""):
        return False, "coordinate_plan_target_mismatch"
    if int(coordinate_result.get("stamp_ns", -1) or -1) != int(plan.get("candidate_stamp_ns", -2) or -2):
        return False, "coordinate_plan_stamp_mismatch"
    if not gates.action_server_available:
        return False, "follow_joint_trajectory_server_unavailable"
    if not _joint_state_fresh(gates):
        return False, "joint_state_stale_or_missing"
    return True, "ok"


def validate_single_step(plan: Any, step: Any, *, operator_step_confirmed: bool) -> tuple[bool, str]:
    """Validate one explicitly confirmed stage and its reviewed joint target."""
    if not operator_step_confirmed:
        return False, "single_step_confirmation_missing"
    if not isinstance(plan, dict) or not isinstance(plan.get("steps"), list):
        return False, "dry_run_plan_invalid"
    item = next((entry for entry in plan["steps"] if isinstance(entry, dict) and entry.get("name") == step), None)
    if item is None:
        return False, "plan_step_missing"
    if step == "close_gripper":
        return (True, "ok") if item.get("kind") == "gripper_intent" else (False, "gripper_step_invalid")
    trajectory = item.get("approved_joint_trajectory")
    if not isinstance(trajectory, dict):
        return False, "approved_joint_trajectory_missing"
    names, positions = trajectory.get("joint_names"), trajectory.get("positions")
    if not isinstance(names, list) or not names or not all(isinstance(name, str) and name for name in names):
        return False, "trajectory_joint_names_invalid"
    if not isinstance(positions, list) or len(positions) != len(names):
        return False, "trajectory_positions_invalid"
    try:
        values = [float(value) for value in positions]
    except (TypeError, ValueError):
        return False, "trajectory_positions_invalid"
    if not all(isfinite(value) for value in values):
        return False, "trajectory_positions_invalid"
    return True, "ok"
