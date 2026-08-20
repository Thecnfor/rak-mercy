"""ROS-free, fail-closed planning contract for a single pen pick.

This module intentionally produces *intents*, never actuator commands.  The
Mercury/Nav2 adapter is a later, separately reviewed boundary.  Keeping that
boundary out of this module makes offline testing useful while ensuring that a
launch of the dry-run node cannot move the robot.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .motion_adapter_contract import required_motion_adapter_contract


@dataclass(frozen=True)
class PickPlanLimits:
    """Site-specific limits; empty workspace bounds deliberately fail closed."""

    max_candidate_age_sec: float = 0.35
    min_detection_confidence: float = 0.50
    max_depth_mad_m: float = 0.012
    min_mask_depth_valid_ratio: float = 0.25
    pregrasp_clearance_m: float = 0.12
    approach_distance_m: float = 0.08
    lift_distance_m: float = 0.15
    retreat_distance_m: float = 0.18
    workspace_min_base_m: tuple[float, float, float] | None = None
    workspace_max_base_m: tuple[float, float, float] | None = None


class PickDryRunStateMachine:
    """Small explicit state machine with no transition that emits a command."""

    def __init__(self) -> None:
        self.state = "waiting_for_candidate"

    def transition(self, payload: dict[str, Any], **kwargs: Any) -> dict[str, Any]:
        previous_state = self.state
        result = build_dry_run_plan(payload, **kwargs)
        self.state = str(result["state"])
        result["previous_state"] = previous_state
        result["state"] = self.state
        return result


def _finite_vector(value: Any, name: str, size: int = 3) -> np.ndarray:
    try:
        vector = np.asarray(value, dtype=np.float64).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name}_must_be_numeric") from exc
    if vector.size != size or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name}_must_be_{size}_finite_numbers")
    return vector


def _unit_vector(value: Any, name: str) -> np.ndarray:
    vector = _finite_vector(value, name)
    magnitude = float(np.linalg.norm(vector))
    if magnitude < 1e-6:
        raise ValueError(f"{name}_zero_length")
    return vector / magnitude


def _stamp_ns(payload: dict[str, Any]) -> int:
    return int(payload.get("stamp_sec", 0) or 0) * 1_000_000_000 + int(payload.get("stamp_nanosec", 0) or 0)


def _workspace(limits: PickPlanLimits) -> tuple[np.ndarray, np.ndarray]:
    if limits.workspace_min_base_m is None or limits.workspace_max_base_m is None:
        raise ValueError("workspace_bounds_not_configured")
    lower = _finite_vector(limits.workspace_min_base_m, "workspace_min_base_m")
    upper = _finite_vector(limits.workspace_max_base_m, "workspace_max_base_m")
    if np.any(lower >= upper):
        raise ValueError("workspace_bounds_invalid")
    return lower, upper


def _inside_workspace(point: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> bool:
    return bool(np.all(point >= lower) and np.all(point <= upper))


def _basis(axis: np.ndarray, normal: np.ndarray) -> np.ndarray:
    """Return a right-handed tool basis whose +Z is the outbound approach axis."""
    tool_z = normal
    tool_x = axis - float(axis @ tool_z) * tool_z
    if float(np.linalg.norm(tool_x)) < 1e-4:
        raise ValueError("pen_axis_parallel_to_approach_normal")
    tool_x /= float(np.linalg.norm(tool_x))
    tool_y = np.cross(tool_z, tool_x)
    return np.column_stack((tool_x, tool_y, tool_z))


def build_dry_run_plan(
    payload: dict[str, Any], *, now_stamp_ns: int, limits: PickPlanLimits = PickPlanLimits(),
    site_profile_validated: bool = False, enable_execution: bool = False,
    operator_approved: bool = False, include_navigation_gate: bool = False,
) -> dict[str, Any]:
    """Validate one candidate and return immutable pre-grasp through retreat intents.

    A plan can be ``dry_run_ready`` only after perception and workspace gates.
    It is never executable here: this package intentionally has no navigation,
    arm, or gripper action client.
    """
    reasons: list[str] = []
    if not isinstance(payload, dict):
        return {"state": "rejected", "reason": "candidate_payload_invalid", "commands_emitted": False}
    if int(payload.get("candidate_count", 0) or 0) != 1:
        reasons.append("candidate_count_must_be_one")
    candidates = payload.get("candidates")
    candidate = candidates[0] if isinstance(candidates, list) and len(candidates) == 1 and isinstance(candidates[0], dict) else None
    if candidate is None:
        reasons.append("candidate_missing")
    if payload.get("valid") is not True or payload.get("trusted_for_grasp") is not True:
        reasons.append("candidate_not_validated_for_grasp")
    stamp_ns = _stamp_ns(payload)
    age_ns = now_stamp_ns - stamp_ns
    if stamp_ns <= 0 or age_ns < 0 or age_ns > int(limits.max_candidate_age_sec * 1e9):
        reasons.append("candidate_timestamp_stale_or_invalid")
    if candidate is not None:
        if candidate.get("valid") is not True or candidate.get("trusted_for_grasp") is not True:
            reasons.append("candidate_not_validated_for_grasp")
        if candidate.get("target_frame") != "base_link":
            reasons.append("candidate_frame_must_be_base_link")
    if reasons:
        return {"state": "rejected", "reason": reasons[0], "reasons": reasons, "commands_emitted": False}

    assert candidate is not None
    try:
        target = _finite_vector(candidate.get("grasp_point_base_m"), "grasp_point_base_m")
        axis = _unit_vector(candidate.get("axis_base_unit"), "axis_base_unit")
        normal = _unit_vector(candidate.get("approach_normal_base_unit"), "approach_normal_base_unit")
        quality = candidate.get("quality") if isinstance(candidate.get("quality"), dict) else {}
        confidence = float(quality.get("detection_confidence", candidate.get("confidence", 0.0)) or 0.0)
        depth_mad = float(quality.get("depth_mad_m", candidate.get("depth_mad_m", float("inf"))) or float("inf"))
        valid_ratio = float(quality.get("mask_depth_valid_ratio", candidate.get("mask_depth_valid_ratio", 0.0)) or 0.0)
        if not np.isfinite(confidence) or confidence < limits.min_detection_confidence:
            reasons.append("detection_confidence_below_limit")
        if not np.isfinite(depth_mad) or depth_mad > limits.max_depth_mad_m:
            reasons.append("depth_mad_exceeds_limit")
        if not np.isfinite(valid_ratio) or valid_ratio < limits.min_mask_depth_valid_ratio:
            reasons.append("mask_depth_valid_ratio_below_limit")
        lower, upper = _workspace(limits)
        basis = _basis(axis, normal)
    except ValueError as exc:
        reasons.append(str(exc))
        lower = upper = basis = None

    if reasons:
        return {"state": "rejected", "reason": reasons[0], "reasons": reasons, "commands_emitted": False}
    assert lower is not None and upper is not None and basis is not None
    # The normal has already been transformed into base_link.  Its sign is part
    # of the site profile and must be physically verified before an adapter is
    # allowed to execute this intent.
    poses = {
        "pre_grasp": target + normal * limits.pregrasp_clearance_m,
        "approach": target + normal * limits.approach_distance_m,
        "grasp": target,
        "lift": target + normal * limits.lift_distance_m,
        "safe_retreat": target + normal * limits.retreat_distance_m,
    }
    for name, point in poses.items():
        if not _inside_workspace(point, lower, upper):
            return {"state": "rejected", "reason": f"{name}_outside_workspace", "commands_emitted": False}
    pose_payload = {name: {"frame_id": "base_link", "position_m": [round(float(v), 5) for v in point], "tool_basis_columns_base": [[round(float(v), 6) for v in row] for row in basis]} for name, point in poses.items()}
    execution_blocks = ["motion_adapter_not_implemented"]
    if not site_profile_validated:
        execution_blocks.append("site_profile_not_validated")
    if not enable_execution:
        execution_blocks.append("enable_execution_false")
    if not operator_approved:
        execution_blocks.append("operator_approval_missing")
    steps: list[dict[str, Any]] = []
    if include_navigation_gate:
        steps.append({"name": "verify_navigation_arrival", "kind": "optional_gate", "requires": ["site_profile", "Nav2 arrival evidence"]})
    steps.extend([
        {"name": "pre_grasp", "kind": "cartesian_intent", "pose": pose_payload["pre_grasp"]},
        {"name": "approach", "kind": "cartesian_intent", "pose": pose_payload["approach"]},
        {"name": "grasp", "kind": "cartesian_intent", "pose": pose_payload["grasp"]},
        {"name": "close_gripper", "kind": "gripper_intent"},
        {"name": "lift", "kind": "cartesian_intent", "pose": pose_payload["lift"]},
        {"name": "safe_retreat", "kind": "cartesian_intent", "pose": pose_payload["safe_retreat"]},
    ])
    return {
        "state": "dry_run_ready",
        "reason": "ok",
        "commands_emitted": False,
        "target_id": str(candidate.get("target_id") or candidate.get("pen_id") or "pen"),
        "candidate_stamp_ns": stamp_ns,
        "tool_orientation_contract": "columns=[gripper_long_axis, lateral_axis, outbound_approach_axis]",
        "navigation_gate_included": include_navigation_gate,
        "steps": steps,
        "execution_eligible": False,
        "execution_block_reasons": execution_blocks,
        "required_motion_adapter_contract": required_motion_adapter_contract(),
    }
