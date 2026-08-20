"""ROS-free contract for a two-arm co-grasp of one pen.

This module deliberately produces *descriptions* of a coordinated operation.
It has no device imports and every returned command is marked as not emitted.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, sqrt
from typing import Any


PLAN_SCHEMA = "dual_pen_cograsp_plan/v1"


@dataclass(frozen=True)
class WorkspaceBounds:
    """Explicit, locally measured Cartesian limits for one tool point."""

    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float
    z_min_m: float
    z_max_m: float

    def contains(self, point: tuple[float, float, float]) -> bool:
        x, y, z = point
        return self.x_min_m <= x <= self.x_max_m and self.y_min_m <= y <= self.y_max_m and self.z_min_m <= z <= self.z_max_m


@dataclass(frozen=True)
class DualPenCograspSiteProfile:
    """Site-specific limits. Defaults fail closed until a site validates them.

    The numeric defaults are conservative contract placeholders, not claims about
    a deployed robot, gripper, pen, or collision model.  A deployment must set
    ``validated=True`` and supply both independently measured workspaces.
    """

    validated: bool = False
    left_workspace: WorkspaceBounds | None = None
    right_workspace: WorkspaceBounds | None = None
    min_confidence: float = 0.80
    max_candidate_age_ns: int = 250_000_000
    min_contact_distance_m: float = 0.040
    max_contact_distance_m: float = 0.180
    min_tool_point_clearance_m: float = 0.030
    pregrasp_offset_m: float = 0.050
    approach_offset_m: float = 0.020
    lift_distance_m: float = 0.040
    # There is intentionally no default: a validated site must establish a
    # safe lifting direction independently from the optical approach normal.
    lift_vector_base_unit: tuple[float, float, float] | None = None
    max_barrier_skew_sec: float = 0.100
    phase_timeout_sec: float = 3.0
    hold_sec: float = 3.0


# A shorter name is convenient for consumers while keeping the intent visible.
SiteProfile = DualPenCograspSiteProfile


def _finite_point(value: Any, field: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{field}_must_be_three_numbers")
    try:
        point = tuple(float(item) for item in value)
    except (TypeError, ValueError):
        raise ValueError(f"{field}_must_be_three_numbers") from None
    if not all(isfinite(item) for item in point):
        raise ValueError(f"{field}_must_be_finite")
    return point  # type: ignore[return-value]


def _distance(first: tuple[float, float, float], second: tuple[float, float, float]) -> float:
    return sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def _unit(value: Any, field: str) -> tuple[float, float, float]:
    point = _finite_point(value, field)
    magnitude = _distance(point, (0.0, 0.0, 0.0))
    if magnitude == 0.0:
        raise ValueError(f"{field}_must_be_nonzero")
    return tuple(component / magnitude for component in point)  # type: ignore[return-value]


def _finite_unit(value: Any, field: str) -> tuple[float, float, float]:
    point = _finite_point(value, field)
    magnitude = _distance(point, (0.0, 0.0, 0.0))
    if abs(magnitude - 1.0) > 1e-3:
        raise ValueError(f"{field}_must_be_unit")
    return tuple(component / magnitude for component in point)  # type: ignore[return-value]


def _offset(point: tuple[float, float, float], direction: tuple[float, float, float], distance: float) -> tuple[float, float, float]:
    return tuple(component + unit * distance for component, unit in zip(point, direction))  # type: ignore[return-value]


def _cross(first: tuple[float, float, float], second: tuple[float, float, float]) -> tuple[float, float, float]:
    return (first[1] * second[2] - first[2] * second[1], first[2] * second[0] - first[0] * second[2], first[0] * second[1] - first[1] * second[0])


def _dot(first: tuple[float, float, float], second: tuple[float, float, float]) -> float:
    return sum(a * b for a, b in zip(first, second))


def _profile_failure(profile: DualPenCograspSiteProfile) -> str | None:
    """Return stable fail-closed evidence for malformed site configuration."""
    def finite(name: str, *, positive: bool = False, nonnegative: bool = False) -> str | None:
        try:
            value = float(getattr(profile, name))
        except (TypeError, ValueError):
            return f"site_profile_invalid:{name}"
        if not isfinite(value) or (positive and value <= 0.0) or (nonnegative and value < 0.0):
            return f"site_profile_invalid:{name}"
        return None
    failure = finite("min_confidence", nonnegative=True)
    if failure:
        return failure
    for name in ("max_candidate_age_ns", "min_contact_distance_m", "max_contact_distance_m", "min_tool_point_clearance_m", "pregrasp_offset_m", "approach_offset_m", "lift_distance_m", "phase_timeout_sec", "hold_sec"):
        failure = finite(name, positive=True)
        if failure:
            return failure
    failure = finite("max_barrier_skew_sec", nonnegative=True)
    if failure:
        return failure
    if not 0.0 <= float(profile.min_confidence) <= 1.0:
        return "site_profile_invalid:min_confidence"
    if float(profile.min_contact_distance_m) > float(profile.max_contact_distance_m):
        return "site_profile_invalid:contact_distance_range"
    if not float(profile.pregrasp_offset_m) > float(profile.approach_offset_m) > 0.0:
        return "site_profile_invalid:approach_offset_m"
    for side in ("left", "right"):
        workspace = getattr(profile, f"{side}_workspace")
        if workspace is None:
            continue
        for axis in ("x", "y", "z"):
            try:
                minimum, maximum = float(getattr(workspace, f"{axis}_min_m")), float(getattr(workspace, f"{axis}_max_m"))
            except (AttributeError, TypeError, ValueError):
                return f"site_profile_invalid:{side}_workspace_{axis}"
            if not isfinite(minimum) or not isfinite(maximum) or minimum >= maximum:
                return f"site_profile_invalid:{side}_workspace_{axis}"
    if profile.lift_vector_base_unit is not None:
        try:
            _finite_unit(profile.lift_vector_base_unit, "lift_vector_base_unit")
        except ValueError:
            return "site_profile_invalid:lift_vector_base_unit"
    return None


def _stamp_ns(candidate: dict[str, Any]) -> int | None:
    value = candidate.get("stamp_ns", candidate.get("source_stamp_ns"))
    if value is None and "stamp_sec" in candidate:
        try:
            value = int(candidate["stamp_sec"]) * 1_000_000_000 + int(candidate.get("stamp_nanosec", candidate.get("nanosec", 0)))
        except (TypeError, ValueError):
            return None
    try:
        stamp = int(value)
    except (TypeError, ValueError):
        return None
    return stamp if stamp > 0 else None


def _reject(code: str, *, target_id: str | None = None) -> dict[str, Any]:
    return {"valid": False, "reason": code, "failure_code": code, "target_id": target_id, "commands_emitted": False}


def _single_candidate(payload: dict[str, Any]) -> tuple[dict[str, Any] | None, str | None]:
    """Accept a direct candidate or exactly one candidate from a producer envelope."""
    if "candidates" not in payload:
        return payload, None
    batch = payload.get("candidates")
    if not isinstance(batch, list) or len(batch) == 0:
        return None, "candidate_count_zero"
    if len(batch) != 1:
        return None, "candidate_count_must_be_exactly_one"
    declared = payload.get("candidate_count")
    if declared is not None:
        try:
            if int(declared) != len(batch):
                return None, "candidate_count_mismatch"
        except (TypeError, ValueError):
            return None, "candidate_count_mismatch"
    if not isinstance(batch[0], dict):
        return None, "candidate_not_mapping"
    nested = batch[0]
    if ("trusted_for_grasp" in payload and payload["trusted_for_grasp"] is not True) or ("trusted_for_grasp" in nested and nested["trusted_for_grasp"] is not True):
        return None, "candidate_not_validated_or_trusted"
    if ("valid" in payload and payload["valid"] is not True) or ("valid" in nested and nested["valid"] is not True):
        return None, "candidate_not_validated_or_trusted"
    # Trust and timestamp evidence belongs to the producer envelope and cannot
    # be weakened or forged by a nested candidate.
    merged = dict(nested)
    # Do not overwrite nested negative evidence.  A positive root value is
    # inherited only when the nested candidate did not supply that evidence.
    for key in ("valid", "trusted_for_grasp"):
        if key in payload and key not in merged:
            merged[key] = payload[key]
    stamp_keys = ("stamp_ns", "source_stamp_ns", "stamp_sec", "stamp_nanosec", "nanosec")
    if any(key in payload for key in stamp_keys):
        for key in stamp_keys:
            merged.pop(key, None)
        for key in stamp_keys:
            if key in payload:
                merged[key] = payload[key]
    return merged, None


def validate_dual_pen_cograsp_candidate(candidate: dict[str, Any], *, now_stamp_ns: int, profile: DualPenCograspSiteProfile = DualPenCograspSiteProfile()) -> dict[str, Any]:
    """Validate one already-validated base-frame pen candidate, fail closed.

    The two points in ``grasp_interval_base_m`` are mandatory; no center point
    or inferred endpoint may be substituted.  Higher base-link Y always becomes
    the left assignment to make the non-crossing convention explicit.
    """
    if not isinstance(candidate, dict):
        return _reject("candidate_not_mapping")
    candidate, envelope_error = _single_candidate(candidate)
    if envelope_error:
        return _reject(envelope_error)
    assert candidate is not None
    target_id = str(candidate.get("target_id") or candidate.get("pen_id") or "") or None
    if not profile.validated:
        return _reject("site_profile_not_validated", target_id=target_id)
    if profile.left_workspace is None or profile.right_workspace is None:
        return _reject("site_workspace_bounds_missing", target_id=target_id)
    profile_failure = _profile_failure(profile)
    if profile_failure:
        return _reject(profile_failure, target_id=target_id)
    if profile.lift_vector_base_unit is None:
        return _reject("lift_vector_base_unit_missing", target_id=target_id)
    if profile.approach_offset_m <= 0.0 or profile.approach_offset_m >= profile.pregrasp_offset_m:
        return _reject("approach_offset_must_be_positive_and_less_than_pregrasp_offset", target_id=target_id)
    try:
        lift_vector = _finite_unit(profile.lift_vector_base_unit, "lift_vector_base_unit")
    except ValueError as exc:
        return _reject(str(exc), target_id=target_id)
    if candidate.get("valid") is not True or candidate.get("trusted_for_grasp") is not True:
        return _reject("candidate_not_validated_or_trusted", target_id=target_id)
    if candidate.get("target_frame", candidate.get("frame_id")) != "base_link":
        return _reject("candidate_frame_must_be_base_link", target_id=target_id)
    try:
        confidence = float(candidate.get("confidence", 0.0))
    except (TypeError, ValueError):
        confidence = 0.0
    if not isfinite(confidence) or confidence < profile.min_confidence:
        return _reject("candidate_untrusted_confidence", target_id=target_id)
    stamp = _stamp_ns(candidate)
    if stamp is None or now_stamp_ns < stamp or now_stamp_ns - stamp > profile.max_candidate_age_ns:
        return _reject("candidate_stale_or_stamp_missing", target_id=target_id)
    interval = candidate.get("grasp_interval_base_m")
    if not isinstance(interval, (list, tuple)) or len(interval) != 2:
        return _reject("grasp_interval_base_m_requires_two_points", target_id=target_id)
    try:
        first, second = _finite_point(interval[0], "grasp_interval_base_m[0]"), _finite_point(interval[1], "grasp_interval_base_m[1]")
    except ValueError as exc:
        return _reject(str(exc), target_id=target_id)
    separation = _distance(first, second)
    if separation < profile.min_contact_distance_m:
        return _reject("contact_distance_too_small", target_id=target_id)
    if separation > profile.max_contact_distance_m:
        return _reject("contact_distance_too_large", target_id=target_id)
    if separation < profile.min_tool_point_clearance_m:
        return _reject("tool_point_clearance_insufficient", target_id=target_id)
    left, right = (first, second) if first[1] > second[1] else (second, first)
    if first[1] == second[1]:
        return _reject("left_right_assignment_ambiguous_equal_y", target_id=target_id)
    if not profile.left_workspace.contains(left):
        return _reject("left_workspace_out_of_bounds", target_id=target_id)
    if not profile.right_workspace.contains(right):
        return _reject("right_workspace_out_of_bounds", target_id=target_id)
    try:
        approach_normal = _finite_unit(candidate.get("approach_normal_base_unit"), "approach_normal_base_unit")
        pen_axis = _finite_unit(candidate.get("axis_base_unit"), "axis_base_unit")
    except ValueError as exc:
        return _reject(str(exc), target_id=target_id)
    lateral_raw = _cross(approach_normal, pen_axis)
    if _distance(lateral_raw, (0.0, 0.0, 0.0)) < 1e-6:
        return _reject("pen_axis_parallel_to_approach_normal", target_id=target_id)
    lateral = _unit(lateral_raw, "lateral_base_unit")
    # Gram-Schmidt preserves an outbound approach component while yielding a
    # proper right-handed orthonormal object/tool basis: axis x lateral = out.
    outbound_approach = _unit(_cross(pen_axis, lateral), "outbound_approach_base_unit")
    object_basis = [list(pen_axis), list(lateral), list(outbound_approach)]
    poses = {
        "left_pregrasp_base_m": _offset(left, approach_normal, profile.pregrasp_offset_m),
        "right_pregrasp_base_m": _offset(right, approach_normal, profile.pregrasp_offset_m),
        "left_approach_base_m": _offset(left, approach_normal, profile.approach_offset_m),
        "right_approach_base_m": _offset(right, approach_normal, profile.approach_offset_m),
        "left_lift_base_m": _offset(left, lift_vector, profile.lift_distance_m),
        "right_lift_base_m": _offset(right, lift_vector, profile.lift_distance_m),
    }
    for side, workspace in (("left", profile.left_workspace), ("right", profile.right_workspace)):
        for phase in ("pregrasp", "approach", "lift"):
            if not workspace.contains(poses[f"{side}_{phase}_base_m"]):
                return _reject(f"{side}_{phase}_workspace_out_of_bounds", target_id=target_id)
    return {
        "valid": True, "reason": "ok", "failure_code": None, "target_id": target_id,
        "source_stamp_ns": stamp, "contact_distance_m": separation,
        "left_contact_base_m": list(left), "right_contact_base_m": list(right),
        "approach_normal_base_unit": list(approach_normal), "axis_base_unit": list(pen_axis), "lift_vector_base_unit": list(lift_vector),
        "object_basis_base_columns": object_basis,
        "tcp_adapter_contract": "future_left_right_tcp_adapters_must_apply_site_validated_tool_transform;_no_joint_or_quaternion_is_inferred",
        **{name: list(point) for name, point in poses.items()},
        "commands_emitted": False,
    }


def build_dual_pen_cograsp_plan(candidate: dict[str, Any], *, now_stamp_ns: int, profile: DualPenCograspSiteProfile = DualPenCograspSiteProfile()) -> dict[str, Any]:
    """Build the no-navigation seven-phase synchronized co-grasp plan."""
    checked = validate_dual_pen_cograsp_candidate(candidate, now_stamp_ns=now_stamp_ns, profile=profile)
    plan: dict[str, Any] = {
        "schema": PLAN_SCHEMA, "mode": "offline_contract", "navigation_included": False,
        "commands_emitted": False, "candidate": checked, "state": "rejected", "steps": [],
    }
    if not checked["valid"]:
        plan["reason"] = checked["reason"]
        return plan
    left = tuple(checked["left_contact_base_m"])
    right = tuple(checked["right_contact_base_m"])
    pre_left, pre_right = tuple(checked["left_pregrasp_base_m"]), tuple(checked["right_pregrasp_base_m"])
    approach_left, approach_right = tuple(checked["left_approach_base_m"]), tuple(checked["right_approach_base_m"])
    lift_left, lift_right = tuple(checked["left_lift_base_m"]), tuple(checked["right_lift_base_m"])
    def pair(left_point: tuple[float, float, float], right_point: tuple[float, float, float]) -> dict[str, Any]:
        basis = checked["object_basis_base_columns"]
        transform = checked["tcp_adapter_contract"]
        return {"left": {"frame_id": "base_link", "tool_point_m": list(left_point), "object_basis_base_columns": basis, "tcp_adapter_contract": transform}, "right": {"frame_id": "base_link", "tool_point_m": list(right_point), "object_basis_base_columns": basis, "tcp_adapter_contract": transform}, "commands_emitted": False}
    plan.update({
        "state": "ready", "reason": "ok", "target_id": checked["target_id"],
        "assignments": {"left": list(left), "right": list(right), "rule": "higher_base_link_y_is_left"},
        "object_basis_base_columns": checked["object_basis_base_columns"],
        "tcp_adapter_contract": checked["tcp_adapter_contract"],
        "steps": [
            {"phase": "pregrasp", "name": "both_pregrasp_ready_barrier", "barrier": True, "deadline_sec": profile.phase_timeout_sec, **pair(pre_left, pre_right)},
            {"phase": "approach", "name": "synchronous_approach", "barrier": True, "deadline_sec": profile.phase_timeout_sec, **pair(approach_left, approach_right)},
            {"phase": "contact", "name": "synchronous_to_contact", "barrier": True, "deadline_sec": profile.phase_timeout_sec, **pair(left, right)},
            {"phase": "close", "name": "synchronous_close", "barrier": True, "deadline_sec": profile.phase_timeout_sec, "commands_emitted": False},
            {"phase": "confirm", "name": "confirm_both_side_grasps", "barrier": True, "deadline_sec": profile.phase_timeout_sec, "commands_emitted": False},
            {"phase": "lift", "name": "synchronous_lift", "barrier": True, "deadline_sec": profile.phase_timeout_sec, **pair(lift_left, lift_right)},
            {"phase": "hold", "name": "hold_both_grasps_3_seconds", "barrier": True, "deadline_sec": profile.hold_sec, "hold_sec": profile.hold_sec, "commands_emitted": False},
        ],
    })
    return plan
