"""ROS-free policy contract for the competition showcase continuation path."""

from __future__ import annotations

from typing import Any, Sequence


SHOWCASE_XY_MM = (400.0, 10.0)
SHOWCASE_CONTACT_Z_M = 0.135
SHOWCASE_ORIENTATION_DEG = (179.99, -12.0, 0.0)
RECOVERABLE_PICK_FAILURE_CLASSES = frozenset({"object_absent", "perception"})


def build_showcase_target(reason: str) -> dict[str, Any]:
    """Build a fixed motion-demonstration target without claiming sensor trust."""
    if not isinstance(reason, str) or not reason.strip():
        raise ValueError("showcase degraded reason must be a non-empty string")
    return {
        "schema": "competition_showcase_target/v1",
        "sensor_target_available": False,
        "synthetic_target": True,
        "degraded": True,
        "degraded_reason": reason.strip(),
        "target_xy_mm": list(SHOWCASE_XY_MM),
        "right_arm_sdk_target_m": [
            SHOWCASE_XY_MM[0] / 1000.0,
            SHOWCASE_XY_MM[1] / 1000.0,
            SHOWCASE_CONTACT_Z_M,
        ],
        "orientation_deg": list(SHOWCASE_ORIENTATION_DEG),
        "commands_emitted": False,
    }


def validate_showcase_target(candidate: Any) -> dict[str, Any]:
    """Validate the separate fixed-marker schema used only for motion display."""
    if not isinstance(candidate, dict) or candidate.get("schema") != "competition_showcase_target/v1":
        raise ValueError("showcase target schema mismatch")
    forbidden = {"trusted_for_venue_execution", "force_fixed_target"}
    if forbidden.intersection(candidate):
        raise ValueError("showcase target must not claim competition target trust")
    expected = build_showcase_target(str(candidate.get("degraded_reason", "")))
    if candidate != expected:
        raise ValueError("showcase target fixed-marker contract mismatch")
    return dict(candidate)


def decide_pick_continuation(
    result: dict[str, Any], *, showcase_enabled: bool
) -> dict[str, Any]:
    """Return the next runner action without changing grasp truth fields."""
    if result.get("schema") != "competition_grasp_verification/v1":
        raise ValueError("pick result schema mismatch")
    reason = str(result.get("reason", "pick_result_invalid"))
    hardware_complete = (
        result.get("motion_completed") is True
        and result.get("transport_pose_reached") is True
        and result.get("hardware_ok") is True
    )
    object_verified = result.get("object_grasp_verified") is True
    competition_success = (
        hardware_complete
        and object_verified
        and result.get("success") is True
        and result.get("navigation_permitted") is True
    )
    if competition_success:
        return {
            "action": "continue_verified",
            "competition_success": True,
            "object_grasp_verified": True,
            "degraded_reason": None,
        }
    if (
        hardware_complete
        and showcase_enabled
        and result.get("verification_failure_class")
        in RECOVERABLE_PICK_FAILURE_CLASSES
    ):
        return {
            "action": "continue_showcase",
            "competition_success": False,
            "object_grasp_verified": False,
            "degraded_reason": reason,
        }
    return {
        "action": "stop",
        "competition_success": False,
        "object_grasp_verified": object_verified,
        "degraded_reason": reason,
    }


def build_transaction_result(
    *,
    transaction_id: str,
    showcase_mode: bool,
    competition_success: bool,
    showcase_complete: bool,
    object_grasp_verified: bool,
    target_source: str,
    pick_motion_completed: bool,
    transport_pose_reached: bool,
    goal4_completed: bool,
    place_motion_completed: bool,
    degraded_reasons: Sequence[str],
    hard_stop_reason: str | None,
) -> dict[str, Any]:
    """Build the terminal transaction record and enforce truthful outcomes."""
    if not transaction_id or not target_source:
        raise ValueError("transaction_id and target_source are required")
    all_showcase_motions = (
        pick_motion_completed
        and transport_pose_reached
        and goal4_completed
        and place_motion_completed
    )
    if showcase_complete and not all_showcase_motions:
        raise ValueError("showcase_complete requires every motion stage")
    if competition_success and not (showcase_complete and object_grasp_verified):
        raise ValueError("competition_success requires verified object and full sequence")
    if hard_stop_reason is not None and showcase_complete:
        raise ValueError("hard stop cannot be a completed showcase")
    reasons = [str(reason) for reason in degraded_reasons if str(reason)]
    return {
        "schema": "competition_transaction_result/v1",
        "transaction_id": transaction_id,
        "showcase_mode": bool(showcase_mode),
        "competition_success": bool(competition_success),
        "showcase_complete": bool(showcase_complete),
        "object_grasp_verified": bool(object_grasp_verified),
        "target_source": target_source,
        "pick_motion_completed": bool(pick_motion_completed),
        "transport_pose_reached": bool(transport_pose_reached),
        "goal4_completed": bool(goal4_completed),
        "place_motion_completed": bool(place_motion_completed),
        "degraded_reasons": reasons,
        "hard_stop_reason": hard_stop_reason,
        "commands_emitted": bool(
            pick_motion_completed or goal4_completed or place_motion_completed
        ),
    }
