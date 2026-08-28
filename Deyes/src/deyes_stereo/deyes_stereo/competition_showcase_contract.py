"""ROS-free policy contract for the competition showcase continuation path."""

from __future__ import annotations

import math
from typing import Any, Mapping, Sequence


SHOWCASE_XY_MM = (400.0, 10.0)
SHOWCASE_CONTACT_Z_M = 0.135
SHOWCASE_ORIENTATION_DEG = (179.99, -12.0, 0.0)
RECOVERABLE_PICK_FAILURE_CLASSES = frozenset({"object_absent", "perception"})
RECOVERABLE_SHOWCASE_FAILURES = frozenset(
    {
        "runtime_camera_exit",
        "runtime_cuda_exit",
        "runtime_yolo_exit",
        "target_timeout",
        "target_zero_objects",
        "target_multiple_objects",
        "target_zero_or_multiple_objects",
        "exact_stamp_mismatch",
        "depth_invalid",
        "camera_info_invalid",
        "pen_feature_invalid",
        "pnp_rejected",
        "roi_verification_unavailable",
        "object_not_verified",
    }
)


def classify_showcase_failure(failure_code: str, *, showcase_enabled: bool) -> str:
    """Classify a canonical failure code; unknown failures always stop."""
    if showcase_enabled and failure_code in RECOVERABLE_SHOWCASE_FAILURES:
        return "continue_showcase"
    return "hard_stop"


def runtime_perception_failure_code(reason: str) -> str:
    """Normalize runner details without weakening the healthy-plane hard gate."""
    value = reason.lower()
    if "rc=4" in value:
        return "target_configuration_or_contract"
    if "table_height_deviation_exceeds_25mm" in value:
        return "healthy_plane_deviation_over_25mm"
    if any(token in value for token in (
        "parameter_environment_mismatch",
        "configuration error",
        "projector_evidence_invalid",
        "venue_profile_schema_invalid",
        "python unavailable",
        "json malformed",
    )):
        return "target_configuration_or_contract"
    if "runtime_vision_launch_exited" in value:
        return "runtime_camera_exit"
    if "competition_target_node_exited" in value:
        return "target_node_exit"
    if "timeout" in value or "rc=2" in value:
        return "target_timeout"
    if "detection_count_must_be_exactly_one" in value:
        return "target_zero_or_multiple_objects"
    if any(token in value for token in (
        "zero", "no_eligible", "no_observed", "detection_not_complete",
    )):
        return "target_zero_objects"
    if "multiple" in value or "ambiguous" in value:
        return "target_multiple_objects"
    if "stamp" in value:
        return "exact_stamp_mismatch"
    if "camera_info" in value:
        return "camera_info_invalid"
    if "depth" in value:
        return "depth_invalid"
    if "feature" in value or "axis" in value:
        return "pen_feature_invalid"
    if any(token in value for token in (
        "projector", "pnp", "reprojection", "touch_plane", "ray_",
        "workspace", "convex_hull",
    )):
        return "pnp_rejected"
    if "runtime_camera" in value:
        return "runtime_camera_exit"
    if "runtime_cuda" in value:
        return "runtime_cuda_exit"
    if "runtime_yolo" in value or "runtime_detector" in value:
        return "runtime_yolo_exit"
    return "unknown_target_failure"


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


def validate_showcase_site(profile: Mapping[str, Any]) -> None:
    """Require the fixed marker to retain every measured 650 mm venue truth."""
    if profile.get("schema") != "competition_venue_profile/v1":
        raise ValueError("showcase site schema mismatch")
    scalar_truth = {
        "table_height_m": 0.650,
        "reference_table_height_m": 0.560,
        "reference_plane_distance_m": 0.559428925,
        "expected_plane_distance_m": 0.469428925,
        "touch_plane_z_m": SHOWCASE_CONTACT_Z_M,
    }
    for field, expected in scalar_truth.items():
        try:
            actual = float(profile[field])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"showcase site {field} missing or invalid") from exc
        if not math.isclose(actual, expected, abs_tol=1e-12):
            raise ValueError(f"showcase site {field} must equal {expected}")
    if list(profile.get("orientation_deg", [])) != list(SHOWCASE_ORIENTATION_DEG):
        raise ValueError("showcase site orientation_deg mismatch")
    fixed_xy = profile.get("fallbacks", {}).get("fixed_xy", {}).get("xy_m")
    if fixed_xy != [0.4, 0.01]:
        raise ValueError("showcase site fixed_xy marker mismatch")
    if not math.isclose(
        float(profile["reference_plane_distance_m"])
        - (float(profile["table_height_m"]) - float(profile["reference_table_height_m"])),
        float(profile["expected_plane_distance_m"]),
        abs_tol=1e-12,
    ):
        raise ValueError("showcase site plane distance direction mismatch")


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
        and result.get("commands_emitted") is True
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


def decide_pick_attempt(
    result: dict[str, Any], *, attempt_number: int, showcase_enabled: bool,
    max_attempts: int = 2,
) -> dict[str, Any]:
    """Choose verified continuation, one fresh-snapshot retry, showcase, or stop.

    A retry is only possible when the completed hardware motion would otherwise
    qualify for truthful showcase continuation.  Hardware and motion failures
    therefore remain hard stops.  The last allowed attempt never requests a
    third grasp.
    """
    if max_attempts != 2:
        raise ValueError("competition pick max_attempts must be exactly 2")
    if attempt_number not in (1, 2):
        raise ValueError("attempt_number must be 1 or 2")
    decision = decide_pick_continuation(result, showcase_enabled=showcase_enabled)
    retry_eligible = (
        attempt_number == 1
        and result.get("verification_failure_class") == "object_absent"
        and result.get("motion_completed") is True
        and result.get("transport_pose_reached") is True
        and result.get("hardware_ok") is True
        and result.get("commands_emitted") is True
        and result.get("object_grasp_verified") is not True
    )
    if retry_eligible:
        return {
            **decision,
            "action": "retry_snapshot",
            "competition_success": False,
            "object_grasp_verified": False,
            "retry_requires_newer_stamp": True,
            "next_attempt_number": 2,
        }
    return decision


def validate_retry_snapshot(
    first_target: Mapping[str, Any], retry_target: Mapping[str, Any]
) -> tuple[bool, str]:
    """Require the second observation to be a new live target, not a replay."""
    if first_target.get("schema") != "competition_pick_target/v1":
        return False, "first_target_schema_mismatch"
    if retry_target.get("schema") != "competition_pick_target/v1":
        return False, "retry_target_schema_mismatch"
    if retry_target.get("valid") is not True:
        return False, "retry_target_invalid"
    if retry_target.get("selection_source") == "fixed_xy_fallback":
        return False, "retry_must_use_live_reidentification"
    try:
        first_stamp = int(first_target.get("stamp_ns", 0))
        retry_stamp = int(retry_target.get("stamp_ns", 0))
    except (TypeError, ValueError):
        return False, "retry_target_stamp_invalid"
    if first_stamp <= 0 or retry_stamp <= first_stamp:
        return False, "retry_snapshot_not_newer"
    return True, "ok"


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
