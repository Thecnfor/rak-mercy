"""Fail-closed, ROS-independent rules for camera-to-tool coordinate requests.

Both simulation and hardware use TF2 for the actual transform.  This module
only defines the request/status envelope so no caller can silently substitute
a hand-entered matrix or an Isaac scene transform for physical hand-eye data.
"""

from __future__ import annotations

from math import isfinite, sqrt
from typing import Any

import numpy as np


CAMERA_FRAME = "left_camera_optical_frame"
BASE_FRAME = "base_link"


def _finite_vector(value: Any, name: str, length: int) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{name}_must_have_{length}_finite_values")
    try:
        result = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name}_must_have_{length}_finite_values") from exc
    if not all(isfinite(item) for item in result):
        raise ValueError(f"{name}_must_have_{length}_finite_values")
    return result


def normalize_quaternion(values: Any) -> list[float]:
    quaternion = _finite_vector(values, "quaternion_xyzw", 4)
    magnitude = sqrt(sum(value * value for value in quaternion))
    if magnitude < 1e-9:
        raise ValueError("quaternion_xyzw_zero_norm")
    return [value / magnitude for value in quaternion]


def validate_request(payload: Any) -> dict[str, Any]:
    """Validate a point or pose request without accepting an embedded matrix."""
    if not isinstance(payload, dict):
        raise ValueError("coordinate_request_must_be_object")
    kind = str(payload.get("kind") or "")
    if kind not in ("point", "pose", "grasp_geometry"):
        raise ValueError("kind_must_be_point_pose_or_grasp_geometry")
    source_frame = str(payload.get("source_frame") or "")
    target_frame = str(payload.get("target_frame") or "")
    if source_frame != CAMERA_FRAME:
        raise ValueError("source_frame_must_be_left_camera_optical_frame")
    if not target_frame or target_frame == CAMERA_FRAME:
        raise ValueError("target_frame_invalid")
    result = {
        "kind": kind, "source_frame": source_frame, "target_frame": target_frame,
        "stamp_ns": int(payload.get("stamp_ns", 0) or 0),
        "position_m": _finite_vector(payload.get("position_m"), "position_m", 3),
    }
    if result["stamp_ns"] < 0:
        raise ValueError("stamp_ns_invalid")
    has_mission, has_epoch = "mission_id" in payload, "nav_epoch" in payload
    if has_mission != has_epoch:
        raise ValueError("navigation_identity_incomplete")
    if has_mission:
        mission_id = payload.get("mission_id")
        try:
            nav_epoch = int(payload.get("nav_epoch"))
        except (TypeError, ValueError) as exc:
            raise ValueError("navigation_identity_invalid") from exc
        if isinstance(payload.get("nav_epoch"), bool) or not isinstance(mission_id, str) or not mission_id.strip() or nav_epoch <= 0:
            raise ValueError("navigation_identity_invalid")
        result.update({"mission_id": mission_id.strip(), "nav_epoch": nav_epoch})
    if kind == "pose":
        result["quaternion_xyzw"] = normalize_quaternion(payload.get("quaternion_xyzw"))
    if kind == "grasp_geometry":
        result["axis_unit"] = _unit_vector(payload.get("axis_unit"), "axis_unit")
        result["approach_normal_unit"] = _unit_vector(payload.get("approach_normal_unit"), "approach_normal_unit")
        result["candidate_id"] = str(payload.get("candidate_id") or "")
        result["transaction_id"] = str(payload.get("transaction_id") or f"pick-{result['stamp_ns']}")
        if not result["candidate_id"]:
            raise ValueError("candidate_id_missing")
        quality = payload.get("quality")
        result["quality"] = dict(quality) if isinstance(quality, dict) else {}
    return result


def _unit_vector(value: Any, name: str) -> list[float]:
    vector = np.asarray(_finite_vector(value, name, 3), dtype=float)
    magnitude = float(np.linalg.norm(vector))
    if magnitude < 1e-9:
        raise ValueError(f"{name}_zero_norm")
    return (vector / magnitude).tolist()


def trusted_for_execution(status: Any) -> tuple[bool, str]:
    """Only physical validated TF may turn a camera request into an executable pose."""
    if not isinstance(status, dict):
        return False, "extrinsics_status_missing"
    if status.get("trusted_for_grasp") is not True:
        return False, "extrinsics_not_trusted_for_grasp"
    if status.get("physical_validated") is not True:
        return False, "extrinsics_not_physically_validated"
    if status.get("tf_published") is not True:
        return False, "validated_extrinsics_tf_not_published"
    return True, "ok"


def quaternion_multiply(first: list[float], second: list[float]) -> list[float]:
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return normalize_quaternion([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def transform_request(request: dict[str, Any], rotation: np.ndarray, translation: np.ndarray, *, tf_quaternion_xyzw: list[float]) -> dict[str, Any]:
    """Apply a TF2-equivalent rigid transform after caller has passed the gate."""
    checked = validate_request(request)
    rotation = np.asarray(rotation, dtype=float)
    translation = np.asarray(translation, dtype=float).reshape(-1)
    if rotation.shape != (3, 3) or translation.shape != (3,) or not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation)):
        raise ValueError("tf_transform_invalid")
    output = {**checked, "position_m": (rotation @ np.asarray(checked["position_m"]) + translation).tolist(), "transform_interface": "tf2"}
    output["frame_id"] = checked["target_frame"]
    if checked["kind"] == "pose":
        output["quaternion_xyzw"] = quaternion_multiply(normalize_quaternion(tf_quaternion_xyzw), checked["quaternion_xyzw"])
    if checked["kind"] == "grasp_geometry":
        output.update({
            "grasp_point_base_m": output["position_m"],
            "axis_base_unit": _unit_vector((rotation @ np.asarray(checked["axis_unit"])).tolist(), "axis_base_unit"),
            "approach_normal_base_unit": _unit_vector((rotation @ np.asarray(checked["approach_normal_unit"])).tolist(), "approach_normal_base_unit"),
            "quality": checked["quality"],
        })
    return output
