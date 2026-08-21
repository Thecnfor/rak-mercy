"""Replayable, fail-closed camera-optical pen grasp-candidate contract.

This is deliberately the boundary before hand-eye calibration.  Isaac and a
physical camera submit the same payload shape and receive only coordinates in
the optical frame of the depth image; a consumer must not treat this result as
an execution permission.
"""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from .ground_plane_contract import rectified_intrinsics, validate_dynamic_plane_for_depth, validate_rectified_depth_pair
from .pen_grasp_contract import build_pen_candidates, feature_matches_depth_stamp, pen_feature_stamp_ns


CAMERA_CANDIDATE_SCHEMA = "vision_grasp_candidates/camera_optical/v1"
CAMERA_OPTICAL_FRAME = "left_camera_optical_frame"
COORDINATE_TEMPLATE_SCHEMA = "coordinate_chain_requests/camera_optical/v1"


def _stamp_parts(stamp_ns: int) -> dict[str, int]:
    return {"stamp_sec": stamp_ns // 1_000_000_000, "stamp_nanosec": stamp_ns % 1_000_000_000}


def _rejected(reason: str, *, stamp_ns: int, source: str, camera_frame: str = "") -> dict[str, Any]:
    return {
        "schema": CAMERA_CANDIDATE_SCHEMA, **_stamp_parts(stamp_ns), "source": source,
        "camera_optical_frame": camera_frame, "valid": False, "reason": reason,
        "candidate_count": 0, "candidates": [], "trusted_for_grasp": False,
        "physical_execution_eligible": False,
    }


def build_camera_optical_pen_candidates(
    feature_payload: Mapping[str, Any], depth_m: np.ndarray, *, depth_stamp_ns: int,
    depth_frame_id: str, depth_width: int, depth_height: int, depth_encoding: str,
    camera_stamp_ns: int, camera_frame_id: str, camera_width: int, camera_height: int,
    projection: tuple[float, ...], plane_payload: Mapping[str, Any], source: str,
    now_stamp_ns: int | None = None, max_candidate_age_ns: int = 500_000_000,
    min_depth_m: float = .20, max_depth_m: float = 2.50,
    min_plane_clearance_m: float = .004, edge_margin_px: int = 12,
) -> dict[str, Any]:
    """Create non-executable optical-frame candidates from one complete frame.

    All joins are equality joins.  ``now_stamp_ns`` is optional for offline
    deterministic replay; when supplied, future and expired frames reject.
    """
    stamp = int(depth_stamp_ns)
    if not isinstance(source, str) or not source:
        return _rejected("source_missing", stamp_ns=stamp, source="")
    if max_candidate_age_ns < 0:
        return _rejected("max_candidate_age_invalid", stamp_ns=stamp, source=source, camera_frame=depth_frame_id)
    if now_stamp_ns is not None and (int(now_stamp_ns) < stamp or int(now_stamp_ns) - stamp > max_candidate_age_ns):
        return _rejected("target_expired", stamp_ns=stamp, source=source, camera_frame=depth_frame_id)
    if depth_encoding != "32FC1":
        return _rejected("depth_must_be_32FC1", stamp_ns=stamp, source=source, camera_frame=depth_frame_id)
    # Keep the camera name stable for the TF2 coordinate-chain contract.  A
    # simulator must remap its optical image into this same named frame; frame
    # aliases are not silently accepted.
    if depth_frame_id != CAMERA_OPTICAL_FRAME:
        return _rejected("camera_optical_frame_mismatch", stamp_ns=stamp, source=source, camera_frame=depth_frame_id)
    if not isinstance(depth_m, np.ndarray) or depth_m.shape[:2] != (depth_height, depth_width):
        return _rejected("depth_array_size_mismatch", stamp_ns=stamp, source=source, camera_frame=depth_frame_id)
    if pen_feature_stamp_ns(dict(feature_payload)) != stamp:
        return _rejected("pen_features_depth_stamp_mismatch", stamp_ns=stamp, source=source, camera_frame=depth_frame_id)
    if str(feature_payload.get("frame_id") or feature_payload.get("source_frame") or "") != depth_frame_id:
        return _rejected("pen_features_depth_frame_mismatch", stamp_ns=stamp, source=source, camera_frame=depth_frame_id)
    try:
        same_size = int(feature_payload.get("image_width", 0)) == depth_width and int(feature_payload.get("image_height", 0)) == depth_height
    except (TypeError, ValueError):
        same_size = False
    if not same_size:
        return _rejected("pen_features_depth_size_mismatch", stamp_ns=stamp, source=source, camera_frame=depth_frame_id)
    camera_contract = validate_rectified_depth_pair(
        depth_stamp_ns=stamp, depth_frame_id=depth_frame_id, depth_width=depth_width, depth_height=depth_height,
        depth_encoding=depth_encoding, info_stamp_ns=int(camera_stamp_ns), info_frame_id=camera_frame_id,
        info_width=int(camera_width), info_height=int(camera_height), projection=projection,
    )
    if not camera_contract.valid:
        return _rejected("depth_camera_info_contract:" + ",".join(camera_contract.reasons), stamp_ns=stamp, source=source, camera_frame=depth_frame_id)
    plane_contract = validate_dynamic_plane_for_depth(dict(plane_payload), depth_stamp_ns=stamp, depth_frame_id=depth_frame_id)
    if not plane_contract.valid:
        return _rejected("table_plane_depth_contract:" + ",".join(plane_contract.reasons), stamp_ns=stamp, source=source, camera_frame=depth_frame_id)
    try:
        normal = np.asarray(plane_payload["plane_normal"], dtype=np.float64).reshape(3)
        normal /= np.linalg.norm(normal)
        geometry = build_pen_candidates(
            dict(feature_payload), depth_m, rectified_intrinsics(projection), plane_payload=dict(plane_payload),
            rotation=None, translation=None, trusted_for_grasp=False, min_depth_m=min_depth_m,
            max_depth_m=max_depth_m, min_plane_clearance_m=min_plane_clearance_m, edge_margin_px=edge_margin_px,
        )
    except (KeyError, TypeError, ValueError, np.linalg.LinAlgError) as exc:
        return _rejected(str(exc), stamp_ns=stamp, source=source, camera_frame=depth_frame_id)
    candidates: list[dict[str, Any]] = []
    for raw in geometry["candidates"]:
        # A geometrically sound result is intentionally marked valid while its
        # trust scope remains camera-only.  No base-frame fields are copied.
        usable = raw.get("reason") == "untrusted_extrinsics" and isinstance(raw.get("center_camera_m"), list)
        item = {key: value for key, value in raw.items() if not key.endswith("_base_m") and not key.endswith("_base_unit")}
        item.update({
            "valid": usable, "trusted_for_grasp": False,
            "reason": "ok" if usable else str(raw.get("reason", "candidate_invalid")),
            "target_frame": depth_frame_id,
            "grasp_point_camera_optical_m": raw.get("center_camera_m"),
            "axis_camera_optical_unit": raw.get("axis_camera_unit"),
            "approach_normal_camera_optical_unit": [round(float(value), 5) for value in normal],
            # The TF2 coordinate-chain owner supplies its reviewed target tool
            # frame before calling validate_request; no transform is embedded.
            "coordinate_chain_point": {
                "kind": "point", "source_frame": CAMERA_OPTICAL_FRAME,
                "stamp_ns": stamp, "position_m": raw.get("center_camera_m"),
            },
        })
        candidates.append(item)
    valid = bool(candidates) and all(item["valid"] for item in candidates)
    return {
        "schema": CAMERA_CANDIDATE_SCHEMA, **_stamp_parts(stamp), "source": source,
        "transaction_id": f"pick-{stamp}",
        "camera_optical_frame": depth_frame_id, "valid": valid,
        "reason": "ok" if valid else "candidate_invalid", "candidate_count": len(candidates),
        "candidates": candidates, "trusted_for_grasp": False, "physical_execution_eligible": False,
    }


def coordinate_chain_templates(result: Mapping[str, Any]) -> dict[str, Any]:
    """Extract only valid non-executable point templates for the TF2 owner."""
    candidates = result.get("candidates") if isinstance(result.get("candidates"), list) else []
    requests = [
        {"target_id": item.get("target_id"), "candidate_valid": True, **dict(item["coordinate_chain_point"])}
        for item in candidates
        if isinstance(item, Mapping) and item.get("valid") is True and isinstance(item.get("coordinate_chain_point"), Mapping)
    ]
    return {
        "schema": COORDINATE_TEMPLATE_SCHEMA, "stamp_sec": int(result.get("stamp_sec", 0) or 0),
        "stamp_nanosec": int(result.get("stamp_nanosec", 0) or 0),
        "valid": bool(requests) and result.get("valid") is True,
        "reason": str(result.get("reason") or "candidate_invalid"),
        "physical_execution_eligible": False, "requests": requests,
    }
