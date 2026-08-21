"""Fail-closed conversion of optical visual candidates into TF2 requests."""

from __future__ import annotations

from typing import Any

from .coordinate_chain_contract import trusted_for_execution, validate_request


def build_coordinate_chain_requests(candidate_payload: Any, *, target_frame: str, extrinsics_status: Any) -> dict[str, Any]:
    """Return requests only when physical hand-eye publication is explicitly live.

    Vision candidates remain camera-only by design.  Their geometric ``valid``
    flag is required, while physical permission comes solely from the status
    published by ``validated_extrinsics_tf``.
    """
    trusted, reason = trusted_for_execution(extrinsics_status)
    if not trusted:
        return {"state": "rejected", "reason": reason, "requests": [], "published": False}
    if not isinstance(candidate_payload, dict):
        return {"state": "rejected", "reason": "candidate_payload_invalid", "requests": [], "published": False}
    if not str(target_frame).strip():
        return {"state": "rejected", "reason": "target_frame_not_configured", "requests": [], "published": False}
    candidates = candidate_payload.get("candidates")
    if candidate_payload.get("valid") is not True or not isinstance(candidates, list) or not candidates:
        return {"state": "rejected", "reason": "camera_candidate_not_valid", "requests": [], "published": False}
    requests: list[dict[str, Any]] = []
    for candidate in candidates:
        if not isinstance(candidate, dict) or candidate.get("valid") is not True:
            return {"state": "rejected", "reason": "camera_candidate_not_valid", "requests": [], "published": False}
        point = candidate.get("coordinate_chain_point")
        if not isinstance(point, dict):
            return {"state": "rejected", "reason": "coordinate_chain_point_missing", "requests": [], "published": False}
        candidate_id = str(candidate.get("target_id") or candidate.get("id") or "")
        stamp_ns = int(point.get("stamp_ns", 0) or 0)
        has_geometry = all(candidate.get(name) is not None for name in (
            "grasp_point_camera_optical_m", "axis_camera_optical_unit", "approach_normal_camera_optical_unit"))
        geometry = ({
            "kind": "grasp_geometry", "source_frame": point.get("source_frame"),
            "stamp_ns": stamp_ns, "position_m": candidate.get("grasp_point_camera_optical_m"),
            "axis_unit": candidate.get("axis_camera_optical_unit"),
            "approach_normal_unit": candidate.get("approach_normal_camera_optical_unit"),
            "candidate_id": candidate_id, "transaction_id": str(candidate_payload.get("transaction_id") or f"pick-{stamp_ns}"),
            "quality": candidate.get("quality") if isinstance(candidate.get("quality"), dict) else {},
        } if has_geometry else dict(point))
        try:
            request = validate_request({**geometry, "target_frame": str(target_frame)})
        except ValueError as exc:
            return {"state": "rejected", "reason": str(exc), "requests": [], "published": False}
        request["candidate_id"] = candidate_id
        if not request["candidate_id"]:
            return {"state": "rejected", "reason": "camera_candidate_id_missing", "requests": [], "published": False}
        requests.append(request)
    return {"state": "ready", "reason": "ok", "requests": requests, "published": bool(requests)}
