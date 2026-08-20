"""ROS-free 2D pen-feature plus stereo-depth grasp-candidate contract."""

from __future__ import annotations

from typing import Any

import numpy as np


def pen_feature_stamp_ns(feature_payload: dict[str, Any]) -> int:
    """Return a pen-feature stamp, or zero when it is absent/malformed."""
    try:
        return int(feature_payload.get("stamp_sec", 0) or 0) * 1_000_000_000 + int(feature_payload.get("stamp_nanosec", 0) or 0)
    except (AttributeError, TypeError, ValueError):
        return 0


def feature_matches_depth_stamp(feature_payload: dict[str, Any], depth_stamp_ns: int) -> bool:
    """A 2D feature is usable only with the exact depth frame that produced it."""
    feature_stamp_ns = pen_feature_stamp_ns(feature_payload)
    return feature_stamp_ns > 0 and feature_stamp_ns == depth_stamp_ns


def _point(value: Any, name: str, size: int = 2) -> np.ndarray:
    point = np.asarray(value, dtype=np.float64).reshape(-1)
    if point.size != size or not np.all(np.isfinite(point)):
        raise ValueError(f"{name}_must_be_{size}_finite_numbers")
    return point


def _parse_one_pen_feature(feature: dict[str, Any]) -> dict[str, Any]:
    """Accept a segmentation producer's explicit pixels and long-axis endpoints.

    ``mask_pixels_px`` is deliberately explicit rather than a detector-specific
    binary encoding.  It makes this downstream module independent of YOLO and
    lets a segmentation or classical contour producer be substituted safely.
    """
    pixels = feature.get("mask_pixels_px")
    if not isinstance(pixels, list) or len(pixels) < 12:
        raise ValueError("mask_pixels_px_requires_at_least_12_pixels")
    mask = np.asarray([_point(item, "mask_pixel") for item in pixels], dtype=np.float64)
    endpoints = feature.get("axis_endpoints_px") or feature.get("long_axis_endpoints_px")
    if not isinstance(endpoints, list) or len(endpoints) != 2:
        raise ValueError("axis_endpoints_px_requires_two_points")
    first, last = _point(endpoints[0], "axis_endpoint"), _point(endpoints[1], "axis_endpoint")
    if float(np.linalg.norm(last - first)) < 8.0:
        raise ValueError("axis_length_below_8_pixels")
    return {"mask_pixels_px": mask, "axis_endpoints_px": np.stack([first, last]), "confidence": float(feature.get("confidence", 0.0) or 0.0), "id": str(feature.get("id") or "pen"), "axis_complete": bool(feature.get("axis_complete", False))}


def parse_pen_features(payload: dict[str, Any]) -> list[dict[str, Any]]:
    """Parse every distinct pen feature in a frame for batch candidate building."""
    features = payload.get("features") if isinstance(payload.get("features"), list) else [payload]
    candidates = [item for item in features if isinstance(item, dict) and str(item.get("label") or item.get("class_name") or "") == "pen"]
    if not candidates:
        raise ValueError("waiting/no_target")
    parsed = [_parse_one_pen_feature(item) for item in candidates]
    if len({item["id"] for item in parsed}) != len(parsed):
        raise ValueError("geometric_conflict_or_indistinguishable")
    return parsed


def parse_pen_feature(payload: dict[str, Any]) -> dict[str, Any]:
    """Compatibility helper for call sites that deliberately require one pen."""
    features = parse_pen_features(payload)
    if len(features) != 1:
        raise ValueError("multiple_pen_features_require_batch_interface")
    return features[0]


def _project(pixels: np.ndarray, depths: np.ndarray, intrinsics: tuple[float, float, float, float]) -> np.ndarray:
    fx, fy, cx, cy = intrinsics
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera_intrinsics_invalid")
    return np.column_stack(((pixels[:, 0] - cx) * depths / fx, (pixels[:, 1] - cy) * depths / fy, depths))


def _depth_at(depth: np.ndarray, pixel: np.ndarray, radius: int, minimum: float, maximum: float) -> tuple[float | None, float]:
    x, y = int(round(float(pixel[0]))), int(round(float(pixel[1])))
    x0, x1 = max(0, x - radius), min(depth.shape[1], x + radius + 1)
    y0, y1 = max(0, y - radius), min(depth.shape[0], y + radius + 1)
    values = depth[y0:y1, x0:x1].reshape(-1)
    values = values[np.isfinite(values) & (values >= minimum) & (values <= maximum)]
    if values.size == 0:
        return None, 0.0
    median = float(np.median(values))
    return median, float(values.size) / float((x1 - x0) * (y1 - y0))


def build_pen_candidate(
    feature_payload: dict[str, Any], depth: np.ndarray, intrinsics: tuple[float, float, float, float], *,
    plane_payload: dict[str, Any] | None, rotation: np.ndarray | None, translation: np.ndarray | None,
    trusted_for_grasp: bool, min_depth_m: float = .20, max_depth_m: float = 1.00,
    min_plane_clearance_m: float = .004, endpoint_radius_px: int = 2, edge_margin_px: int = 12,
) -> dict[str, Any]:
    """Build a candidate, withholding every base-frame point until trust passes."""
    feature = parse_pen_feature(feature_payload)
    pixels = feature["mask_pixels_px"]
    endpoints = feature["axis_endpoints_px"]
    near_edge = bool(
        np.any(pixels[:, 0] <= edge_margin_px) or np.any(pixels[:, 0] >= depth.shape[1] - 1 - edge_margin_px)
        or np.any(pixels[:, 1] <= edge_margin_px) or np.any(pixels[:, 1] >= depth.shape[0] - 1 - edge_margin_px)
        or np.any(endpoints[:, 0] <= edge_margin_px) or np.any(endpoints[:, 0] >= depth.shape[1] - 1 - edge_margin_px)
        or np.any(endpoints[:, 1] <= edge_margin_px) or np.any(endpoints[:, 1] >= depth.shape[0] - 1 - edge_margin_px)
    )
    if near_edge or not feature["axis_complete"]:
        return {"valid": False, "trusted_for_grasp": False, "target_id": feature["id"], "pen_id": feature["id"], "reason": "edge_truncation" if near_edge else "axis_completeness_unconfirmed", "target_visibility": "edge_truncated" if near_edge else "unknown_or_incomplete"}
    in_image = (pixels[:, 0] >= 0) & (pixels[:, 0] < depth.shape[1]) & (pixels[:, 1] >= 0) & (pixels[:, 1] < depth.shape[0])
    pixels = pixels[in_image]
    # Sampling every mask pixel creates excessive work for dense masks; preserve
    # spatial coverage deterministically.
    if len(pixels) > 800:
        pixels = pixels[np.linspace(0, len(pixels) - 1, 800, dtype=int)]
    samples: list[tuple[np.ndarray, float]] = []
    for pixel in pixels:
        z, _ = _depth_at(depth, pixel, 0, min_depth_m, max_depth_m)
        if z is not None:
            samples.append((pixel, z))
    if len(samples) < 12:
        return {"valid": False, "trusted_for_grasp": False, "reason": "insufficient_valid_depth_samples", "target_id": feature["id"], "pen_id": feature["id"]}
    sample_pixels = np.asarray([item[0] for item in samples])
    sample_depth = np.asarray([item[1] for item in samples])
    points_camera = _project(sample_pixels, sample_depth, intrinsics)
    if not isinstance(plane_payload, dict):
        return {"valid": False, "trusted_for_grasp": False, "reason": "table_plane_missing", "target_id": feature["id"], "pen_id": feature["id"]}
    if plane_payload.get("coordinate_contract") != "dynamic_table_plane_camera_relative_only" or plane_payload.get("valid_for_table_removal") is not True or plane_payload.get("degraded") is not False:
        return {"valid": False, "trusted_for_grasp": False, "reason": "table_plane_not_fresh_camera_relative_evidence", "target_id": feature["id"], "pen_id": feature["id"]}
    try:
        normal = _point(plane_payload["plane_normal"], "plane_normal", 3)
        center = _point(plane_payload["plane_center_camera_m"], "plane_center_camera_m", 3)
        normal /= np.linalg.norm(normal)
    except (KeyError, ValueError):
        return {"valid": False, "trusted_for_grasp": False, "reason": "table_plane_invalid", "target_id": feature["id"], "pen_id": feature["id"]}
    off_plane = np.abs((points_camera - center) @ normal) >= min_plane_clearance_m
    if int(np.count_nonzero(off_plane)) < 8:
        return {"valid": False, "trusted_for_grasp": False, "reason": "table_plane_removal_left_too_few_pen_samples", "target_id": feature["id"], "pen_id": feature["id"]}
    sample_depth = sample_depth[off_plane]
    points_camera = points_camera[off_plane]
    median = float(np.median(sample_depth))
    mad = float(np.median(np.abs(sample_depth - median)))
    robust = np.abs(sample_depth - median) <= max(.003, 3.0 * 1.4826 * mad)
    sample_depth, points_camera = sample_depth[robust], points_camera[robust]
    if len(points_camera) < 8:
        return {"valid": False, "trusted_for_grasp": False, "reason": "pen_depth_samples_not_robust", "target_id": feature["id"], "pen_id": feature["id"]}
    endpoint_points: list[np.ndarray] = []
    for endpoint in feature["axis_endpoints_px"]:
        z, _ = _depth_at(depth, endpoint, endpoint_radius_px, min_depth_m, max_depth_m)
        if z is None:
            return {"valid": False, "trusted_for_grasp": False, "reason": "endpoint_depth_missing", "target_id": feature["id"], "pen_id": feature["id"]}
        endpoint_points.append(_project(endpoint.reshape(1, 2), np.asarray([z]), intrinsics)[0])
    axis_camera = endpoint_points[1] - endpoint_points[0]
    axis_norm = float(np.linalg.norm(axis_camera))
    if axis_norm < .01:
        return {"valid": False, "trusted_for_grasp": False, "reason": "pen_axis_3d_too_short", "target_id": feature["id"], "pen_id": feature["id"]}
    center_camera = np.median(points_camera, axis=0)
    valid_ratio = float(len(points_camera)) / float(max(1, len(pixels)))
    confidence = float(np.clip(feature["confidence"], 0, 1) * min(1.0, valid_ratio / .25) * np.exp(-mad / .012))
    result: dict[str, Any] = {
        "valid": False, "trusted_for_grasp": False, "target_id": feature["id"], "pen_id": feature["id"],
        "reason": "untrusted_extrinsics" if not trusted_for_grasp else "ok",
        "target_visibility": "full",
        "confidence": round(confidence, 4), "depth_median_m": round(median, 4), "depth_mad_m": round(mad, 4),
        "mask_depth_valid_ratio": round(valid_ratio, 4), "table_removed_count": int(np.count_nonzero(~off_plane)),
        "quality": {"detection_confidence": round(float(feature["confidence"]), 4), "depth_mad_m": round(mad, 4), "mask_depth_valid_ratio": round(valid_ratio, 4), "table_removed_count": int(np.count_nonzero(~off_plane))},
        "center_camera_m": [round(float(v), 4) for v in center_camera],
        "axis_camera_unit": [round(float(v), 5) for v in axis_camera / axis_norm],
        "endpoints_camera_m": [[round(float(v), 4) for v in point] for point in endpoint_points],
        "grasp_interval_camera_m": [[round(float(v), 4) for v in endpoint_points[0] + .20 * axis_camera], [round(float(v), 4) for v in endpoint_points[1] - .20 * axis_camera]],
    }
    if not trusted_for_grasp or rotation is None or translation is None:
        return result
    base_center = rotation @ center_camera + translation
    base_endpoints = [rotation @ point + translation for point in endpoint_points]
    base_axis = base_endpoints[1] - base_endpoints[0]
    base_axis /= np.linalg.norm(base_axis)
    base_normal = rotation @ normal
    result.update({
        "valid": True, "trusted_for_grasp": True, "reason": "ok",
        "target_frame": "base_link", "grasp_point_base_m": [round(float(v), 4) for v in base_center],
        "axis_base_unit": [round(float(v), 5) for v in base_axis],
        "approach_normal_base_unit": [round(float(v), 5) for v in base_normal],
        "endpoints_base_m": [[round(float(v), 4) for v in point] for point in base_endpoints],
        "grasp_interval_base_m": [[round(float(v), 4) for v in base_endpoints[0] + .20 * (base_endpoints[1] - base_endpoints[0])], [round(float(v), 4) for v in base_endpoints[1] - .20 * (base_endpoints[1] - base_endpoints[0])]],
    })
    return result


def build_pen_candidates(
    payload: dict[str, Any], depth: np.ndarray, intrinsics: tuple[float, float, float, float], **kwargs: Any,
) -> dict[str, Any]:
    """Single-target wrapper; kept batch-shaped for a stable consumer schema."""
    features = parse_pen_features(payload)
    candidates = [
        build_pen_candidate(
            {"label": "pen", "id": item["id"], "confidence": item["confidence"], "axis_complete": item["axis_complete"],
             "mask_pixels_px": item["mask_pixels_px"].tolist(), "axis_endpoints_px": item["axis_endpoints_px"].tolist()},
            depth, intrinsics, **kwargs,
        )
        for item in features
    ]
    centers = [item.get("center_camera_m") for item in candidates]
    conflict = len(candidates) == 2 and all(isinstance(center, list) and len(center) == 3 for center in centers) and float(np.linalg.norm(np.asarray(centers[0]) - np.asarray(centers[1]))) < .03
    if conflict:
        for candidate in candidates:
            candidate.update({"valid": False, "trusted_for_grasp": False, "reason": "geometric_conflict_or_indistinguishable"})
            for key in ("grasp_point_base_m", "axis_base_unit", "approach_normal_base_unit", "endpoints_base_m", "grasp_interval_base_m"):
                candidate.pop(key, None)
    return {
        "valid": bool(candidates) and all(item["valid"] for item in candidates),
        "trusted_for_grasp": bool(candidates) and all(item["trusted_for_grasp"] for item in candidates),
        "reason": "geometric_conflict_or_indistinguishable" if conflict else ("ok" if candidates and all(item["valid"] for item in candidates) else "candidate_invalid"),
        "candidate_count": len(candidates), "candidates": candidates,
    }
