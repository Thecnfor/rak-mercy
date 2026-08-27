"""ROS-free venue pick-target contract for the fixed 650 mm table.

The calibrated projector is deliberately injected.  A projector implements
``ray_for_pixel(u, v, camera_info)`` and returns either ``(origin, direction)``
or a mapping containing ``origin_m``, ``direction_unit`` and the required
``predicted_camera_z_m``.  This is compatible with the venue_touch_projector/v1
provider without importing its implementation here.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping, Sequence

import numpy as np

TARGET_SCHEMA = "competition_pick_target/v1"
TOUCH_Z_M = .135
TABLE_HEIGHT_M = .650
REFERENCE_TABLE_HEIGHT_M = .560


@dataclass(frozen=True)
class TargetPolicy:
    fixed_height_enabled: bool = True
    bbox_fallback_enabled: bool = False
    fixed_xy_fallback_enabled: bool = False
    fixed_xy_m: tuple[float, float] = (.400, .010)
    bbox_edge_margin_px: int = 12
    depth_min_m: float = .15
    depth_max_m: float = 2.0
    depth_agreement_m: float = .04
    table_height_tolerance_m: float = .025
    reference_plane_distance_m: float | None = None
    workspace_x_m: tuple[float, float] = (.25, .55)
    workspace_y_m: tuple[float, float] = (-.20, .20)


def _stamp(payload: Mapping[str, Any]) -> int:
    if "stamp_ns" in payload:
        return int(payload["stamp_ns"])
    return int(payload.get("stamp_sec", 0)) * 1_000_000_000 + int(payload.get("stamp_nanosec", 0))


def _reject(reason: str, stamp_ns: int = 0, **extra: Any) -> dict[str, Any]:
    return {"schema": TARGET_SCHEMA, "stamp_ns": stamp_ns, "valid": False,
            "trusted_for_venue_execution": False, "reason": reason,
            "commands_emitted": False, **extra}


def _point_in_convex_polygon(point: tuple[float, float], polygon: Sequence[Sequence[float]]) -> bool:
    if len(polygon) < 3:
        return False
    signs = []
    for a, b in zip(polygon, list(polygon[1:]) + [polygon[0]]):
        cross = (float(b[0])-float(a[0]))*(point[1]-float(a[1])) - (float(b[1])-float(a[1]))*(point[0]-float(a[0]))
        if abs(cross) > 1e-9:
            signs.append(cross > 0)
    return bool(signs) and (all(signs) or not any(signs))


def _plane_audit(plane: Mapping[str, Any] | None, policy: TargetPolicy) -> tuple[bool, str, float | None]:
    if (not isinstance(plane, Mapping) or plane.get("valid_for_table_removal") is not True
            or plane.get("degraded") is not False):
        return True, "fixed_height_unverified", None
    try:
        residual = float(plane.get("residual_rms_m"))
    except (TypeError, ValueError):
        return True, "fixed_height_unverified", None
    if not math.isfinite(residual) or residual > .01 or policy.reference_plane_distance_m is None:
        return True, "fixed_height_unverified", None
    try:
        distance = abs(float(plane.get("plane_distance_camera_m")))
    except (TypeError, ValueError):
        return True, "fixed_height_unverified", None
    if not math.isfinite(distance):
        return True, "fixed_height_unverified", None
    expected = abs(float(policy.reference_plane_distance_m)) + (TABLE_HEIGHT_M-REFERENCE_TABLE_HEIGHT_M)
    if abs(distance - expected) > policy.table_height_tolerance_m:
        return False, "table_height_deviation_exceeds_25mm", distance
    return True, "fixed_height_verified", distance


def _axis_midpoint(features: Mapping[str, Any]) -> tuple[tuple[float, float] | None, str | None]:
    items = features.get("features", [])
    if len(items) != 1:
        return None, None
    feature = items[0]
    endpoints = feature.get("axis_endpoints_px", feature.get("endpoints_px"))
    if feature.get("axis_complete") is not True or not isinstance(endpoints, Sequence) or len(endpoints) != 2:
        return None, None
    try:
        return ((float(endpoints[0][0]) + float(endpoints[1][0])) / 2.0,
                (float(endpoints[0][1]) + float(endpoints[1][1])) / 2.0), str(feature.get("target_id", "pen"))
    except (TypeError, ValueError, IndexError):
        return None, None


def _bbox_center(detection: Mapping[str, Any], width: int, height: int, margin: int) -> tuple[tuple[float, float] | None, str | None]:
    boxes = detection.get("detections", detection.get("boxes", []))
    if detection.get("complete") is False or len(boxes) != 1:
        return None, None
    box = boxes[0]
    raw = box.get("bbox_xyxy", box.get("xyxy"))
    try:
        x1, y1, x2, y2 = (float(v) for v in raw)
    except (TypeError, ValueError):
        return None, None
    if x1 < margin or y1 < margin or x2 > width-margin or y2 > height-margin or x2 <= x1 or y2 <= y1:
        return None, None
    return ((x1+x2)/2.0, (y1+y2)/2.0), str(box.get("target_id", "pen"))


def _ray(projector: Any, u: float, v: float, camera_info: Mapping[str, Any]) -> tuple[np.ndarray, np.ndarray, float | None]:
    method = getattr(projector, "ray_for_pixel", None) or getattr(projector, "pixel_to_ray", None)
    result = method(u, v, camera_info) if method else projector(u, v, camera_info)
    if isinstance(result, Mapping):
        origin = result.get("origin_m", result.get("origin"))
        direction = result.get("direction_unit", result.get("direction"))
        predicted = result.get("predicted_camera_z_m")
    else:
        origin, direction = result
        predicted = None
    o = np.asarray(origin, dtype=float).reshape(3); d = np.asarray(direction, dtype=float).reshape(3)
    if not np.all(np.isfinite(o)) or not np.all(np.isfinite(d)) or np.linalg.norm(d) < 1e-9:
        raise ValueError("projector_ray_invalid")
    return o, d / np.linalg.norm(d), None if predicted is None else float(predicted)


def build_competition_pick_target(*, detection: Mapping[str, Any], pen_features: Mapping[str, Any],
                                  depth_m: np.ndarray, camera_info: Mapping[str, Any],
                                  ground_plane: Mapping[str, Any] | None, projector: Any,
                                  touch_hull_xy_m: Sequence[Sequence[float]],
                                  policy: TargetPolicy = TargetPolicy()) -> dict[str, Any]:
    """Return one unclamped right_arm_sdk target at fixed Z=.135 m."""
    stamps = [_stamp(detection), _stamp(pen_features), _stamp(camera_info)]
    depth_stamp = int(camera_info.get("depth_stamp_ns", stamps[-1]))
    stamps.append(depth_stamp)
    if not stamps[0] or len(set(stamps)) != 1:
        return _reject("exact_stamp_mismatch", stamps[0] if stamps else 0)
    stamp_ns = stamps[0]
    if not policy.fixed_height_enabled:
        return _reject("fixed_height_policy_disabled", stamp_ns)
    if (isinstance(ground_plane, Mapping) and ground_plane.get("valid_for_table_removal") is True
            and _stamp(ground_plane) != stamp_ns):
        return _reject("ground_plane_exact_stamp_mismatch", stamp_ns)
    ok, height_state, measured_height = _plane_audit(ground_plane, policy)
    if not ok:
        return _reject(height_state, stamp_ns, measured_table_height_m=measured_height)
    shape = np.asarray(depth_m).shape
    if len(shape) != 2:
        return _reject("depth_image_invalid", stamp_ns)
    pixel, target_id = _axis_midpoint(pen_features)
    source = "axis_midpoint"
    if pixel is None and policy.bbox_fallback_enabled:
        pixel, target_id = _bbox_center(detection, shape[1], shape[0], policy.bbox_edge_margin_px)
        source = "bbox_center"
    if pixel is None and policy.fixed_xy_fallback_enabled:
        xy = policy.fixed_xy_m; source = "fixed_xy_fallback"; target_id = "fixed-xy"
        median_depth = predicted_depth = None
    elif pixel is None:
        return _reject("no_eligible_axis_or_bbox_target", stamp_ns, height_verification=height_state)
    else:
        u, v = pixel
        iu, iv = int(round(u)), int(round(v))
        crop = np.asarray(depth_m, dtype=float)[max(0,iv-2):iv+3, max(0,iu-2):iu+3]
        valid_depth = crop[np.isfinite(crop) & (crop >= policy.depth_min_m) & (crop <= policy.depth_max_m)]
        if valid_depth.size == 0:
            return _reject("target_depth_invalid", stamp_ns)
        median_depth = float(np.median(valid_depth))
        try:
            origin, direction, predicted_depth = _ray(projector, u, v, camera_info)
        except (TypeError, ValueError, AttributeError) as exc:
            return _reject(f"projector_rejected:{exc}", stamp_ns)
        if abs(direction[2]) < 1e-9:
            return _reject("ray_parallel_to_touch_plane", stamp_ns)
        distance = (TOUCH_Z_M-origin[2])/direction[2]
        if distance <= 0:
            return _reject("touch_plane_behind_ray", stamp_ns)
        point = origin + distance*direction
        xy = (float(point[0]), float(point[1]))
        if predicted_depth is None or not math.isfinite(predicted_depth):
            return _reject("projector_predicted_camera_z_missing", stamp_ns)
        if abs(predicted_depth-median_depth) > policy.depth_agreement_m:
            return _reject("projected_depth_disagrees_with_cuda_median", stamp_ns,
                           predicted_depth_m=predicted_depth, cuda_median_depth_m=median_depth)
    if not _point_in_convex_polygon(xy, touch_hull_xy_m):
        return _reject("target_outside_touch_convex_hull", stamp_ns, target_xy_m=list(xy))
    if not (policy.workspace_x_m[0] <= xy[0] <= policy.workspace_x_m[1] and
            policy.workspace_y_m[0] <= xy[1] <= policy.workspace_y_m[1]):
        return _reject("target_outside_workspace", stamp_ns, target_xy_m=list(xy))
    return {"schema": TARGET_SCHEMA, "stamp_ns": stamp_ns, "valid": True, "reason": "ok",
            "trusted_for_venue_execution": source != "fixed_xy_fallback", "commands_emitted": False,
            "target_id": target_id, "selection_source": source, "pixel_uv": None if pixel is None else list(pixel),
            "right_arm_sdk_target_m": [xy[0], xy[1], TOUCH_Z_M], "orientation_deg": [179.99, -12.0, 0.0],
            "cuda_median_depth_m": median_depth, "predicted_depth_m": predicted_depth,
            "height_verification": height_state, "measured_table_height_m": measured_height,
            "fixed_table_height_m": TABLE_HEIGHT_M, "reference_table_height_m": REFERENCE_TABLE_HEIGHT_M,
            "xy_clamped": False}
