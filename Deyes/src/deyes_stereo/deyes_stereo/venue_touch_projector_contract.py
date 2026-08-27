"""ROS-free fixed-plane projector for venue touch calibration.

The frame named here is the Mercury right-arm SDK Cartesian frame.  It is not
``base_link`` and this module neither publishes nor consumes TF.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np


SCHEMA = "venue_touch_projector/v1"
MATRIX_DIRECTION = "camera_from_right_arm_sdk"


@dataclass(frozen=True)
class ProjectionResult:
    usable: bool
    point_right_arm_sdk_m: np.ndarray | None
    camera_depth_m: float | None
    reasons: tuple[str, ...]


def _matrix4(value: Any) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError("transform_must_be_finite_4x4")
    return matrix


def validate_camera_from_right_arm_sdk(matrix: Any) -> tuple[str, ...]:
    """Validate a rigid transform with the declared direction only."""
    reasons: list[str] = []
    try:
        transform = _matrix4(matrix)
    except (TypeError, ValueError):
        return ("camera_from_right_arm_sdk_invalid_shape",)
    rotation = transform[:3, :3]
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        reasons.append("homogeneous_last_row_invalid")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
        reasons.append("rotation_not_orthonormal")
    if abs(float(np.linalg.det(rotation)) - 1.0) > 1e-5:
        reasons.append("rotation_determinant_not_positive_one")
    return tuple(reasons)


def invert_rigid(matrix: Any) -> np.ndarray:
    transform = _matrix4(matrix)
    reasons = validate_camera_from_right_arm_sdk(transform)
    if reasons:
        raise ValueError(",".join(reasons))
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = transform[:3, :3].T
    inverse[:3, 3] = -transform[:3, :3].T @ transform[:3, 3]
    return inverse


def point_in_convex_polygon(point_xy: Sequence[float], polygon_xy: Any, tolerance_m: float = 1e-9) -> bool:
    """Return whether a point is in/on a convex polygon of either winding."""
    point = np.asarray(point_xy, dtype=np.float64)
    polygon = np.asarray(polygon_xy, dtype=np.float64)
    if point.shape != (2,) or polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3:
        return False
    edges = np.roll(polygon, -1, axis=0) - polygon
    offsets = point - polygon
    crosses = edges[:, 0] * offsets[:, 1] - edges[:, 1] * offsets[:, 0]
    return bool(np.all(crosses >= -tolerance_m) or np.all(crosses <= tolerance_m))


def _intrinsics(projection: Any) -> np.ndarray:
    p = np.asarray(projection, dtype=np.float64)
    if p.shape == (3, 4):
        if abs(float(p[0, 3])) > 1e-9 or abs(float(p[1, 3])) > 1e-9:
            raise ValueError("left_rectified_projection_translation_must_be_zero")
        k = p[:, :3]
    elif p.shape == (3, 3):
        k = p
    else:
        raise ValueError("projection_must_be_3x3_or_3x4")
    if not np.all(np.isfinite(k)) or k[0, 0] <= 0.0 or k[1, 1] <= 0.0:
        raise ValueError("projection_intrinsics_invalid")
    return k


def project_pixel_to_fixed_z(
    pixel_uv: Sequence[float],
    projection: Any,
    camera_from_right_arm_sdk: Any,
    fixed_z_right_arm_sdk_m: float,
    *,
    matrix_direction: str = MATRIX_DIRECTION,
    workspace_xyz_m: Sequence[Sequence[float]] | None = None,
    calibration_hull_xy_m: Any | None = None,
    require_inside_calibration_hull: bool = True,
) -> ProjectionResult:
    """Intersect a rectified-camera ray with a fixed right-arm SDK Z plane."""
    reasons: list[str] = []
    if matrix_direction != MATRIX_DIRECTION:
        return ProjectionResult(False, None, None, ("matrix_direction_must_be_camera_from_right_arm_sdk",))
    try:
        transform = _matrix4(camera_from_right_arm_sdk)
        k = _intrinsics(projection)
    except (TypeError, ValueError) as exc:
        return ProjectionResult(False, None, None, (str(exc),))
    reasons.extend(validate_camera_from_right_arm_sdk(transform))
    pixel = np.asarray(pixel_uv, dtype=np.float64)
    if pixel.shape != (2,) or not np.all(np.isfinite(pixel)):
        reasons.append("pixel_uv_invalid")
    if not np.isfinite(fixed_z_right_arm_sdk_m) or abs(float(fixed_z_right_arm_sdk_m)) > 2.0:
        reasons.append("fixed_z_m_unit_or_range_invalid")
    if reasons:
        return ProjectionResult(False, None, None, tuple(reasons))

    right_from_camera = invert_rigid(transform)
    origin = right_from_camera[:3, 3]
    ray_camera = np.linalg.solve(k, np.array([pixel[0], pixel[1], 1.0]))
    ray_right = right_from_camera[:3, :3] @ ray_camera
    denominator = float(ray_right[2])
    if abs(denominator) < 1e-9:
        return ProjectionResult(False, None, None, ("camera_ray_parallel_to_fixed_z_plane",))
    scale = (float(fixed_z_right_arm_sdk_m) - float(origin[2])) / denominator
    if scale <= 0.0:
        return ProjectionResult(False, None, None, ("fixed_z_intersection_behind_camera",))
    point = origin + scale * ray_right
    camera_point = transform[:3, :3] @ point + transform[:3, 3]
    depth = float(camera_point[2])
    if depth <= 0.0:
        reasons.append("intersection_not_positive_camera_depth")

    if workspace_xyz_m is not None:
        bounds = np.asarray(workspace_xyz_m, dtype=np.float64)
        if bounds.shape != (3, 2) or not np.all(np.isfinite(bounds)) or np.any(bounds[:, 0] > bounds[:, 1]):
            reasons.append("workspace_bounds_invalid")
        elif np.any(point < bounds[:, 0] - 1e-9) or np.any(point > bounds[:, 1] + 1e-9):
            reasons.append("intersection_outside_workspace")
    if require_inside_calibration_hull:
        if calibration_hull_xy_m is None:
            reasons.append("calibration_hull_missing")
        elif not point_in_convex_polygon(point[:2], calibration_hull_xy_m):
            reasons.append("intersection_outside_calibration_hull")
    return ProjectionResult(not reasons, point, depth, tuple(reasons))


def validate_evidence_document(document: Any) -> tuple[str, ...]:
    """Validate the safety-critical, ROS-free evidence envelope."""
    if not isinstance(document, dict):
        return ("document_must_be_mapping",)
    reasons: list[str] = []
    if document.get("schema") != SCHEMA:
        reasons.append("schema_invalid")
    if document.get("matrix_direction") != MATRIX_DIRECTION:
        reasons.append("matrix_direction_invalid")
    if document.get("coordinate_frame") != "right_arm_sdk":
        reasons.append("coordinate_frame_must_be_right_arm_sdk")
    if document.get("publishes_tf") is not False:
        reasons.append("publishes_tf_must_be_false")
    if document.get("is_base_link_hand_eye") is not False:
        reasons.append("must_not_claim_base_link_hand_eye")
    reasons.extend(validate_camera_from_right_arm_sdk(document.get(MATRIX_DIRECTION, {}).get("matrix", [])))
    inverse = document.get("right_arm_sdk_from_camera", {}).get("matrix", [])
    try:
        forward = _matrix4(document.get(MATRIX_DIRECTION, {}).get("matrix", []))
        backward = _matrix4(inverse)
        if not np.allclose(forward @ backward, np.eye(4), atol=1e-6):
            reasons.append("forward_inverse_matrix_mismatch")
    except (TypeError, ValueError):
        reasons.append("right_arm_sdk_from_camera_invalid")
    metrics = document.get("metrics", {})
    for key in ("reprojection_rms_px", "reprojection_p95_px", "loo_base_xy_rms_mm", "loo_base_xy_p95_mm"):
        if not isinstance(metrics.get(key), (int, float)) or not np.isfinite(metrics[key]):
            reasons.append(f"metric_{key}_invalid")
    return tuple(dict.fromkeys(reasons))
