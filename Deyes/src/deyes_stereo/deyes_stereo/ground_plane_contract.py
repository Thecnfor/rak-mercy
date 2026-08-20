"""Pure contracts for rectified-depth table-plane estimation.

Dynamic table planes are camera-relative evidence only.  They must never stand
in for the independently validated ``base_link_T_left_camera`` extrinsics.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class FrameContract:
    valid: bool
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class PlaneFit:
    normal: np.ndarray
    center: np.ndarray
    inlier_mask: np.ndarray
    residuals_m: np.ndarray

    @property
    def inlier_count(self) -> int:
        return int(np.count_nonzero(self.inlier_mask))

    @property
    def inlier_ratio(self) -> float:
        return float(self.inlier_count) / float(len(self.inlier_mask)) if len(self.inlier_mask) else 0.0

    @property
    def residual_rms_m(self) -> float:
        values = self.residuals_m[self.inlier_mask]
        return float(np.sqrt(np.mean(values * values))) if len(values) else float("inf")

    @property
    def residual_p95_m(self) -> float:
        values = self.residuals_m[self.inlier_mask]
        return float(np.percentile(values, 95)) if len(values) else float("inf")


def validate_rectified_depth_pair(
    *, depth_stamp_ns: int, depth_frame_id: str, depth_width: int, depth_height: int,
    depth_encoding: str, info_stamp_ns: int, info_frame_id: str, info_width: int,
    info_height: int, projection: Any,
) -> FrameContract:
    reasons: list[str] = []
    if depth_encoding != "32FC1":
        reasons.append("depth_encoding_must_be_32FC1")
    if not depth_frame_id or not info_frame_id or depth_frame_id != info_frame_id:
        reasons.append("depth_camera_info_frame_mismatch")
    if depth_stamp_ns != info_stamp_ns:
        reasons.append("depth_camera_info_stamp_mismatch")
    if depth_width <= 0 or depth_height <= 0 or depth_width != info_width or depth_height != info_height:
        reasons.append("depth_camera_info_size_mismatch")
    try:
        p = np.asarray(projection, dtype=np.float64).reshape(-1)
    except (TypeError, ValueError):
        p = np.empty((0,), dtype=np.float64)
    if p.size != 12 or not np.all(np.isfinite(p)):
        reasons.append("rectified_projection_invalid")
    elif p[0] <= 0.0 or p[5] <= 0.0:
        reasons.append("rectified_projection_focal_length_invalid")
    elif abs(float(p[3])) > 1e-9 or abs(float(p[7])) > 1e-9:
        # This node consumes the rectified *left* projection only.
        reasons.append("rectified_left_projection_translation_must_be_zero")
    return FrameContract(not reasons, tuple(reasons))


def rectified_intrinsics(projection: Any) -> tuple[float, float, float, float]:
    """Return the only projection terms valid for left-rectified backprojection."""
    p = np.asarray(projection, dtype=np.float64).reshape(-1)
    if p.size != 12 or not np.all(np.isfinite(p)) or p[0] <= 0.0 or p[5] <= 0.0:
        raise ValueError("rectified_projection_invalid")
    return float(p[0]), float(p[5]), float(p[2]), float(p[6])


def validate_dynamic_plane_for_depth(plane: Any, *, depth_stamp_ns: int, depth_frame_id: str) -> FrameContract:
    """A dynamic plane is usable only for the exact rectified depth frame."""
    if not isinstance(plane, dict):
        return FrameContract(False, ("table_plane_missing",))
    reasons: list[str] = []
    plane_stamp = int(plane.get("stamp_sec", 0) or 0) * 1_000_000_000 + int(plane.get("stamp_nanosec", 0) or 0)
    if plane_stamp != depth_stamp_ns:
        reasons.append("table_plane_depth_stamp_mismatch")
    if str(plane.get("camera_frame") or "") != depth_frame_id:
        reasons.append("table_plane_depth_frame_mismatch")
    if plane.get("coordinate_contract") != "dynamic_table_plane_camera_relative_only":
        reasons.append("table_plane_coordinate_contract_invalid")
    if plane.get("valid_for_table_removal") is not True or plane.get("degraded") is not False:
        reasons.append("table_plane_not_fresh")
    return FrameContract(not reasons, tuple(reasons))


def project_rectified_depth_pixels(u: np.ndarray, v: np.ndarray, z: np.ndarray, projection: Any) -> np.ndarray:
    """Backproject with CameraInfo.P[0], P[5], P[2], P[6]; never raw K."""
    fx, fy, cx, cy = rectified_intrinsics(projection)
    return np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z)).astype(np.float64)


def fit_plane_ransac(points: np.ndarray, distance_threshold: float, iterations: int, seed: int | None = None) -> PlaneFit | None:
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3 or not np.all(np.isfinite(points)):
        return None
    rng = np.random.default_rng(seed)
    best_mask: np.ndarray | None = None
    best_normal: np.ndarray | None = None
    for _ in range(max(1, int(iterations))):
        p0, p1, p2 = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = float(np.linalg.norm(normal))
        if norm < 1e-9:
            continue
        normal /= norm
        mask = np.abs(points @ normal - float(p0 @ normal)) <= distance_threshold
        if best_mask is None or int(mask.sum()) > int(best_mask.sum()):
            best_mask, best_normal = mask, normal
    if best_mask is None or best_normal is None or int(best_mask.sum()) < 3:
        return None
    center = points[best_mask].mean(axis=0)
    _, _, vt = np.linalg.svd(points[best_mask] - center)
    normal = vt[-1]
    if float(normal @ best_normal) < 0.0:
        normal = -normal
    normal /= float(np.linalg.norm(normal))
    residuals = np.abs(points @ normal - float(center @ normal))
    mask = residuals <= distance_threshold
    if int(mask.sum()) < 3:
        return None
    center = points[mask].mean(axis=0)
    return PlaneFit(normal=normal, center=center, inlier_mask=mask, residuals_m=residuals)


def evaluate_plane(points: np.ndarray, normal: np.ndarray, center: np.ndarray, distance_threshold: float) -> PlaneFit:
    points = np.asarray(points, dtype=np.float64)
    normal = np.asarray(normal, dtype=np.float64)
    center = np.asarray(center, dtype=np.float64)
    normal /= float(np.linalg.norm(normal))
    residuals = np.abs(points @ normal - float(center @ normal))
    return PlaneFit(normal=normal, center=center, inlier_mask=residuals <= distance_threshold, residuals_m=residuals)


def normal_delta_deg(current: np.ndarray, previous: np.ndarray | None) -> float | None:
    if previous is None:
        return None
    dot = float(np.clip(np.asarray(current) @ np.asarray(previous), -1.0, 1.0))
    return float(np.degrees(np.arccos(abs(dot))))
