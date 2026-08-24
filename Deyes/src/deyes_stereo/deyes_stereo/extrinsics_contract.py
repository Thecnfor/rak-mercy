"""Validated camera-to-robot extrinsics contract.

The stereo calibration describes geometry *inside* the camera pair.  It is not
evidence that a point in ``left_camera_optical_frame`` is correctly expressed
in ``base_link``.  This module keeps those two identities separate and is
intentionally ROS-free so that it can be tested off the robot.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml


REQUIRED_SOURCE_FRAME = "left_camera_optical_frame"
REQUIRED_TARGET_FRAME = "base_link"
REQUIRED_SOURCES = {"physical_point_correspondences", "physical_charuco_robot_world_handeye"}


@dataclass(frozen=True)
class ExtrinsicsValidation:
    valid: bool
    reasons: tuple[str, ...]
    calibration_id: str = ""
    rotation: np.ndarray | None = None
    translation: np.ndarray | None = None


def _vector(value: Any, name: str, size: int) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float64).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name}_must_be_numeric") from exc
    if result.size != size or not np.all(np.isfinite(result)):
        raise ValueError(f"{name}_must_have_{size}_finite_values")
    return result


def quaternion_to_matrix(values: Any) -> np.ndarray:
    x, y, z, w = _vector(values, "quaternion_xyzw", 4)
    norm = float(np.linalg.norm([x, y, z, w]))
    if norm < 1e-9:
        raise ValueError("quaternion_xyzw_zero_norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]],
        dtype=np.float64,
    )


def matrix_to_quaternion(matrix: np.ndarray) -> list[float]:
    r = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(r))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        x, y, z, w = (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s
    elif r[0, 0] >= r[1, 1] and r[0, 0] >= r[2, 2]:
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        x, y, z, w = 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s
    elif r[1, 1] >= r[2, 2]:
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        x, y, z, w = (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s
    else:
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        x, y, z, w = (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s, (r[1, 0] - r[0, 1]) / s
    return [float(x), float(y), float(z), float(w)]


def validate_extrinsics(
    document: dict[str, Any], *, stereo_document: dict[str, Any] | None = None,
    expected_robot_id: str = "", expected_camera_pair_id: str = "",
    max_rms_m: float = 0.005, max_p95_m: float = 0.010,
) -> ExtrinsicsValidation:
    """Validate a physical ``base_link_T_left_camera`` result for grasp use."""
    reasons: list[str] = []
    calibration_id = str(document.get("calibration_id") or "").strip()
    if not calibration_id:
        reasons.append("calibration_id_missing")
    if document.get("validated") is not True:
        reasons.append("extrinsics_not_validated")
    if document.get("operator_confirmation") is not True:
        reasons.append("operator_confirmation_missing")
    if document.get("source") not in REQUIRED_SOURCES:
        reasons.append("source_is_not_physical_point_correspondences")
    if document.get("source") == "physical_charuco_robot_world_handeye" and document.get("trusted_for_execution") is not True:
        reasons.append("charuco_handeye_not_trusted_for_execution")
    if str(document.get("source_frame") or "") != REQUIRED_SOURCE_FRAME:
        reasons.append("source_frame_must_be_left_camera_optical_frame")
    if str(document.get("target_frame") or "") != REQUIRED_TARGET_FRAME:
        reasons.append("target_frame_must_be_base_link")
    if not str(document.get("robot_id") or "").strip():
        reasons.append("robot_id_missing")
    if not str(document.get("camera_pair_id") or "").strip():
        reasons.append("camera_pair_id_missing")
    if expected_robot_id and document.get("robot_id") != expected_robot_id:
        reasons.append("robot_id_mismatch")
    if expected_camera_pair_id and document.get("camera_pair_id") != expected_camera_pair_id:
        reasons.append("camera_pair_id_mismatch")
    metrics = document.get("metrics") or {}
    count = int(metrics.get("correspondence_count", 0) or 0)
    rms = float(metrics.get("rms_m", float("inf")) or float("inf"))
    p95 = float(metrics.get("p95_m", float("inf")) or float("inf"))
    if count < 6:
        reasons.append("insufficient_correspondences")
    if not np.isfinite(rms) or rms > max_rms_m:
        reasons.append("rms_exceeds_limit")
    if not np.isfinite(p95) or p95 > max_p95_m:
        reasons.append("p95_exceeds_limit")
    if stereo_document is None:
        reasons.append("stereo_calibration_not_supplied")
    else:
        if stereo_document.get("validated") is not True:
            reasons.append("stereo_calibration_not_validated")
        stereo_id = str(stereo_document.get("calibration_id") or "")
        if not stereo_id or document.get("stereo_calibration_id") != stereo_id:
            reasons.append("stereo_calibration_identity_mismatch")
        for key in ("robot_id", "camera_pair_id"):
            if document.get(key) != stereo_document.get(key):
                reasons.append(f"stereo_{key}_mismatch")
    try:
        rotation = quaternion_to_matrix(document.get("quaternion_xyzw"))
        translation = _vector(document.get("translation_m"), "translation_m", 3)
    except ValueError as exc:
        reasons.append(str(exc))
        rotation, translation = None, None
    return ExtrinsicsValidation(not reasons, tuple(reasons), calibration_id, rotation, translation)


def load_yaml_document(path_text: str) -> dict[str, Any]:
    path = Path(path_text).expanduser()
    with path.open("r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}
    if not isinstance(document, dict):
        raise ValueError("yaml_root_must_be_a_mapping")
    return document


def solve_base_from_camera(cameras: np.ndarray, bases: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Solve ``base_point = R @ camera_point + t`` by Kabsch alignment."""
    cameras = np.asarray(cameras, dtype=np.float64)
    bases = np.asarray(bases, dtype=np.float64)
    if cameras.shape != bases.shape or cameras.ndim != 2 or cameras.shape[1] != 3:
        raise ValueError("correspondences_must_be_matching_nx3_arrays")
    if cameras.shape[0] < 6:
        raise ValueError("at_least_six_correspondences_required")
    if not np.all(np.isfinite(cameras)) or not np.all(np.isfinite(bases)):
        raise ValueError("correspondences_must_be_finite")
    centered = cameras - cameras.mean(axis=0)
    if np.linalg.matrix_rank(centered, tol=1e-6) < 2:
        raise ValueError("camera_correspondence_geometry_is_degenerate")
    if float(np.max(np.linalg.norm(centered[:, None] - centered[None, :], axis=2))) < 0.08:
        raise ValueError("camera_correspondence_span_below_0_08m")
    h = centered.T @ (bases - bases.mean(axis=0))
    u, _, vt = np.linalg.svd(h)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    translation = bases.mean(axis=0) - rotation @ cameras.mean(axis=0)
    residuals = np.linalg.norm((rotation @ cameras.T).T + translation - bases, axis=1)
    return rotation, translation, residuals
