"""Fail-closed ChArUco robot-world/hand-eye evidence contract.

This module only validates saved observations and calls OpenCV's solver.  It has
no serial, ROS motion, or ``drag_teach_execute`` path.
"""
from __future__ import annotations
from dataclasses import dataclass
from itertools import combinations
import math
from typing import Any
import numpy as np
from .charuco_board_generator import require_charuco

MIN_SAMPLES, MAX_SAMPLES = 12, 16

@dataclass(frozen=True)
class HandeyeGate:
    ready: bool
    reasons: tuple[str, ...]

def _matrix(value: Any, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (4, 4) or not np.all(np.isfinite(result)) or not np.allclose(result[3], [0, 0, 0, 1]):
        raise ValueError(f"{name}_must_be_finite_4x4_transform")
    if not np.allclose(result[:3,:3].T @ result[:3,:3], np.eye(3), atol=1e-5):
        raise ValueError(f"{name}_rotation_not_orthonormal")
    return result

def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    value = np.clip((np.trace(a[:3,:3].T @ b[:3,:3]) - 1) / 2, -1, 1)
    return float(math.degrees(math.acos(value)))

def validate_session(payload: dict[str, Any]) -> HandeyeGate:
    reasons: list[str] = []
    if payload.get("drag_mode") != "manual_save_pause": reasons.append("manual_save_pause_contract_required")
    if payload.get("drag_teach_execute_called") is not False: reasons.append("drag_teach_execute_must_be_absent")
    samples = payload.get("samples") if isinstance(payload.get("samples"), list) else []
    if not MIN_SAMPLES <= len(samples) <= MAX_SAMPLES: reasons.append("sample_count_not_in_12_to_16")
    transforms: list[np.ndarray] = []
    for item in samples:
        if not isinstance(item, dict) or item.get("locked") is not True: reasons.append("unlocked_pose_sample"); continue
        if item.get("serial_busy") is True or item.get("feedback_fresh") is not True or item.get("captured_while_dragging") is True: reasons.append("unsafe_robot_sample")
        if abs(float(item.get("head_deviation_deg", float("inf")))) > .5: reasons.append("head_deviation_exceeds_0_5_deg")
        if int(item.get("common_charuco_ids", 0)) < 12: reasons.append("common_charuco_ids_below_12")
        if float(item.get("skew_ms", float("inf"))) > 10: reasons.append("image_skew_exceeds_10ms")
        try: transforms.append(_matrix(item.get("base_T_gripper"), "base_T_gripper"))
        except ValueError as exc: reasons.append(str(exc))
    if transforms:
        span = max(float(np.linalg.norm(a[:3,3]-b[:3,3])) for a,b in combinations(transforms,2))
        declared_axes = payload.get("rotation_axes_span_deg", {})
        axes = [float(declared_axes.get(axis, 0.0)) for axis in ("axis_1", "axis_2")]
        if span < .100: reasons.append("translation_span_below_100mm")
        if any(angle < 30 for angle in axes): reasons.append("rotation_span_below_30deg")
    for key in ("robot_id", "camera_pair_id", "stereo_calibration_id"):
        if not str(payload.get(key, "")).strip(): reasons.append(f"{key}_missing")
    return HandeyeGate(not reasons, tuple(dict.fromkeys(reasons)))

def solve_robot_world_handeye(samples: list[dict[str, Any]], *, robot_id: str, camera_pair_id: str, stereo_calibration_id: str) -> dict[str, Any]:
    import cv2
    require_charuco(cv2)
    gate = validate_session({"drag_mode":"manual_save_pause", "drag_teach_execute_called":False, "samples":samples,
                             "robot_id":robot_id, "camera_pair_id":camera_pair_id, "stereo_calibration_id":stereo_calibration_id,
                             "rotation_axes_span_deg":{"axis_1":31, "axis_2":31}})
    if not gate.ready: raise ValueError("handeye_session_rejected:" + ",".join(gate.reasons))
    base = [_matrix(s["base_T_gripper"], "base_T_gripper") for s in samples]
    camera = [_matrix(s["camera_T_charuco"], "camera_T_charuco") for s in samples]
    r_w2c, t_w2c, r_b2g, t_b2g = cv2.calibrateRobotWorldHandEye([x[:3,:3] for x in camera], [x[:3,3] for x in camera], [x[:3,:3] for x in base], [x[:3,3] for x in base])
    return {"source":"physical_charuco_robot_world_handeye", "validated":False, "trusted_for_execution":False,
            "robot_id":robot_id, "camera_pair_id":camera_pair_id, "stereo_calibration_id":stereo_calibration_id,
            "world_T_camera": np.block([[r_w2c, np.asarray(t_w2c).reshape(3,1)], [np.zeros((1,3)), np.ones((1,1))]]).tolist(),
            "base_T_gripper": np.block([[r_b2g, np.asarray(t_b2g).reshape(3,1)], [np.zeros((1,3)), np.ones((1,1))]]).tolist(),
            "method":"opencv_calibrateRobotWorldHandEye", "validation_reasons":["operator_confirmation_required"]}
