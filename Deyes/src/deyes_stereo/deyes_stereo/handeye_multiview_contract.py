"""ROS-free, fail-closed collection contract for physical eye-to-hand samples.

Each sample binds one explicitly identified checkerboard feature observed by
the *left rectified camera* to the same feature reached by an end effector in
``base_link``.  This module never discovers robot interfaces or commands a
robot.  Its output is a reviewable session report and, only when ready, the
input payload accepted by :mod:`handeye_calibration`.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
from math import acos, degrees, isfinite, sqrt
import argparse
import json
from pathlib import Path
from typing import Any


CAMERA_FRAME = "left_camera_optical_frame"
BASE_FRAME = "base_link"
ARM_SIDES = ("left", "right")


@dataclass(frozen=True)
class HandeyeCollectionLimits:
    """Physical collection thresholds; all are validation gates, not commands."""

    min_samples: int = 8
    max_stamp_skew_ms: float = 20.0
    min_translation_span_m: float = 0.08
    min_rotation_span_deg: float = 15.0
    min_distinct_pose_separation_m: float = 0.02
    min_distinct_pose_rotation_deg: float = 5.0


def _vector(value: Any, field: str, size: int) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f"{field}_must_be_{size}_finite_values")
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field}_must_be_{size}_finite_values") from exc
    if not all(isfinite(item) for item in result):
        raise ValueError(f"{field}_must_be_{size}_finite_values")
    return result


def _stamp(value: Any, field: str) -> int:
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field}_invalid") from exc
    if result <= 0:
        raise ValueError(f"{field}_invalid")
    return result


def _unit_quaternion(value: Any, field: str) -> tuple[float, float, float, float]:
    result = _vector(value, field, 4)
    magnitude = sqrt(sum(item * item for item in result))
    if magnitude < 1e-9:
        raise ValueError(f"{field}_zero_norm")
    return tuple(item / magnitude for item in result)  # type: ignore[return-value]


def _distance(first: tuple[float, ...], second: tuple[float, ...]) -> float:
    return sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def _quaternion_angle_deg(first: tuple[float, float, float, float], second: tuple[float, float, float, float]) -> float:
    dot = min(1.0, max(-1.0, abs(sum(a * b for a, b in zip(first, second)))))
    return degrees(2.0 * acos(dot))


def _reject(reason: str, *, reasons: list[str] | None = None) -> dict[str, Any]:
    return {
        "schema": "handeye_multiview_session/v1", "state": "rejected", "reason": reason,
        "reasons": reasons or [reason], "validated": False, "ready_for_solver": False,
        "dry_run": True, "commands_emitted": False,
    }


def _parse_sample(value: Any, limits: HandeyeCollectionLimits) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError("sample_must_be_object")
    camera = value.get("camera_checkerboard_pose")
    tool = value.get("end_effector_base_pose")
    if not isinstance(camera, dict) or not isinstance(tool, dict):
        raise ValueError("camera_checkerboard_pose_and_end_effector_base_pose_required")
    side = str(value.get("arm_side") or "")
    if side not in ARM_SIDES:
        raise ValueError("arm_side_must_be_left_or_right")
    if str(value.get("correspondence_id") or "").strip() == "":
        raise ValueError("correspondence_id_required")
    if camera.get("frame_id") != CAMERA_FRAME:
        raise ValueError("camera_checkerboard_frame_must_be_left_camera_optical_frame")
    if tool.get("frame_id") != BASE_FRAME:
        raise ValueError("end_effector_frame_must_be_base_link")
    camera_stamp = _stamp(camera.get("stamp_ns"), "camera_stamp_ns")
    tool_stamp = _stamp(tool.get("stamp_ns"), "end_effector_stamp_ns")
    skew_ms = abs(camera_stamp - tool_stamp) / 1e6
    if skew_ms > limits.max_stamp_skew_ms:
        raise ValueError("camera_end_effector_timestamp_skew_exceeds_limit")
    camera_position = _vector(camera.get("position_m"), "camera_checkerboard_position_m", 3)
    tool_position = _vector(tool.get("position_m"), "end_effector_position_base_m", 3)
    camera_quaternion = _unit_quaternion(camera.get("quaternion_xyzw"), "camera_checkerboard_quaternion_xyzw")
    tool_quaternion = _unit_quaternion(tool.get("quaternion_xyzw"), "end_effector_quaternion_xyzw")
    joints = _vector(value.get("joint_positions_deg"), "joint_positions_deg", 6)
    return {
        "sample_id": str(value.get("sample_id") or ""), "correspondence_id": str(value["correspondence_id"]),
        "arm_side": side, "camera_stamp_ns": camera_stamp, "end_effector_stamp_ns": tool_stamp,
        "stamp_skew_ms": round(skew_ms, 3), "camera_point_m": list(camera_position),
        "base_point_m": list(tool_position), "camera_quaternion_xyzw": list(camera_quaternion),
        "end_effector_quaternion_xyzw": list(tool_quaternion), "joint_positions_deg": list(joints),
    }


def build_handeye_multiview_session(payload: dict[str, Any], *, limits: HandeyeCollectionLimits = HandeyeCollectionLimits()) -> dict[str, Any]:
    """Validate a collection session; a successful session remains unvalidated.

    ``ready_for_solver`` means only that samples are internally coherent and
    varied enough to call the existing physical point-correspondence solver.
    It does *not* prove the resulting camera-to-base transform.
    """
    if not isinstance(payload, dict):
        return _reject("session_payload_invalid")
    if payload.get("dry_run", True) is not True:
        return _reject("dry_run_must_remain_true")
    if payload.get("request_execution") is True:
        return _reject("motion_execution_not_supported_by_handeye_collection")
    required = ("session_id", "calibration_id", "robot_id", "camera_pair_id", "stereo_calibration_id")
    missing = [name for name in required if not str(payload.get(name) or "").strip()]
    if missing:
        return _reject("session_identity_missing", reasons=["session_identity_missing", *[f"{name}_missing" for name in missing]])
    raw_samples = payload.get("samples")
    if not isinstance(raw_samples, list):
        return _reject("samples_must_be_list")
    parsed: list[dict[str, Any]] = []
    for index, raw in enumerate(raw_samples):
        try:
            sample = _parse_sample(raw, limits)
        except ValueError as exc:
            return _reject(str(exc), reasons=[str(exc), f"sample_index:{index}"])
        parsed.append(sample)
    reasons: list[str] = []
    if len(parsed) < limits.min_samples:
        reasons.append("insufficient_multiview_samples")
    ids = [sample["correspondence_id"] for sample in parsed]
    if len(set(ids)) != len(ids):
        reasons.append("correspondence_id_must_be_unique")
    camera_positions = [tuple(sample["camera_point_m"]) for sample in parsed]
    base_positions = [tuple(sample["base_point_m"]) for sample in parsed]
    camera_quaternions = [tuple(sample["camera_quaternion_xyzw"]) for sample in parsed]
    tool_quaternions = [tuple(sample["end_effector_quaternion_xyzw"]) for sample in parsed]
    camera_span = max((_distance(a, b) for a, b in combinations(camera_positions, 2)), default=0.0)
    base_span = max((_distance(a, b) for a, b in combinations(base_positions, 2)), default=0.0)
    camera_rotation_span = max((_quaternion_angle_deg(a, b) for a, b in combinations(camera_quaternions, 2)), default=0.0)
    tool_rotation_span = max((_quaternion_angle_deg(a, b) for a, b in combinations(tool_quaternions, 2)), default=0.0)
    if camera_span < limits.min_translation_span_m:
        reasons.append("camera_translation_diversity_insufficient")
    if base_span < limits.min_translation_span_m:
        reasons.append("base_translation_diversity_insufficient")
    if camera_rotation_span < limits.min_rotation_span_deg:
        reasons.append("camera_rotation_diversity_insufficient")
    if tool_rotation_span < limits.min_rotation_span_deg:
        reasons.append("end_effector_rotation_diversity_insufficient")
    for first, second in combinations(parsed, 2):
        camera_delta = _distance(tuple(first["camera_point_m"]), tuple(second["camera_point_m"]))
        angle_delta = _quaternion_angle_deg(tuple(first["camera_quaternion_xyzw"]), tuple(second["camera_quaternion_xyzw"]))
        if camera_delta < limits.min_distinct_pose_separation_m and angle_delta < limits.min_distinct_pose_rotation_deg:
            reasons.append("duplicate_or_near_duplicate_camera_pose")
            break
    metrics = {
        "sample_count": len(parsed), "camera_translation_span_m": round(camera_span, 6),
        "base_translation_span_m": round(base_span, 6), "camera_rotation_span_deg": round(camera_rotation_span, 3),
        "end_effector_rotation_span_deg": round(tool_rotation_span, 3),
        "max_stamp_skew_ms": round(max((sample["stamp_skew_ms"] for sample in parsed), default=0.0), 3),
        "arms_used": sorted({sample["arm_side"] for sample in parsed}),
    }
    ready = not reasons
    return {
        "schema": "handeye_multiview_session/v1", "state": "ready_for_solver" if ready else "rejected",
        "reason": "ok" if ready else reasons[0], "reasons": reasons, "validated": False,
        "ready_for_solver": ready, "dry_run": True, "commands_emitted": False,
        "session_id": str(payload["session_id"]), "calibration_id": str(payload["calibration_id"]),
        "robot_id": str(payload["robot_id"]), "camera_pair_id": str(payload["camera_pair_id"]),
        "stereo_calibration_id": str(payload["stereo_calibration_id"]), "metrics": metrics,
        "samples": parsed,
    }


def build_handeye_solver_payload(session: dict[str, Any], *, operator_confirmation: bool = False) -> dict[str, Any]:
    """Convert only a ready session to the existing solver's input schema."""
    if not isinstance(session, dict) or session.get("ready_for_solver") is not True:
        raise ValueError("handeye_session_not_ready_for_solver")
    return {
        "calibration_id": session["calibration_id"], "robot_id": session["robot_id"],
        "camera_pair_id": session["camera_pair_id"], "stereo_calibration_id": session["stereo_calibration_id"],
        "operator_confirmation": bool(operator_confirmation),
        "correspondences": [
            {"camera_point_m": sample["camera_point_m"], "base_point_m": sample["base_point_m"]}
            for sample in session["samples"]
        ],
    }


def main(argv: list[str] | None = None) -> None:
    """Write a reviewable collection report; this command cannot move a robot."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="operator-collected multiview JSON")
    parser.add_argument("--output", required=True, help="session report JSON outside the repository")
    args = parser.parse_args(argv)
    with Path(args.input).expanduser().open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    session = build_handeye_multiview_session(payload)
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as handle:
        json.dump(session, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    print(json.dumps({"output": str(output), "ready_for_solver": session["ready_for_solver"], "validated": False}, ensure_ascii=False))


if __name__ == "__main__":
    main()
