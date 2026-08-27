#!/usr/bin/env python3
"""Capture post-lift YOLO evidence for the original target ROI.

Live mode consumes fresh detector payloads from ROS 2.  Fixture mode replays the
same JSON payloads for contract tests; it never represents a live-system pass.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time
from typing import Iterable


MODEL_ID = "pen-yolov5-student-01875-416-v1"


def _load_target(path: Path, roi_half_size_px: float) -> tuple[list[float], list[float]]:
    target = json.loads(path.read_text(encoding="utf-8"))
    if target.get("schema") != "competition_pick_target/v1" or target.get("valid") is not True:
        raise ValueError("target must be a valid competition_pick_target/v1")
    pixel = target.get("pixel_uv")
    if not isinstance(pixel, list) or len(pixel) != 2:
        raise ValueError("target pixel_uv missing; live ROI grasp verification is unavailable")
    u, v = (float(pixel[0]), float(pixel[1]))
    if not math.isfinite(u) or not math.isfinite(v):
        raise ValueError("target pixel_uv is not finite")
    return [u, v], [u - roi_half_size_px, v - roi_half_size_px,
                    u + roi_half_size_px, v + roi_half_size_px]


def _bbox_intersects_roi(bbox: object, roi: list[float]) -> bool:
    if not isinstance(bbox, list) or len(bbox) != 4:
        raise ValueError("detector bbox_xyxy must contain four values")
    x1, y1, x2, y2 = (float(value) for value in bbox)
    if not all(math.isfinite(value) for value in (x1, y1, x2, y2)) or x2 < x1 or y2 < y1:
        raise ValueError("detector bbox_xyxy is invalid")
    return x2 >= roi[0] and x1 <= roi[2] and y2 >= roi[1] and y1 <= roi[3]


def _frame_has_pen_in_roi(payload: object, roi: list[float]) -> tuple[bool, int]:
    if not isinstance(payload, dict):
        raise ValueError("detector payload must be an object")
    if payload.get("model_id") != MODEL_ID:
        raise ValueError("detector model_id mismatch")
    detections = payload.get("detections")
    if not isinstance(detections, list):
        raise ValueError("detector payload detections must be a list")
    if any(not isinstance(item, dict) for item in detections):
        raise ValueError("detector detection item must be an object")
    stamp_ns = int(payload.get("stamp_sec", 0)) * 1_000_000_000 + int(payload.get("stamp_nanosec", 0))
    if stamp_ns <= 0:
        raise ValueError("detector payload stamp missing")
    return any(_bbox_intersects_roi(item.get("bbox_xyxy"), roi) for item in detections), stamp_ns


def _collect_clear_frames(payloads: Iterable[object], roi: list[float], required: int) -> tuple[list[bool], list[int]]:
    clear: list[bool] = []
    stamps: list[int] = []
    last_stamp = 0
    for payload in payloads:
        has_pen, stamp_ns = _frame_has_pen_in_roi(payload, roi)
        if stamp_ns <= last_stamp:
            raise ValueError("detector payload stamps are not strictly increasing")
        last_stamp = stamp_ns
        if has_pen:
            clear.clear(); stamps.clear()
        else:
            clear.append(False); stamps.append(stamp_ns)
            if len(clear) == required:
                return clear, stamps
    raise ValueError(f"did not observe {required} consecutive clear original-ROI frames")


def _fixture_payloads(path: Path) -> list[object]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def _live_payloads(topic: str, timeout: float, roi: list[float], required: int) -> list[object]:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import String
    except ImportError as exc:
        raise RuntimeError(f"ROS 2 Python unavailable: {exc}") from exc

    payloads: list[object] = []
    consecutive_clear = 0
    last_stamp = 0
    error: str | None = None
    rclpy.init(args=None)
    node = Node("competition_grasp_feedback_adapter")

    def receive(message: String) -> None:
        nonlocal consecutive_clear, last_stamp, error
        try:
            payload = json.loads(message.data)
            has_pen, stamp_ns = _frame_has_pen_in_roi(payload, roi)
            if stamp_ns <= last_stamp:
                raise ValueError("detector payload stamps are not strictly increasing")
            last_stamp = stamp_ns
            payloads.append(payload)
            consecutive_clear = 0 if has_pen else consecutive_clear + 1
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            error = str(exc)

    node.create_subscription(String, topic, receive, qos_profile_sensor_data)
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline and consecutive_clear < required and error is None:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if error is not None:
        raise ValueError(error)
    return payloads


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-json", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--topic", default="/x1/detection/boxes")
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--required-clear-frames", type=int, default=3)
    parser.add_argument("--roi-half-size-px", type=float, default=24.0)
    parser.add_argument("--empty-closed-feedback", type=float, required=True)
    parser.add_argument("--gripper-feedback", type=float, required=True)
    parser.add_argument("--detections-fixture", type=Path)
    args = parser.parse_args()
    try:
        if args.required_clear_frames != 3:
            raise ValueError("competition grasp verification requires exactly three clear frames")
        pixel, roi = _load_target(args.target_json, args.roi_half_size_px)
        payloads = (_fixture_payloads(args.detections_fixture) if args.detections_fixture
                    else _live_payloads(args.topic, args.timeout, roi, args.required_clear_frames))
        clear, stamps = _collect_clear_frames(payloads, roi, args.required_clear_frames)
        result = {
            "schema": "competition_grasp_feedback/v1",
            "source": "fixture_replay" if args.detections_fixture else "live_ros2_and_mercury_feedback",
            "live": args.detections_fixture is None,
            "target_pixel_uv": pixel,
            "original_roi_xyxy": roi,
            "roi_pen_last3": clear,
            "detector_stamp_ns_last3": stamps,
            "empty_closed_feedback": args.empty_closed_feedback,
            "gripper_feedback": args.gripper_feedback,
            "gripper_feedback_delta": args.gripper_feedback - args.empty_closed_feedback,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return 0
    except (OSError, ValueError, TypeError, json.JSONDecodeError, RuntimeError) as exc:
        print(str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
