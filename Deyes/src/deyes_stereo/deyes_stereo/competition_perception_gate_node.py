"""One-shot YOLO + CUDA-depth admission gate for the fixed competition venue."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .competition_perception_gate_contract import evaluate_detection_depth, stamp_ns


def _image_stamp_ns(msg: Image | CameraInfo) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


class CompetitionPerceptionGate(Node):
    def __init__(self, output: Path, timeout_sec: float) -> None:
        super().__init__("competition_perception_gate")
        self.output = output
        self.deadline = time.monotonic() + timeout_sec
        self.depth_by_stamp: dict[int, Image] = {}
        self.info_by_stamp: dict[int, CameraInfo] = {}
        self.result: dict | None = None
        self.create_subscription(Image, "/x1/stereo/depth", self._on_depth, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, "/x1/stereo/left/camera_info_rect", self._on_info, qos_profile_sensor_data
        )
        self.create_subscription(String, "/x1/detection/boxes", self._on_boxes, 10)

    @staticmethod
    def _trim(cache: dict, maximum: int = 40) -> None:
        while len(cache) > maximum:
            cache.pop(next(iter(cache)))

    def _on_depth(self, msg: Image) -> None:
        self.depth_by_stamp[_image_stamp_ns(msg)] = msg
        self._trim(self.depth_by_stamp)

    def _on_info(self, msg: CameraInfo) -> None:
        self.info_by_stamp[_image_stamp_ns(msg)] = msg
        self._trim(self.info_by_stamp)

    def _finish(self, accepted: bool, reason: str, **details: object) -> None:
        if self.result is not None:
            return
        self.result = {"accepted": accepted, "reason": reason, **details}
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(self.result, ensure_ascii=False, indent=2), encoding="utf-8")

    def _on_boxes(self, msg: String) -> None:
        if self.result is not None:
            return
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self._finish(False, "invalid_detection_json")
            return
        stamp = stamp_ns(payload)
        depth_msg = self.depth_by_stamp.get(stamp)
        info_msg = self.info_by_stamp.get(stamp)
        if depth_msg is None or info_msg is None:
            return  # A later detector frame normally matches the rolling caches.
        if depth_msg.encoding != "32FC1":
            self._finish(False, "depth_encoding_not_32FC1", encoding=depth_msg.encoding)
            return
        if depth_msg.width != info_msg.width or depth_msg.height != info_msg.height:
            self._finish(False, "depth_camera_info_size_mismatch")
            return
        if depth_msg.header.frame_id != info_msg.header.frame_id:
            self._finish(False, "depth_camera_info_frame_mismatch")
            return
        row_values = int(depth_msg.step) // 4
        values = np.frombuffer(depth_msg.data, dtype=np.float32)
        if row_values < depth_msg.width or values.size < row_values * depth_msg.height:
            self._finish(False, "depth_buffer_size_mismatch")
            return
        depth = values[: row_values * depth_msg.height].reshape(depth_msg.height, row_values)[
            :, : depth_msg.width
        ]
        gate = evaluate_detection_depth(payload, depth)
        self._finish(
            gate.accepted,
            gate.reason,
            transaction_id=payload.get("transaction_id", ""),
            stamp_ns=stamp,
            depth_m=gate.depth_m,
            valid_pixels=gate.valid_pixels,
            coverage=gate.coverage,
            bbox_xyxy=gate.bbox_xyxy,
            model_id=payload.get("model_id", ""),
        )


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--output", type=Path, default=Path("/tmp/deyes_perception_gate.json"))
    args, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = CompetitionPerceptionGate(args.output, args.timeout)
    try:
        while rclpy.ok() and node.result is None and time.monotonic() < node.deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.result is None:
            node._finish(False, "perception_timeout")
        accepted = bool(node.result and node.result.get("accepted"))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if accepted else 2)


if __name__ == "__main__":
    main(sys.argv[1:])
