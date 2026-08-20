"""Publish conservative office-pen grasp candidates from segmentation and depth."""

from __future__ import annotations

import json
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .depth_coordinate_node import depth_msg_to_array
from .extrinsics_contract import ExtrinsicsValidation, load_yaml_document, validate_extrinsics
from .pen_grasp_contract import build_pen_candidates


def _stamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


class PenGraspNode(Node):
    def __init__(self) -> None:
        super().__init__("pen_grasp_node")
        defaults = {
            "pen_features_topic": "/x1/detection/pen_features", "depth_topic": "/x1/stereo/depth",
            "camera_info_topic": "/x1/stereo/left/camera_info_rect", "plane_topic": "/x1/ground/plane",
            "output_topic": "/x1/grasp/pen_candidates", "status_topic": "/x1/grasp/pen_candidates_status",
            "extrinsics_path": "", "stereo_calibration_path": "", "max_feature_age_sec": .15,
            "min_depth_m": .20, "max_depth_m": 1.00, "min_plane_clearance_m": .004, "edge_margin_px": 12, "publish_period_sec": .08,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self._feature: dict[str, Any] | None = None
        self._feature_stamp_ns = 0
        self._plane: dict[str, Any] | None = None
        self._depth: np.ndarray | None = None
        self._depth_stamp_ns = 0
        self._camera_info: CameraInfo | None = None
        self._last_stamp_ns = -1
        self._max_feature_age_ns = int(float(self.get_parameter("max_feature_age_sec").value) * 1e9)
        self._trust = self._load_trust()
        self._output = self.create_publisher(String, str(self.get_parameter("output_topic").value), qos_profile_sensor_data)
        self._status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), qos_profile_sensor_data)
        self.create_subscription(String, str(self.get_parameter("pen_features_topic").value), self._on_feature, qos_profile_sensor_data)
        self.create_subscription(String, str(self.get_parameter("plane_topic").value), self._on_plane, qos_profile_sensor_data)
        self.create_subscription(Image, str(self.get_parameter("depth_topic").value), self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, str(self.get_parameter("camera_info_topic").value), self._on_camera, qos_profile_sensor_data)
        self.create_timer(float(self.get_parameter("publish_period_sec").value), self._on_timer)
        self._status("warn" if not self._trust.valid else "ok", trusted_for_grasp=self._trust.valid, reasons=list(self._trust.reasons))

    def _load_trust(self) -> ExtrinsicsValidation:
        try:
            extrinsics = load_yaml_document(str(self.get_parameter("extrinsics_path").value))
            stereo = load_yaml_document(str(self.get_parameter("stereo_calibration_path").value))
            return validate_extrinsics(extrinsics, stereo_document=stereo)
        except (OSError, ValueError) as exc:
            return ExtrinsicsValidation(False, (str(exc),))

    def _status(self, level: str, **values: Any) -> None:
        message = String()
        message.data = json.dumps({"level": level, **values}, ensure_ascii=False, separators=(",", ":"))
        self._status_pub.publish(message)

    def _on_feature(self, msg: String) -> None:
        try:
            self._feature = json.loads(msg.data)
            self._feature_stamp_ns = int(self._feature.get("stamp_sec", 0) or 0) * 1_000_000_000 + int(self._feature.get("stamp_nanosec", 0) or 0)
        except (json.JSONDecodeError, AttributeError, TypeError) as exc:
            self._status("warn", trusted_for_grasp=False, reason=f"invalid_pen_feature_json:{exc}")

    def _on_plane(self, msg: String) -> None:
        try:
            self._plane = json.loads(msg.data)
        except json.JSONDecodeError:
            self._plane = None

    def _on_depth(self, msg: Image) -> None:
        try:
            self._depth = depth_msg_to_array(msg)
            self._depth_stamp_ns = _stamp_ns(msg)
        except RuntimeError as exc:
            self._status("warn", trusted_for_grasp=False, reason=str(exc))

    def _on_camera(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _on_timer(self) -> None:
        if self._depth is None or self._camera_info is None or self._feature is None:
            return
        if self._depth_stamp_ns == self._last_stamp_ns:
            return
        if self._feature_stamp_ns and abs(self._feature_stamp_ns - self._depth_stamp_ns) > self._max_feature_age_ns:
            self._status("warn", trusted_for_grasp=False, reason="pen_feature_depth_timestamp_mismatch")
            return
        k = self._camera_info.k
        if len(k) < 9:
            self._status("warn", trusted_for_grasp=False, reason="camera_info_invalid")
            return
        try:
            result = build_pen_candidates(
                self._feature, self._depth, (float(k[0]), float(k[4]), float(k[2]), float(k[5])), plane_payload=self._plane,
                rotation=self._trust.rotation, translation=self._trust.translation, trusted_for_grasp=self._trust.valid,
                min_depth_m=float(self.get_parameter("min_depth_m").value), max_depth_m=float(self.get_parameter("max_depth_m").value),
                min_plane_clearance_m=float(self.get_parameter("min_plane_clearance_m").value), edge_margin_px=int(self.get_parameter("edge_margin_px").value),
            )
        except ValueError as exc:
            result = {"valid": False, "trusted_for_grasp": False, "reason": str(exc), "candidate_count": 0, "candidates": []}
        result.update({"stamp_sec": self._depth_stamp_ns // 1_000_000_000, "stamp_nanosec": self._depth_stamp_ns % 1_000_000_000, "source_frame": "left_camera_optical_frame", "extrinsics_calibration_id": self._trust.calibration_id})
        message = String()
        message.data = json.dumps(result, ensure_ascii=False, separators=(",", ":"))
        self._output.publish(message)
        self._status("ok" if result["valid"] else "warn", trusted_for_grasp=bool(result["trusted_for_grasp"]), reason=result["reason"], confidence=result.get("confidence", 0.0))
        self._last_stamp_ns = self._depth_stamp_ns

def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = PenGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
