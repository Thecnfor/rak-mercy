"""Observe and report the required camera/base/official end-effector TF paths."""

from __future__ import annotations

import json
from typing import Any

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class TFChainAuditNode(Node):
    def __init__(self) -> None:
        super().__init__("tf_chain_audit_node")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "left_camera_optical_frame")
        # Names intentionally default empty: use frame names discovered from
        # the official Mercury robot description, never guessed names.
        self.declare_parameter("required_end_effector_frames", [])
        self.declare_parameter("status_topic", "/x1/coordinate_chain/tf_audit")
        self._base = str(self.get_parameter("base_frame").value)
        self._camera = str(self.get_parameter("camera_frame").value)
        self._frames = [str(frame).strip() for frame in self.get_parameter("required_end_effector_frames").value if str(frame).strip()]
        self._buffer = Buffer(cache_time=Duration(seconds=5.0)); self._listener = TransformListener(self._buffer, self)
        self._publisher = self.create_publisher(String, str(self.get_parameter("status_topic").value), qos_profile_sensor_data)
        self.create_timer(1.0, self._audit)

    def _audit(self) -> None:
        required = [self._camera, *self._frames]
        missing: list[str] = []
        if not self._frames:
            missing.append("required_end_effector_frames_not_configured")
        for frame in required:
            try:
                self._buffer.lookup_transform(self._base, frame, Time(), timeout=Duration(seconds=0.05))
            except TransformException:
                missing.append(frame)
        payload = {"level": "ok" if not missing else "invalid", "base_frame": self._base, "camera_frame": self._camera, "required_end_effector_frames": self._frames, "observed": not missing, "missing": missing, "interface": "tf2"}
        message = String(); message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":")); self._publisher.publish(message)


def main(args: Any = None) -> None:
    rclpy.init(args=args); node = TFChainAuditNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
