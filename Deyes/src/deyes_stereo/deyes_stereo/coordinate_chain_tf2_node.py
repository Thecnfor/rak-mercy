"""TF2-only camera point/pose gateway; it never creates actuator clients."""

from __future__ import annotations

import json
from typing import Any

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .coordinate_chain_contract import transform_request, trusted_for_execution, validate_request


class CoordinateChainTF2Node(Node):
    def __init__(self) -> None:
        super().__init__("coordinate_chain_tf2_node")
        for name, value in {
            "request_topic": "/x1/coordinate_chain/request",
            "result_topic": "/x1/coordinate_chain/result",
            "status_topic": "/x1/coordinate_chain/status",
            "extrinsics_status_topic": "/x1/stereo/extrinsics_status",
            "transform_timeout_sec": 0.05,
        }.items():
            self.declare_parameter(name, value)
        self._trust: dict[str, Any] | None = None
        self._buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._listener = TransformListener(self._buffer, self)
        self._result = self.create_publisher(String, str(self.get_parameter("result_topic").value), qos_profile_sensor_data)
        self._status = self.create_publisher(String, str(self.get_parameter("status_topic").value), qos_profile_sensor_data)
        self.create_subscription(String, str(self.get_parameter("extrinsics_status_topic").value), self._on_trust, qos_profile_sensor_data)
        self.create_subscription(String, str(self.get_parameter("request_topic").value), self._on_request, qos_profile_sensor_data)

    def _publish(self, publisher: Any, payload: dict[str, Any]) -> None:
        msg = String(); msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":")); publisher.publish(msg)

    def _on_trust(self, msg: String) -> None:
        try:
            value = json.loads(msg.data)
            self._trust = value if isinstance(value, dict) else None
        except json.JSONDecodeError:
            self._trust = None

    def _on_request(self, msg: String) -> None:
        try:
            raw = json.loads(msg.data)
            request = validate_request(raw)
            trusted, reason = trusted_for_execution(self._trust)
            if not trusted:
                self._publish(self._status, {"level": "invalid", "trusted_for_execution": False, "reason": reason})
                return
            stamp = Time(nanoseconds=request["stamp_ns"]) if request["stamp_ns"] else Time()
            transform = self._buffer.lookup_transform(request["target_frame"], request["source_frame"], stamp, timeout=Duration(seconds=float(self.get_parameter("transform_timeout_sec").value)))
            q, t = transform.transform.rotation, transform.transform.translation
            result = transform_request(request, np.asarray([[1 - 2*(q.y*q.y + q.z*q.z), 2*(q.x*q.y - q.z*q.w), 2*(q.x*q.z + q.y*q.w)], [2*(q.x*q.y + q.z*q.w), 1 - 2*(q.x*q.x + q.z*q.z), 2*(q.y*q.z - q.x*q.w)], [2*(q.x*q.z - q.y*q.w), 2*(q.y*q.z + q.x*q.w), 1 - 2*(q.x*q.x + q.y*q.y)]]), np.asarray([t.x, t.y, t.z]), tf_quaternion_xyzw=[q.x, q.y, q.z, q.w])
            # Preserve the bridge identity: an execution admission must prove
            # that its dry-run plan is for this exact transformed observation.
            result.update({"candidate_id": str(raw.get("candidate_id") or ""), "trusted_for_execution": True, "calibration_id": self._trust.get("calibration_id", "")})
            self._publish(self._result, result); self._publish(self._status, {"level": "ok", "trusted_for_execution": True, "target_frame": request["target_frame"]})
        except (ValueError, TransformException, json.JSONDecodeError) as exc:
            self._publish(self._status, {"level": "invalid", "trusted_for_execution": False, "reason": str(exc)})


def main(args: Any = None) -> None:
    rclpy.init(args=args); node = CoordinateChainTF2Node()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
