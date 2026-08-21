"""Publish TF2 requests from visual optical-frame candidates; never actuate."""

from __future__ import annotations

import json
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .coordinate_chain_bridge_contract import build_coordinate_chain_requests


class CoordinateChainCandidateBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("coordinate_chain_candidate_bridge_node")
        for name, value in {"candidate_topic": "/x1/grasp/camera_candidates", "request_topic": "/x1/coordinate_chain/request", "status_topic": "/x1/coordinate_chain/bridge_status", "extrinsics_status_topic": "/x1/stereo/extrinsics_status", "target_frame": ""}.items():
            self.declare_parameter(name, value)
        self._extrinsics_status: dict[str, Any] | None = None
        self._request_pub = self.create_publisher(String, str(self.get_parameter("request_topic").value), qos_profile_sensor_data)
        self._status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), qos_profile_sensor_data)
        self.create_subscription(String, str(self.get_parameter("extrinsics_status_topic").value), self._on_extrinsics, qos_profile_sensor_data)
        self.create_subscription(String, str(self.get_parameter("candidate_topic").value), self._on_candidate, qos_profile_sensor_data)

    def _publish(self, publisher: Any, payload: dict[str, Any]) -> None:
        message = String(); message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":")); publisher.publish(message)

    def _on_extrinsics(self, message: String) -> None:
        try:
            value = json.loads(message.data); self._extrinsics_status = value if isinstance(value, dict) else None
        except json.JSONDecodeError:
            self._extrinsics_status = None

    def _on_candidate(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            self._publish(self._status_pub, {"level": "invalid", "published": False, "reason": "candidate_json_invalid"})
            return
        result = build_coordinate_chain_requests(payload, target_frame=str(self.get_parameter("target_frame").value), extrinsics_status=self._extrinsics_status)
        if result["published"]:
            for request in result["requests"]:
                self._publish(self._request_pub, request)
        self._publish(self._status_pub, {"level": "ok" if result["published"] else "invalid", "published": result["published"], "reason": result["reason"], "request_count": len(result["requests"])})


def main(args: Any = None) -> None:
    rclpy.init(args=args); node = CoordinateChainCandidateBridgeNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
