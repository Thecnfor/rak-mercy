"""ROS 2 JSON wrapper around :mod:`pick_nav_contract`.

It is intentionally observation-only.  ``live_navigation_action`` is exposed
for future deployment wiring but defaults false and this node never creates an
ActionClient, publishes ``cmd_vel``, or addresses arm/gripper hardware.
"""

from __future__ import annotations

import json
from typing import Any, Callable

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .pick_nav_contract import NavPickLimits, PickNavCoordinator


class PickNavCoordinatorNode(Node):
    def __init__(self) -> None:
        super().__init__("pick_nav_coordinator_node")
        defaults = {
            "mission_topic": "/x1/pick/nav_mission", "navigation_evidence_topic": "/x1/pick/navigation_evidence",
            "transaction_status_topic": "/x1/pick/transaction_status", "pick_execution_status_topic": "/x1/pick/execution_status",
            "nav_gate_topic": "/x1/pick/nav_gate", "reset_service": "/x1/pick/nav_reset",
            "gate_heartbeat_sec": .1, "navigation_timeout_sec": 95.0, "live_navigation_action": False,
            "max_position_error_m": .05, "max_yaw_error_rad": .08, "max_linear_speed_mps": .01,
            "max_angular_speed_radps": .02, "stable_duration_sec": .5, "max_evidence_age_sec": .35,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        limits = NavPickLimits(**{key: float(self.get_parameter(key).value) for key in (
            "max_position_error_m", "max_yaw_error_rad", "max_linear_speed_mps", "max_angular_speed_radps", "stable_duration_sec", "max_evidence_age_sec")})
        self._machine = PickNavCoordinator(limits)
        self._navigation_started_ns: int | None = None
        gate_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._gate_pub = self.create_publisher(String, str(self.get_parameter("nav_gate_topic").value), gate_qos)
        self._subscribe("mission_topic", self._on_mission)
        self._subscribe("navigation_evidence_topic", self._on_navigation_evidence)
        self._subscribe("transaction_status_topic", self._machine.bind_snapshot)
        self._subscribe("pick_execution_status_topic", self._on_pick_execution_status)
        self.create_service(Trigger, str(self.get_parameter("reset_service").value), self._on_reset)
        self.create_timer(float(self.get_parameter("gate_heartbeat_sec").value), self._heartbeat)
        self.get_logger().warn("observation-only nav gate; live_navigation_action is disabled by default and no action/client command path exists")

    def _subscribe(self, parameter: str, handler: Callable[[dict[str, Any]], dict[str, Any]]) -> None:
        self.create_subscription(String, str(self.get_parameter(parameter).value), lambda msg: self._on_json(msg, handler), qos_profile_sensor_data)

    def _on_json(self, message: String, handler: Callable[[dict[str, Any]], dict[str, Any]]) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("payload_must_be_object")
            result = handler(payload)
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            result = self._machine._lock("invalid_json:" + str(exc))
        self._publish_gate(result)

    def _on_mission(self, payload: dict[str, Any]) -> dict[str, Any]:
        result = self._machine.start(payload)
        if result["state"] == "NAVIGATING":
            self._navigation_started_ns = self.get_clock().now().nanoseconds
        return result

    def _on_navigation_evidence(self, payload: dict[str, Any]) -> dict[str, Any]:
        result = self._machine.navigation_evidence(payload, now_ns=self.get_clock().now().nanoseconds)
        if result["state"] not in {"NAVIGATING", "ARRIVED_VERIFY"}:
            self._navigation_started_ns = None
        return result

    def _on_pick_execution_status(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Consume the existing executor status topic, never command it."""
        state = str(payload.get("state") or "").lower()
        if state == "executing":
            return self._machine.begin_pick(payload)
        if state in {"succeeded", "failed", "timeout", "timed_out", "cancelled", "rejected", "dry_run_complete"}:
            return self._machine.pick_terminal(payload)
        return self._machine._gate("pick_execution_nonterminal")

    def _publish_gate(self, result: dict[str, Any]) -> None:
        result["live_navigation_action"] = bool(self.get_parameter("live_navigation_action").value)
        output = String(); output.data = json.dumps(result, ensure_ascii=False, separators=(",", ":")); self._gate_pub.publish(output)

    def _heartbeat(self) -> None:
        # Missing navigation evidence must not leave a transaction in progress
        # indefinitely.  This is ROS-clock based so simulation time remains
        # deterministic, and still emits no command of any kind.
        if self._machine.state in {"NAVIGATING", "ARRIVED_VERIFY"} and self._navigation_started_ns is not None:
            timeout_ns = int(float(self.get_parameter("navigation_timeout_sec").value) * 1e9)
            now_ns = self.get_clock().now().nanoseconds
            if timeout_ns <= 0 or now_ns - self._navigation_started_ns >= timeout_ns:
                self._navigation_started_ns = None
                self._publish_gate(self._machine._lock("navigation_evidence_timeout"))
                return
        # Keep the authorization fresh while snapshot builds its post-arrival
        # stability window. Other states remain event-driven.
        if self._machine.state == "PICK_ARMED":
            self._publish_gate(self._machine._gate("arrival_verified_heartbeat"))

    def _on_reset(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        result = self._machine.reset(explicit=True)
        self._navigation_started_ns = None
        response.success = result["state"] == "IDLE"
        response.message = str(result["reason"])
        self._publish_gate(result)
        return response


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = PickNavCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
