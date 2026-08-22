"""Read-only ROS graph inspection for future motion adapters."""

from __future__ import annotations

import json
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .motion_adapter_contract import inspect_ros_interface_presence


class MotionInterfaceProbeNode(Node):
    def __init__(self) -> None:
        super().__init__("motion_interface_probe_node")
        for name, value in {
            "status_topic": "/x1/pick/interface_probe_status", "publish_period_sec": 2.0,
            "check_interfaces_only": True, "enable_execution": False,
        }.items():
            self.declare_parameter(name, value)
        self._status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), qos_profile_sensor_data)
        self.create_timer(float(self.get_parameter("publish_period_sec").value), self._publish)
        self.get_logger().warn("read-only interface probe: no action client, joint_states subscription, serial SDK, or command publisher")

    def _publish(self) -> None:
        report = inspect_ros_interface_presence(
            action_names_and_types=self.get_action_names_and_types(),
            topic_names_and_types=self.get_topic_names_and_types(),
        )
        report["enable_execution"] = bool(self.get_parameter("enable_execution").value)
        report["execution_permitted"] = False
        message = String()
        message.data = json.dumps(report, ensure_ascii=False, separators=(",", ":"))
        self._status_pub.publish(message)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = MotionInterfaceProbeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
