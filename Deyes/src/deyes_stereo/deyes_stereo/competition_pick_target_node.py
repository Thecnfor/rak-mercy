"""Optional ROS 2 publishing boundary for venue pick targets."""
from __future__ import annotations
import json


def main(args=None) -> None:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError("competition_pick_target_node requires ROS 2") from exc

    class NodeImpl(Node):
        def __init__(self):
            super().__init__("competition_pick_target_node")
            self.declare_parameter("output_topic", "/x1/competition/pick_target")
            publisher = self.create_publisher(String, str(self.get_parameter("output_topic").value), 1)
            message = String()
            message.data = json.dumps({"schema":"competition_pick_target/v1", "valid":False,
                "trusted_for_venue_execution":False, "reason":"waiting_for_exact_stamp_projector_adapter",
                "commands_emitted":False})
            publisher.publish(message)
            self._publisher = publisher

    rclpy.init(args=args); node = NodeImpl()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
