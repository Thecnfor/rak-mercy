#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import String


class OneShotHeatmapSaver(Node):
    def __init__(self, topic: str, output: Path) -> None:
        super().__init__("admin_gui_depth_heatmap_snapshot")
        self.output = output
        self.received = False
        self.subscription = self.create_subscription(
            String,
            topic,
            self.on_message,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

    def on_message(self, msg: String) -> None:
        payload = json.loads(msg.data)
        if "points" not in payload:
            raise RuntimeError("heatmap payload missing points field")
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(
            json.dumps(payload, ensure_ascii=False, separators=(",", ":")),
            encoding="utf-8",
        )
        self.received = True


def main() -> int:
    parser = argparse.ArgumentParser(description="Capture one ROS heatmap message to JSON.")
    parser.add_argument("--topic", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout", type=float, default=2.5)
    args = parser.parse_args()

    output = Path(args.output).resolve()
    rclpy.init()
    node = OneShotHeatmapSaver(args.topic, output)
    deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)

    try:
        while rclpy.ok() and not node.received:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.get_clock().now().nanoseconds >= deadline:
                raise TimeoutError(f"timeout waiting for heatmap on {args.topic}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
