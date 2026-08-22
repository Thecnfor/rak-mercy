"""Read-only TF2 inventory, audit report, and site-template generator."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from .tf_frame_probe_contract import build_site_template


class TFFrameProbeNode(Node):
    def __init__(self) -> None:
        super().__init__("tf_frame_probe_node")
        self.declare_parameter("status_topic", "/x1/coordinate_chain/tf_probe")
        self.declare_parameter("report_path", "")
        self.declare_parameter("template_path", "")
        self._buffer = Buffer(); self._listener = TransformListener(self._buffer, self)
        self._publisher = self.create_publisher(String, str(self.get_parameter("status_topic").value), qos_profile_sensor_data)
        self.create_timer(1.0, self._probe)

    def _write_yaml(self, path_text: str, value: dict[str, Any]) -> None:
        if path_text:
            path = Path(path_text).expanduser(); path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(yaml.safe_dump(value, allow_unicode=True, sort_keys=False), encoding="utf-8")

    def _probe(self) -> None:
        try:
            tree = yaml.safe_load(self._buffer.all_frames_as_yaml()) or {}
            frames = sorted(str(name) for name in tree)
            report = {"level": "ok" if frames else "invalid", "interface": "tf2_read_only", "frame_count": len(frames), "frames": frames, "frame_parents": {name: str(value.get("parent", "")) for name, value in tree.items() if isinstance(value, dict)}}
            self._write_yaml(str(self.get_parameter("report_path").value), report)
            self._write_yaml(str(self.get_parameter("template_path").value), build_site_template(frames))
        except (TypeError, ValueError, yaml.YAMLError, OSError) as exc:
            report = {"level": "invalid", "interface": "tf2_read_only", "reason": str(exc), "frame_count": 0, "frames": []}
        message = String(); message.data = json.dumps(report, ensure_ascii=False, separators=(",", ":")); self._publisher.publish(message)


def main(args: Any = None) -> None:
    rclpy.init(args=args); node = TFFrameProbeNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
