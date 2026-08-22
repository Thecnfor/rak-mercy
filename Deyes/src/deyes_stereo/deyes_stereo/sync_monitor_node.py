from __future__ import annotations

from typing import Any, Dict

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

from .sync_policy import MonitorParams, MonitorSeverity, SyncHealthMonitor


def _stamp_to_ns(header: Any) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


class DeyesSyncMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("deyes_sync_monitor")

        params = {
            "camera_id": "unknown_camera",
            "left_image_topic": "/stereo/left/image_raw",
            "right_image_topic": "/stereo/right/image_raw",
            "left_camera_info_topic": "/stereo/left/camera_info",
            "right_camera_info_topic": "/stereo/right/camera_info",
            "expected_min_rate_hz": 10.0,
            "image_timeout_sec": 0.5,
            "camera_info_timeout_sec": 1.0,
            "hard_sync_max_ms": 3.0,
            "soft_sync_max_ms": 10.0,
            "allow_soft_sync": False,
            "drop_gap_factor": 1.8,
            "stale_after_missed_frames": 3.0,
            "diagnostics_topic": "~/diagnostics",
            "gate_topic": "~/depth_gate_ok",
            "failure_reason_topic": "~/failure_reason",
            "evaluate_period_sec": 0.5,
        }
        for name, default in params.items():
            self.declare_parameter(name, default)

        self._camera_id = self.get_parameter("camera_id").value
        self._monitor = SyncHealthMonitor(
            MonitorParams(
                expected_min_rate_hz=float(self.get_parameter("expected_min_rate_hz").value),
                image_timeout_sec=float(self.get_parameter("image_timeout_sec").value),
                camera_info_timeout_sec=float(self.get_parameter("camera_info_timeout_sec").value),
                hard_sync_max_ms=float(self.get_parameter("hard_sync_max_ms").value),
                soft_sync_max_ms=float(self.get_parameter("soft_sync_max_ms").value),
                allow_soft_sync=bool(self.get_parameter("allow_soft_sync").value),
                drop_gap_factor=float(self.get_parameter("drop_gap_factor").value),
                stale_after_missed_frames=float(
                    self.get_parameter("stale_after_missed_frames").value
                ),
            )
        )
        self._last_summary = ""

        diagnostics_topic = self.get_parameter("diagnostics_topic").value
        gate_topic = self.get_parameter("gate_topic").value
        failure_reason_topic = self.get_parameter("failure_reason_topic").value
        evaluate_period_sec = float(self.get_parameter("evaluate_period_sec").value)

        self._diagnostics_pub = self.create_publisher(DiagnosticArray, diagnostics_topic, 10)
        self._gate_pub = self.create_publisher(Bool, gate_topic, 10)
        self._reason_pub = self.create_publisher(String, failure_reason_topic, 10)
        self._subscriptions = []

        subscriptions = {
            "left_image_topic": (Image, lambda msg: self._on_image("left", msg)),
            "right_image_topic": (Image, lambda msg: self._on_image("right", msg)),
            "left_camera_info_topic": (
                CameraInfo,
                lambda msg: self._on_camera_info("left", msg),
            ),
            "right_camera_info_topic": (
                CameraInfo,
                lambda msg: self._on_camera_info("right", msg),
            ),
        }
        for param_name, (msg_type, callback) in subscriptions.items():
            topic_name = self.get_parameter(param_name).value
            subscription = self.create_subscription(
                msg_type, topic_name, callback, qos_profile_sensor_data
            )
            self._subscriptions.append(subscription)

        self.create_timer(evaluate_period_sec, self._on_timer)
        self.get_logger().info("deyes_sync_monitor started")

    def _receipt_time_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _on_image(self, side: str, msg: Image) -> None:
        self._monitor.update_image(
            side,
            stamp_ns=_stamp_to_ns(msg.header),
            width=int(msg.width),
            height=int(msg.height),
            frame_id=msg.header.frame_id,
            encoding=msg.encoding,
            receipt_time_sec=self._receipt_time_sec(),
        )

    def _on_camera_info(self, side: str, msg: CameraInfo) -> None:
        self._monitor.update_camera_info(
            side,
            stamp_ns=_stamp_to_ns(msg.header),
            width=int(msg.width),
            height=int(msg.height),
            frame_id=msg.header.frame_id,
            receipt_time_sec=self._receipt_time_sec(),
        )

    def _on_timer(self) -> None:
        evaluation = self._monitor.evaluate(self._receipt_time_sec())
        self._publish_gate(evaluation.gate_ok, evaluation.summary)
        self._publish_diagnostics(evaluation.severity, evaluation.summary, evaluation.metrics)
        if evaluation.summary != self._last_summary:
            # ROS 2 Galactic 下同一个 rclpy logger 不能在 info/warn/error 之间
            # 动态切换，统一使用 info 输出状态摘要，严重级别保留在 diagnostics 中。
            diff_ms = evaluation.metrics.get("left_right_stamp_diff_ms", "nan")
            self.get_logger().info(f"{evaluation.summary} | stamp_diff_ms={diff_ms}")
            self._last_summary = evaluation.summary

    def _publish_gate(self, gate_ok: bool, summary: str) -> None:
        self._gate_pub.publish(Bool(data=gate_ok))
        self._reason_pub.publish(String(data=summary if not gate_ok else ""))

    def _publish_diagnostics(
        self, severity: MonitorSeverity, summary: str, metrics: Dict[str, str]
    ) -> None:
        status = DiagnosticStatus()
        status.name = f"{self.get_name()}/sync_health"
        status.hardware_id = self._camera_id
        # ROS 2 Galactic 的 diagnostic_msgs 要求 level 是长度为 1 的 bytes。
        status.level = bytes([int(severity)])
        status.message = summary
        status.values = [KeyValue(key=key, value=value) for key, value in sorted(metrics.items())]

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self._diagnostics_pub.publish(msg)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = DeyesSyncMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
