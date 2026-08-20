"""ROS 2 field collector for the 10-minute stereo endurance acceptance report.

It records observations only; final pass/fail is delegated to the ROS-free
``stereo_acceptance`` contract so it can be reproduced off the robot.
"""

from __future__ import annotations

import argparse
import json
import re
import time
from pathlib import Path
from typing import Sequence

from .stereo_acceptance import AcceptanceInputError, _output_dir, evaluate_runtime_metrics, write_report


def _rate(times: list[float], observation_duration_sec: float) -> float:
    """Use the whole acceptance interval so a stream that stops cannot pass."""
    return len(times) / observation_duration_sec if observation_duration_sec > 0.0 else 0.0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Collect X1 stereo 10-minute runtime evidence")
    parser.add_argument("--output-dir", required=True, help="absolute temp directory outside repository")
    parser.add_argument("--duration-sec", type=float, default=600.0)
    parser.add_argument("--rviz-check-file", help="JSON object with the three required operator-confirmed RViz checks")
    args, ros_args = parser.parse_known_args(argv)
    if args.duration_sec <= 0.0:
        parser.error("duration-sec must be positive")
    try:
        output_dir = _output_dir(args.output_dir)
    except AcceptanceInputError as exc:
        parser.error(str(exc))
    try:
        import rclpy
        from rclpy.node import Node
        from diagnostic_msgs.msg import DiagnosticArray
        from sensor_msgs.msg import Image, PointCloud2
        from std_msgs.msg import String
        from rclpy.qos import qos_profile_sensor_data
    except ImportError as exc:  # Allows pure tests and reports on development PCs.
        parser.error(f"ROS 2 runtime unavailable: {exc}")

    class RuntimeCollector(Node):
        def __init__(self) -> None:
            super().__init__("stereo_runtime_acceptance_monitor")
            self.started = time.monotonic()
            self.left: list[float] = []
            self.right: list[float] = []
            self.depth: list[float] = []
            self.points: list[float] = []
            self.max_skew_ms: float | None = None
            self.pair_diagnostics_observed = False
            self.capture_failures = 0.0
            self.coverage: list[float] = []
            self.coverage_observed = False
            self.depth_status_observed = False
            self.overrun_started: float | None = None
            self.max_overrun_streak_sec = 0.0
            self.overrun_events = 0
            self.calibration_validated = False
            self.calibration_id = "unassigned"
            self.points_status_observed = False
            self.pointcloud_status_always_validated = True
            self.pointcloud_calibration_identity_consistent = True
            self._pointcloud_calibration_identity: str | None = None
            self.finished = False
            # Camera/depth/point-cloud publishers use rclcpp::SensorDataQoS
            # (BEST_EFFORT + VOLATILE); the integer depth shorthand creates a
            # reliable subscription and is incompatible with those streams.
            sensor_qos = qos_profile_sensor_data
            self.create_subscription(
                Image, "/x1/left_camera/image_raw", lambda _: self.left.append(time.monotonic()), sensor_qos
            )
            self.create_subscription(
                Image, "/x1/right_camera/image_raw", lambda _: self.right.append(time.monotonic()), sensor_qos
            )
            self.create_subscription(
                Image, "/x1/stereo/depth", lambda _: self.depth.append(time.monotonic()), sensor_qos
            )
            self.create_subscription(
                PointCloud2, "/x1/stereo/points", lambda _: self.points.append(time.monotonic()), sensor_qos
            )
            self.create_subscription(DiagnosticArray, "/x1/stereo/pair_diagnostics", self.pair_diagnostics, 20)
            self.create_subscription(DiagnosticArray, "/x1/stereo/points_status", self.points_status, 20)
            self.create_subscription(String, "/cuda_stereo_depth_node/status", self.depth_status, 20)
            self.create_subscription(String, "/cuda_stereo_depth_node/status_detail", self.depth_detail, 20)
            self.timer = self.create_timer(1.0, self.finish_if_due)

        def pair_diagnostics(self, message: DiagnosticArray) -> None:
            for status in message.status:
                values = {item.key: item.value for item in status.values}
                if "current_skew_ms" in values:
                    try:
                        skew = float(values["current_skew_ms"])
                        if skew >= 0.0:
                            self.pair_diagnostics_observed = True
                            self.max_skew_ms = skew if self.max_skew_ms is None else max(self.max_skew_ms, skew)
                    except ValueError:
                        pass
                for key in ("left_failures", "right_failures"):
                    try:
                        self.capture_failures = max(self.capture_failures, float(values.get(key, "0")))
                    except ValueError:
                        self.capture_failures = float("inf")

        def depth_status(self, message: String) -> None:
            now = time.monotonic()
            self.depth_status_observed = True
            if message.data == "processing_overrun":
                if self.overrun_started is None:
                    self.overrun_started = now
                    self.overrun_events += 1
            elif self.overrun_started is not None:
                self.max_overrun_streak_sec = max(self.max_overrun_streak_sec, now - self.overrun_started)
                self.overrun_started = None

        def points_status(self, message: DiagnosticArray) -> None:
            for status in message.status:
                values = {item.key: item.value for item in status.values}
                raw_validated = values.get("calibration_validated")
                calibration_id = values.get("calibration_id")
                if raw_validated is None or calibration_id is None:
                    continue
                self.points_status_observed = True
                self.calibration_validated = raw_validated.strip().lower() == "true"
                self.calibration_id = calibration_id
                if not self.calibration_validated or calibration_id.strip() in {"", "unassigned"}:
                    self.pointcloud_status_always_validated = False
                if self._pointcloud_calibration_identity is None:
                    self._pointcloud_calibration_identity = calibration_id
                elif calibration_id != self._pointcloud_calibration_identity:
                    self.pointcloud_calibration_identity_consistent = False

        def depth_detail(self, message: String) -> None:
            match = re.search(r"coverage_ratio_center_roi=([0-9.]+)", message.data)
            if match:
                self.coverage.append(float(match.group(1)))
                self.coverage_observed = True

        def finish_if_due(self) -> None:
            now = time.monotonic()
            if now - self.started < args.duration_sec:
                return
            if self.overrun_started is not None:
                self.max_overrun_streak_sec = max(self.max_overrun_streak_sec, now - self.overrun_started)
            rviz_checks = {name: False for name in (
                "flat_plane_has_no_obvious_warping_or_layering",
                "no_obvious_ghosting",
                "optical_axes_are_x_right_y_down_z_forward",
            )}
            if args.rviz_check_file:
                try:
                    with Path(args.rviz_check_file).open("r", encoding="utf-8") as handle:
                        supplied = json.load(handle)
                    if isinstance(supplied, dict):
                        rviz_checks.update(supplied)
                except (OSError, json.JSONDecodeError):
                    self.get_logger().error("RViz check file could not be read; manual checks remain false")
            observation_duration = now - self.started
            metrics = {
                "duration_sec": observation_duration,
                "capture_failures": self.capture_failures,
                "pair_max_skew_ms": self.max_skew_ms if self.max_skew_ms is not None else 0.0,
                "pair_diagnostics_observed": self.pair_diagnostics_observed,
                "left_image_hz": _rate(self.left, observation_duration),
                "right_image_hz": _rate(self.right, observation_duration),
                "depth_hz": _rate(self.depth, observation_duration),
                "points_hz": _rate(self.points, observation_duration),
                "center_roi_coverage_min": min(self.coverage) if self.coverage else 0.0,
                "depth_status_observed": self.depth_status_observed,
                "depth_coverage_observed": self.coverage_observed,
                # Five seconds is the explicit definition of a sustained overrun in this collector.
                "processing_overrun_sustained": self.max_overrun_streak_sec >= 5.0,
                "processing_overrun_events": self.overrun_events,
                "processing_overrun_max_streak_sec": self.max_overrun_streak_sec,
                "calibration_validated": self.calibration_validated,
                "calibration_id": self.calibration_id,
                "points_status_observed": self.points_status_observed,
                "pointcloud_status_always_validated": self.pointcloud_status_always_validated,
                "pointcloud_calibration_identity_consistent": self.pointcloud_calibration_identity_consistent,
                "rviz_manual_checks": rviz_checks,
            }
            raw_path = output_dir / "runtime_metrics.json"
            raw_path.write_text(json.dumps(metrics, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            report = evaluate_runtime_metrics(metrics)
            paths = write_report(report, output_dir, "runtime_acceptance", "X1 Stereo Runtime Acceptance")
            self.get_logger().info(
                f"wrote {raw_path}, {paths[0]} and {paths[1]}; validated={report['overall_validated']}")
            # Let the owning spin loop perform orderly node destruction and
            # shutdown.  Calling rclpy.shutdown() from a timer callback does
            # not reliably terminate rclpy.spin() on Galactic.
            self.finished = True

    rclpy.init(args=ros_args)
    collector = RuntimeCollector()
    try:
        while rclpy.ok() and not collector.finished:
            rclpy.spin_once(collector, timeout_sec=1.0)
    finally:
        collector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
