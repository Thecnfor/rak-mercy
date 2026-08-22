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


def _counter_rate(first: int | None, last: int | None, observation_duration_sec: float) -> float:
    """Rate from a producer counter over the complete acceptance interval.

    Dividing by the full interval, rather than the span between received
    diagnostics, detects a source that stops half way through the run.
    """
    if first is None or last is None or last < first or observation_duration_sec <= 0.0:
        return 0.0
    return (last - first) / observation_duration_sec


def _message_rate(message_count: int, observation_duration_sec: float) -> float:
    """Use only small state messages; never deserialize image/PointCloud payloads."""
    return message_count / observation_duration_sec if observation_duration_sec > 0.0 else 0.0


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
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from diagnostic_msgs.msg import DiagnosticArray
        from std_msgs.msg import String
    except ImportError as exc:  # Allows pure tests and reports on development PCs.
        parser.error(f"ROS 2 runtime unavailable: {exc}")

    class RuntimeCollector(Node):
        def __init__(self) -> None:
            super().__init__("stereo_runtime_acceptance_monitor")
            self.started = time.monotonic()
            self.max_skew_ms: float | None = None
            self.pair_diagnostics_observed = False
            self.pair_diagnostics_counter_contiguous = True
            self.capture_first_published_pairs: int | None = None
            self.capture_last_published_pairs: int | None = None
            self.capture_drop_skew = 0
            self.capture_drop_stale = 0
            self.capture_wait_pair = 0
            self.capture_failures = 0.0
            self.coverage: list[float] = []
            self.coverage_observed = False
            self.depth_status_observed = False
            self.depth_published_messages = 0
            self.overrun_started: float | None = None
            self.max_overrun_streak_sec = 0.0
            self.overrun_events = 0
            self.calibration_validated = False
            self.calibration_id = "unassigned"
            self.points_status_observed = False
            self.pointcloud_status_always_validated = True
            self.pointcloud_calibration_identity_consistent = True
            self._pointcloud_calibration_identity: str | None = None
            self.points_first_published_clouds: int | None = None
            self.points_last_published_clouds: int | None = None
            self.points_counter_monotonic = True
            self.finished = False
            # Do not subscribe to the four large image/cloud payload topics.
            # Python message deserialization itself was observed to make the old
            # single-threaded collector undercount healthy 30/14 Hz publishers.
            # These source-owned diagnostic/state topics carry the authoritative
            # producer counters and remain lightweight on ROS 2 Galactic.
            self._pair_group = MutuallyExclusiveCallbackGroup()
            self._points_group = MutuallyExclusiveCallbackGroup()
            self._depth_group = MutuallyExclusiveCallbackGroup()
            self._timer_group = MutuallyExclusiveCallbackGroup()
            self.create_subscription(
                DiagnosticArray, "/x1/stereo/pair_diagnostics", self.pair_diagnostics, 20,
                callback_group=self._pair_group)
            self.create_subscription(
                DiagnosticArray, "/x1/stereo/points_status", self.points_status, 20,
                callback_group=self._points_group)
            self.create_subscription(
                String, "/cuda_stereo_depth_node/status", self.depth_status, 20,
                callback_group=self._depth_group)
            self.create_subscription(
                String, "/cuda_stereo_depth_node/status_detail", self.depth_detail, 20,
                callback_group=self._depth_group)
            self.timer = self.create_timer(1.0, self.finish_if_due, callback_group=self._timer_group)

        def pair_diagnostics(self, message: DiagnosticArray) -> None:
            for status in message.status:
                values = {item.key: item.value for item in status.values}
                try:
                    published_pairs = int(values["published_pairs"])
                except (KeyError, ValueError):
                    continue
                if self.capture_first_published_pairs is None:
                    self.capture_first_published_pairs = published_pairs
                if self.capture_last_published_pairs is not None:
                    delta = published_pairs - self.capture_last_published_pairs
                    # current_skew_ms describes the last accepted pair only when
                    # exactly one new pair was published. A rejected candidate
                    # may otherwise report a large skew and must not taint the
                    # accepted-pair <=10ms gate.
                    if delta not in (0, 1):
                        self.pair_diagnostics_counter_contiguous = False
                if self.capture_last_published_pairs is None or published_pairs > self.capture_last_published_pairs:
                    if "current_skew_ms" in values and self.capture_last_published_pairs is not None:
                        try:
                            skew = float(values["current_skew_ms"])
                            if skew >= 0.0:
                                self.pair_diagnostics_observed = True
                                self.max_skew_ms = skew if self.max_skew_ms is None else max(self.max_skew_ms, skew)
                        except ValueError:
                            self.pair_diagnostics_counter_contiguous = False
                self.capture_last_published_pairs = published_pairs
                for key, attribute in (
                    ("drop_skew", "capture_drop_skew"),
                    ("drop_stale", "capture_drop_stale"),
                    ("wait_pair", "capture_wait_pair"),
                ):
                    try:
                        setattr(self, attribute, max(getattr(self, attribute), int(values.get(key, "0"))))
                    except ValueError:
                        self.pair_diagnostics_counter_contiguous = False
                for key in ("left_failures", "right_failures"):
                    try:
                        self.capture_failures = max(self.capture_failures, float(values.get(key, "0")))
                    except ValueError:
                        self.capture_failures = float("inf")

        def depth_status(self, message: String) -> None:
            now = time.monotonic()
            self.depth_status_observed = True
            if message.data in {"ok", "processing_overrun"}:
                self.depth_published_messages += 1
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
                raw_published_clouds = values.get("published_clouds")
                if raw_validated is None or calibration_id is None or raw_published_clouds is None:
                    continue
                try:
                    published_clouds = int(raw_published_clouds)
                except ValueError:
                    self.points_counter_monotonic = False
                    continue
                self.points_status_observed = True
                if self.points_first_published_clouds is None:
                    self.points_first_published_clouds = published_clouds
                if self.points_last_published_clouds is not None and published_clouds < self.points_last_published_clouds:
                    self.points_counter_monotonic = False
                self.points_last_published_clouds = published_clouds
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
                "pair_diagnostics_counter_contiguous": self.pair_diagnostics_counter_contiguous,
                "capture_published_pairs_delta": (
                    (self.capture_last_published_pairs or 0) - (self.capture_first_published_pairs or 0)),
                "capture_drop_skew": self.capture_drop_skew,
                "capture_drop_stale": self.capture_drop_stale,
                "capture_wait_pair": self.capture_wait_pair,
                "left_image_hz": _counter_rate(
                    self.capture_first_published_pairs, self.capture_last_published_pairs, observation_duration),
                "right_image_hz": _counter_rate(
                    self.capture_first_published_pairs, self.capture_last_published_pairs, observation_duration),
                "depth_hz": _message_rate(self.depth_published_messages, observation_duration),
                "points_hz": _counter_rate(
                    self.points_first_published_clouds, self.points_last_published_clouds, observation_duration),
                "points_published_clouds_delta": (
                    (self.points_last_published_clouds or 0) - (self.points_first_published_clouds or 0)),
                "points_counter_monotonic": self.points_counter_monotonic,
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
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(collector)
    try:
        while rclpy.ok() and not collector.finished:
            executor.spin_once(timeout_sec=1.0)
    finally:
        executor.shutdown()
        collector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
