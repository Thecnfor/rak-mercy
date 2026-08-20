import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "src" / "deyes_stereo"
sys.path.insert(0, str(PACKAGE_ROOT))

from deyes_stereo.sync_policy import MonitorParams, MonitorSeverity, SyncHealthMonitor


def feed_nominal_frame(monitor: SyncHealthMonitor, t_sec: float, stamp_ns: int) -> None:
    for side in ("left", "right"):
        monitor.update_image(
            side,
            stamp_ns=stamp_ns,
            width=1280,
            height=720,
            frame_id=f"{side}_camera_optical_frame",
            encoding="mono8",
            receipt_time_sec=t_sec,
        )
        monitor.update_camera_info(
            side,
            stamp_ns=stamp_ns,
            width=1280,
            height=720,
            frame_id=f"{side}_camera_optical_frame",
            receipt_time_sec=t_sec,
        )


class SyncPolicyTests(unittest.TestCase):
    def test_nominal_sync_is_valid(self) -> None:
        monitor = SyncHealthMonitor(MonitorParams(expected_min_rate_hz=5.0))
        feed_nominal_frame(monitor, 0.0, 0)
        feed_nominal_frame(monitor, 0.2, 200_000_000)
        result = monitor.evaluate(0.21)
        self.assertTrue(result.gate_ok)
        self.assertEqual(result.severity, MonitorSeverity.OK)

    def test_timestamp_diff_over_limit_rejects_output(self) -> None:
        monitor = SyncHealthMonitor(MonitorParams(expected_min_rate_hz=5.0, hard_sync_max_ms=3.0))
        feed_nominal_frame(monitor, 0.0, 0)
        monitor.update_image(
            "left",
            stamp_ns=200_000_000,
            width=1280,
            height=720,
            frame_id="left_camera_optical_frame",
            encoding="mono8",
            receipt_time_sec=0.2,
        )
        monitor.update_image(
            "right",
            stamp_ns=206_000_000,
            width=1280,
            height=720,
            frame_id="right_camera_optical_frame",
            encoding="mono8",
            receipt_time_sec=0.2,
        )
        monitor.update_camera_info(
            "left",
            stamp_ns=200_000_000,
            width=1280,
            height=720,
            frame_id="left_camera_optical_frame",
            receipt_time_sec=0.2,
        )
        monitor.update_camera_info(
            "right",
            stamp_ns=206_000_000,
            width=1280,
            height=720,
            frame_id="right_camera_optical_frame",
            receipt_time_sec=0.2,
        )
        result = monitor.evaluate(0.21)
        self.assertFalse(result.gate_ok)
        self.assertIn("stereo_soft_sync_only", result.reasons)

    def test_timestamp_diff_over_soft_limit_is_out_of_sync(self) -> None:
        monitor = SyncHealthMonitor(
            MonitorParams(
                expected_min_rate_hz=5.0,
                hard_sync_max_ms=3.0,
                soft_sync_max_ms=10.0,
                allow_soft_sync=False,
            )
        )
        feed_nominal_frame(monitor, 0.0, 0)
        monitor.update_image(
            "left",
            stamp_ns=200_000_000,
            width=1280,
            height=720,
            frame_id="left_camera_optical_frame",
            encoding="mono8",
            receipt_time_sec=0.2,
        )
        monitor.update_image(
            "right",
            stamp_ns=215_000_000,
            width=1280,
            height=720,
            frame_id="right_camera_optical_frame",
            encoding="mono8",
            receipt_time_sec=0.2,
        )
        monitor.update_camera_info(
            "left",
            stamp_ns=200_000_000,
            width=1280,
            height=720,
            frame_id="left_camera_optical_frame",
            receipt_time_sec=0.2,
        )
        monitor.update_camera_info(
            "right",
            stamp_ns=215_000_000,
            width=1280,
            height=720,
            frame_id="right_camera_optical_frame",
            receipt_time_sec=0.2,
        )
        result = monitor.evaluate(0.21)
        self.assertFalse(result.gate_ok)
        self.assertIn("stereo_out_of_sync", result.reasons)

    def test_camera_info_size_mismatch_is_error(self) -> None:
        monitor = SyncHealthMonitor(MonitorParams(expected_min_rate_hz=5.0))
        feed_nominal_frame(monitor, 0.0, 0)
        monitor.update_camera_info(
            "left",
            stamp_ns=200_000_000,
            width=640,
            height=480,
            frame_id="left_camera_optical_frame",
            receipt_time_sec=0.2,
        )
        monitor.update_image(
            "left",
            stamp_ns=200_000_000,
            width=1280,
            height=720,
            frame_id="left_camera_optical_frame",
            encoding="mono8",
            receipt_time_sec=0.2,
        )
        result = monitor.evaluate(0.21)
        self.assertFalse(result.gate_ok)
        self.assertIn("left_camera_info_size_mismatch", result.reasons)

    def test_stale_stream_is_rejected(self) -> None:
        monitor = SyncHealthMonitor(
            MonitorParams(expected_min_rate_hz=10.0, image_timeout_sec=0.2, stale_after_missed_frames=1.0)
        )
        feed_nominal_frame(monitor, 0.0, 0)
        result = monitor.evaluate(0.5)
        self.assertFalse(result.gate_ok)
        self.assertIn("left_image_stale", result.reasons)
        self.assertIn("right_image_stale", result.reasons)

    def test_low_rate_without_stale_is_warn_only(self) -> None:
        monitor = SyncHealthMonitor(
            MonitorParams(
                expected_min_rate_hz=30.0,
                image_timeout_sec=0.5,
                camera_info_timeout_sec=0.5,
                stale_after_missed_frames=20.0,
            )
        )
        feed_nominal_frame(monitor, 0.0, 0)
        feed_nominal_frame(monitor, 0.2, 200_000_000)
        result = monitor.evaluate(0.25)
        self.assertTrue(result.gate_ok)
        self.assertEqual(result.severity, MonitorSeverity.WARN)
        self.assertIn("left_image_low_rate", result.reasons)
        self.assertIn("right_image_low_rate", result.reasons)

    def test_soft_sync_can_be_allowed_for_static_replay(self) -> None:
        monitor = SyncHealthMonitor(
            MonitorParams(
                expected_min_rate_hz=5.0,
                hard_sync_max_ms=3.0,
                soft_sync_max_ms=10.0,
                allow_soft_sync=True,
            )
        )
        feed_nominal_frame(monitor, 0.0, 0)
        monitor.update_image(
            "left",
            stamp_ns=200_000_000,
            width=1280,
            height=720,
            frame_id="left_camera_optical_frame",
            encoding="mono8",
            receipt_time_sec=0.2,
        )
        monitor.update_image(
            "right",
            stamp_ns=206_000_000,
            width=1280,
            height=720,
            frame_id="right_camera_optical_frame",
            encoding="mono8",
            receipt_time_sec=0.2,
        )
        monitor.update_camera_info(
            "left",
            stamp_ns=200_000_000,
            width=1280,
            height=720,
            frame_id="left_camera_optical_frame",
            receipt_time_sec=0.2,
        )
        monitor.update_camera_info(
            "right",
            stamp_ns=206_000_000,
            width=1280,
            height=720,
            frame_id="right_camera_optical_frame",
            receipt_time_sec=0.2,
        )
        result = monitor.evaluate(0.21)
        self.assertTrue(result.gate_ok)
        self.assertEqual(result.severity, MonitorSeverity.WARN)
        self.assertIn("stereo_soft_sync_only", result.reasons)


if __name__ == "__main__":
    unittest.main()
