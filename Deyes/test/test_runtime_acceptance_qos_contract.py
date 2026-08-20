from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
MONITOR_PATH = (
    ROOT / "src" / "deyes_stereo" / "deyes_stereo" / "runtime_acceptance_monitor.py"
)


def test_runtime_monitor_uses_sensor_qos_for_sensor_streams() -> None:
    content = MONITOR_PATH.read_text(encoding="utf-8")

    assert "from rclpy.qos import qos_profile_sensor_data" in content
    assert "sensor_qos = qos_profile_sensor_data" in content
    assert '"/x1/left_camera/image_raw"' in content
    assert '"/x1/right_camera/image_raw"' in content
    assert '"/x1/stereo/depth"' in content
    assert '"/x1/stereo/points"' in content

    # These four subscriptions must not regress to the integer depth shorthand.
    sensor_topics = (
        '"/x1/left_camera/image_raw"',
        '"/x1/right_camera/image_raw"',
        '"/x1/stereo/depth"',
        '"/x1/stereo/points"',
    )
    for topic in sensor_topics:
        pattern = rf"create_subscription\(.*?{re.escape(topic)}.*?sensor_qos\s*\n\s*\)"
        assert re.search(pattern, content, flags=re.DOTALL)

    assert content.count("sensor_qos\n            )") == 4


def test_runtime_monitor_keeps_status_topics_on_explicit_reliable_depth() -> None:
    content = MONITOR_PATH.read_text(encoding="utf-8")

    # Diagnostics/String status are not sensor streams and retain the existing
    # reliable keep-last depth shorthand.
    assert '"/x1/stereo/pair_diagnostics", self.pair_diagnostics, 20' in content
    assert '"/x1/stereo/points_status", self.points_status, 20' in content
    assert '"/cuda_stereo_depth_node/status", self.depth_status, 20' in content
    assert '"/cuda_stereo_depth_node/status_detail", self.depth_detail, 20' in content
