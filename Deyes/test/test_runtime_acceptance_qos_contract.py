from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MONITOR_PATH = (
    ROOT / "src" / "deyes_stereo" / "deyes_stereo" / "runtime_acceptance_monitor.py"
)


def test_runtime_monitor_avoids_sensor_payload_subscriptions_for_rate_measurement() -> None:
    content = MONITOR_PATH.read_text(encoding="utf-8")

    assert "qos_profile_sensor_data" not in content
    assert '"/x1/left_camera/image_raw"' not in content
    assert '"/x1/right_camera/image_raw"' not in content
    assert '"/x1/stereo/depth"' not in content
    assert '"/x1/stereo/points"' not in content
    assert "published_pairs" in content
    assert "published_clouds" in content


def test_runtime_monitor_keeps_status_topics_on_explicit_reliable_depth() -> None:
    content = MONITOR_PATH.read_text(encoding="utf-8")

    # Diagnostics/String status are not sensor streams and retain the existing
    # reliable keep-last depth shorthand.
    assert '"/x1/stereo/pair_diagnostics", self.pair_diagnostics, 20' in content
    assert '"/x1/stereo/points_status", self.points_status, 20' in content
    assert '"/cuda_stereo_depth_node/status", self.depth_status, 20' in content
    assert '"/cuda_stereo_depth_node/status_detail", self.depth_detail, 20' in content
