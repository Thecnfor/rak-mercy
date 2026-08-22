"""AST-only safeguards for the ROS 2 Galactic runtime acceptance monitor."""

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MONITOR = ROOT / "src" / "deyes_stereo" / "deyes_stereo" / "runtime_acceptance_monitor.py"


def test_runtime_monitor_uses_lightweight_telemetry_not_large_payload_subscriptions() -> None:
    source = MONITOR.read_text(encoding="utf-8")
    tree = ast.parse(source)
    assert "MultiThreadedExecutor" in source
    assert "MutuallyExclusiveCallbackGroup" in source
    subscriptions = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "create_subscription"
    ]
    subscribed_types = {
        call.args[0].id for call in subscriptions
        if call.args and isinstance(call.args[0], ast.Name)
    }
    assert subscribed_types == {"DiagnosticArray", "String"}
    assert "sensor_msgs.msg import Image" not in source
    assert "PointCloud2" not in source


def test_runtime_monitor_uses_source_counters_and_accepted_pair_skew_only() -> None:
    source = MONITOR.read_text(encoding="utf-8")
    assert "published_pairs" in source
    assert "published_clouds" in source
    assert "delta not in (0, 1)" in source
    assert "current_skew_ms describes the last accepted pair" in source
    assert "_counter_rate(" in source
