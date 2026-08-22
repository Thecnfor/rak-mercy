from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "src" / "deyes_bringup" / "launch" / "navigation_single_shot_pick.launch.py"
SINGLE = ROOT / "src" / "deyes_bringup" / "launch" / "single_shot_pick.launch.py"
CONFIG = ROOT / "config" / "stereo" / "pick_nav_coordinator.defaults.yaml"
SETUP = ROOT / "src" / "deyes_stereo" / "setup.py"
COORDINATOR = ROOT / "src" / "deyes_stereo" / "deyes_stereo" / "pick_nav_coordinator_node.py"


def test_navigation_wrapper_is_optional_dry_run_and_forces_gate_before_snapshot():
    source = LAUNCH.read_text(encoding="utf-8")
    assert 'executable="pick_nav_coordinator"' in source
    assert 'forwarded["require_nav_gate"] = "true"' in source
    assert 'if name != "live_navigation_action"' in source
    for name, default in (("dry_run", "true"), ("enable_live_execution", "false"), ("operator_confirmed", "false"), ("live_navigation_action", "false")):
        assert f'("{name}", "{default}")' in source
    assert "goals_launcher" not in source


def test_single_shot_keeps_navigation_gate_disabled_unless_wrapper_overrides_it():
    source = SINGLE.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("require_nav_gate",default_value="false")' in source
    assert '"require_nav_gate":LaunchConfiguration("require_nav_gate")' in source


def test_coordinator_is_registered_and_remains_without_nav2_package_dependency():
    assert "pick_nav_coordinator = deyes_stereo.pick_nav_coordinator_node:main" in SETUP.read_text(encoding="utf-8")
    config = CONFIG.read_text(encoding="utf-8")
    assert "live_navigation_action: false" in config
    assert 'transaction_status_topic: "/x1/pick/transaction_status"' in config
    assert 'pick_execution_status_topic: "/x1/pick/execution_status"' in config
    assert "gate_heartbeat_sec: 0.1" in config
    assert "navigation_timeout_sec: 95.0" in config
    package = (ROOT / "src" / "deyes_stereo" / "package.xml").read_text(encoding="utf-8")
    assert "nav2_msgs" not in package


def test_nav_gate_qos_is_latched_reliable_and_authorization_has_heartbeat():
    source = COORDINATOR.read_text(encoding="utf-8")
    assert "ReliabilityPolicy.RELIABLE" in source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in source
    assert "create_timer" in source and "_heartbeat" in source
    assert 'reset_service": "/x1/pick/nav_reset"' in source
    assert '"navigation_timeout_sec": 95.0' in source
    assert '"navigation_evidence_timeout"' in source
    assert "self._navigation_started_ns = None" in source
