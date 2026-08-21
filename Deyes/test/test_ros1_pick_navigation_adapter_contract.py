import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).parents[2] / "scripts" / "pick_navigation_adapter_ros1.py"
SPEC = importlib.util.spec_from_file_location("pick_navigation_adapter_ros1", SCRIPT)
adapter = importlib.util.module_from_spec(SPEC); assert SPEC.loader is not None; SPEC.loader.exec_module(adapter)


PROFILE = {"schema": "pick_navigation_site/v1", "allowed_targets": [{"target_id": "table-1-front", "pose": {"frame_id": "map", "x": 1.25, "y": -0.75, "yaw_rad": 0.0}}]}
MISSION = {"mission_id": "m-1", "nav_epoch": 3, "target_id": "table-1-front", "pose": {"frame_id": "map", "x": 1.25, "y": -0.75, "yaw_rad": 0.0}}


def test_import_is_ros_free_and_startup_defaults_fail_closed():
    assert adapter.startup_gate(enable_navigation=False, operator_confirmed=True, site_profile_path="x") == "enable_navigation_false"
    assert adapter.startup_gate(enable_navigation=True, operator_confirmed=False, site_profile_path="x") == "operator_confirmed_false"
    assert adapter.startup_gate(enable_navigation=True, operator_confirmed=True, site_profile_path="") == "site_profile_path_missing"


def test_only_exact_allowlisted_table_pose_is_accepted():
    allowlist, reason = adapter.valid_site_profile(PROFILE)
    assert reason == "ok" and allowlist is not None
    assert adapter.validate_mission(MISSION, allowlist)[1] == "ok"
    changed = dict(MISSION); changed["pose"] = {**MISSION["pose"], "x": 1.250001}
    assert adapter.validate_mission(changed, allowlist)[1] == "mission_pose_not_exact_site_allowlist_match"
    assert adapter.validate_mission({**MISSION, "target_id": "other"}, allowlist)[1] == "mission_target_not_allowlisted"


def test_evidence_keeps_mission_epoch_and_has_no_command_claim():
    evidence = adapter.navigation_evidence(MISSION, result="succeeded", stamp_ns=99, reason="arrival_verified")
    assert evidence["mission_id"] == "m-1" and evidence["nav_epoch"] == 3
    assert evidence["commands_emitted"] is False and evidence["navigation_goal_sent"] is False
    sent = adapter.navigation_evidence(MISSION, result="succeeded", stamp_ns=99, reason="arrival_verified", goal_sent=True)
    assert sent["commands_emitted"] is True and sent["navigation_goal_sent"] is True


def test_source_contains_stale_check_repeated_success_and_completed_latch_logic():
    source = SCRIPT.read_text(encoding="utf-8")
    for marker in ("amcl_or_odom_stale_after_success", "(now - self._amcl_receipt).to_sec() > 0.5", "(now - self._odom_receipt).to_sec() > 0.5", "(now - self._last_success_publish).to_sec() >= 0.1", "(now - self._success_started).to_sec() >= 0.7", "adapter_completed_latched"):
        assert marker in source
