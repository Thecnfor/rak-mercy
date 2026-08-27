import hashlib

from isaac_competition_65cm_adapter import (
    FIXED_PHYSICS_DT_SEC,
    classify_timeline_progress,
    validate_synthetic_pen_trace,
    validate_safe_base_adapter,
    motion_preflight,
)


BASE_ENV = {
    "ROS_DOMAIN_ID": "46",
    "X1_NAMESPACE": "x1_sim",
    "X1_ENABLE_MOTION": "1",
    "X1_ACKNOWLEDGE_MOTION_RISK": "1",
    "X1_EXECUTE": "1",
}


def test_motion_preflight_requires_isolated_domain_ack_and_no_real_devices():
    assert motion_preflight(BASE_ENV, device_paths=[])["accepted"]
    for key, value in (
        ("ROS_DOMAIN_ID", "45"),
        ("X1_NAMESPACE", "x1"),
        ("X1_ENABLE_MOTION", "0"),
        ("X1_ACKNOWLEDGE_MOTION_RISK", "0"),
        ("X1_EXECUTE", "0"),
    ):
        changed = dict(BASE_ENV)
        changed[key] = value
        assert not motion_preflight(changed, device_paths=[])["accepted"]
    assert not motion_preflight(BASE_ENV, device_paths=["/dev/right_arm"])["accepted"]


def test_headless_controller_uses_nonzero_fixed_physics_dt():
    result = motion_preflight(BASE_ENV, device_paths=[])
    assert FIXED_PHYSICS_DT_SEC == 1.0 / 60.0
    assert result["physics_hz"] == 60.0
    assert result["differential_controller_dt_sec"] > 0.0
    assert result["motion_scope"] == "isaac_sim_only"


def test_runtime_requires_playing_timeline_and_advancing_simulation_time():
    assert classify_timeline_progress(1.0, 1.1, playing=True) == (True, "ok")
    assert classify_timeline_progress(1.0, 1.0, playing=True) == (
        False,
        "simulation_time_not_advancing",
    )
    assert classify_timeline_progress(1.0, 1.1, playing=False) == (
        False,
        "timeline_not_playing",
    )


def test_synthetic_pen_trace_is_explicit_and_requires_30mm_lift():
    trace = {
        "synthetic_attachment": True,
        "rigid_body_disabled_while_carried": True,
        "rigid_body_reenabled_after_release": True,
        "release_settle_updates": 30,
        "initial_world_m": [3.62, 0.75, 0.658],
        "lifted_world_m": [3.62, 0.75, 0.700],
        "placed_world_m": [2.87, 0.14, 0.658],
    }
    assert validate_synthetic_pen_trace(trace) == (True, "ok")
    trace["lifted_world_m"][2] = 0.687
    assert validate_synthetic_pen_trace(trace) == (False, "pen_lift_below_30mm")
    trace["lifted_world_m"][2] = 0.700
    trace["synthetic_attachment"] = False
    assert validate_synthetic_pen_trace(trace) == (
        False,
        "synthetic_attachment_provenance_missing",
    )


def test_synthetic_pen_trace_requires_table_2_place_bounds():
    trace = {
        "synthetic_attachment": True,
        "rigid_body_disabled_while_carried": True,
        "rigid_body_reenabled_after_release": True,
        "release_settle_updates": 30,
        "initial_world_m": [3.49, 0.84, 0.656],
        "lifted_world_m": [3.49, 0.84, 0.716],
        "placed_world_m": [999.0, 999.0, 999.0],
    }
    assert validate_synthetic_pen_trace(trace) == (
        False,
        "pen_place_outside_table_2_fixture",
    )
    trace["placed_world_m"] = [2.87, 0.14, 0.800]
    assert validate_synthetic_pen_trace(trace) == (False, "pen_place_height_invalid")
    trace["placed_world_m"] = [2.87, 0.14, 0.668]
    trace["release_settle_updates"] = 0
    assert validate_synthetic_pen_trace(trace) == (
        False,
        "synthetic_release_not_settled",
    )


def test_safe_base_adapter_requires_pinned_sha256(tmp_path):
    adapter = tmp_path / "isolated_scene_adapter.py"
    adapter.write_text("# audited fixture\n", encoding="utf-8")
    expected = hashlib.sha256(adapter.read_bytes()).hexdigest()

    assert validate_safe_base_adapter(adapter, expected_sha256=expected) == (True, "ok")
    assert validate_safe_base_adapter(adapter, expected_sha256="0" * 64) == (
        False,
        "safe_base_adapter_sha256_mismatch",
    )
    assert validate_safe_base_adapter(tmp_path / "missing.py") == (
        False,
        "safe_base_adapter_missing",
    )
