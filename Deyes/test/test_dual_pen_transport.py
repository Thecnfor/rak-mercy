"""Offline acceptance tests for dual-pen transport inside the yellow work zone."""

from dataclasses import replace
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "deyes_stereo"))

from deyes_stereo.dual_pen_cograsp_contract import WorkspaceBounds  # noqa: E402
from deyes_stereo.dual_pen_transport_contract import (  # noqa: E402
    DualPenTransportProfile, NavigationPose, SimulationWorldBinding,
    YellowWorkZone, build_dual_pen_transport_plan,
)
from deyes_stereo.dual_pen_transport_simulation import (  # noqa: E402
    FakeDualPenTransportAdapter, run_dual_pen_transport_simulation,
)


NOW = 60_000_000_000


def _profile():
    return DualPenTransportProfile(
        validated_for_simulation=True,
        left_workspace=WorkspaceBounds(.1, 1.2, 0., .8, .7, 1.6),
        right_workspace=WorkspaceBounds(.1, 1.2, -.8, 0., .7, 1.6),
        lift_vector_base_unit=(0., 0., 1.), min_pen_separation_m=.10,
        min_tool_clearance_m=.08,
    )


def _route():
    return (
        NavigationPose("pickup_work_pose", "map", 3.14, .75, 0.),
        NavigationPose("place_work_pose", "map", 2.96, .55, -1.53588974175501),
        YellowWorkZone("yellow_work_zone_v3", "map", 2.60, 3.18, .55, 1.12),
    )


def _world():
    return SimulationWorldBinding(
        simulation_execution_allowed=True, world_id="team_rak_finals_20260820_v4",
        scene_path="/var/workspace/docker/isaac/scenes/team_rak_v4/outputs/team_rak_v4.usd",
        scene_sha256="11d59b9fff96304d263d2d6df4e4958b876b30fe0a1b03ca461098944a419cd6",
        random_seed=1363, validated_at_ns=NOW,
    )


def _transport_safety():
    pickup, place, zone = _route()
    world = _world()
    return {
        "source": "isaac_sim_collision_sweep",
        "transport_safe_pose_confirmed": True,
        "full_transport_collision_sweep_validated": True,
        "world_id": world.world_id, "scene_sha256": world.scene_sha256,
        "random_seed": world.random_seed, "yellow_work_zone_id": zone.zone_id,
        "pickup_pose_id": pickup.pose_id, "place_pose_id": place.pose_id,
        "stamp_ns": NOW,
    }


def _drops():
    return {
        "left": {"drop_id": "table_2_left_slot", "table_id": "table_2", "target_frame": "base_link", "position_base_m": [.45, .20, 1.20], "approach_normal_base_unit": [0., 0., 1.]},
        "right": {"drop_id": "table_2_right_slot", "table_id": "table_2", "target_frame": "base_link", "position_base_m": [.48, -.20, 1.20], "approach_normal_base_unit": [0., 0., 1.]},
    }


def _candidate(target_id, point, confidence=.95, **changes):
    value = {"target_id": target_id, "label": "pen", "source_table_id": "table_1", "valid": True, "trusted_for_grasp": True, "target_frame": "base_link", "stamp_ns": NOW, "confidence": confidence, "grasp_point_base_m": point, "axis_base_unit": [1., 0., 0.], "approach_normal_base_unit": [0., 0., 1.]}
    value.update(changes)
    return value


def _payload(candidates=None, **changes):
    value = {
        "camera_to_base": {"source": "isaac_sim_scene_tf", "simulation_validated": True, "physical_validated": False, "source_frame": "Left_camera", "target_frame": "base_link", "transform_id": "team_rak_tf", "stamp_ns": NOW},
        "candidates": candidates or [_candidate("pen-left", [.55, .25, 1.20]), _candidate("pen-right", [.58, -.24, 1.20]), _candidate("low-confidence", [.70, .30, 1.20], confidence=.40)],
    }
    value.update(changes)
    return value


def _plan(payload=None, **changes):
    pickup, place, zone = _route()
    kwargs = dict(perception_payload=payload or _payload(), now_stamp_ns=NOW, profile=_profile(), pickup_work_pose=pickup, place_work_pose=place, yellow_work_zone=zone, table_2_drop_targets=_drops(), simulation_world=_world(), loaded_transport_safety_contract=_transport_safety())
    kwargs.update(changes)
    return build_dual_pen_transport_plan(**kwargs)


def _run_kwargs():
    pickup, place, zone = _route()
    return dict(perception_payload=_payload(), now_stamp_ns=NOW, profile=_profile(), pickup_work_pose=pickup, place_work_pose=place, yellow_work_zone=zone, table_2_drop_targets=_drops(), simulation_world=_world(), loaded_transport_safety_contract=_transport_safety())


def test_plan_requires_two_poses_explicit_zone_and_scene_bound_full_sweep():
    plan = _plan()
    assert plan["state"] == "simulation_plan_ready"
    assert not plan["physical_execution_eligible"] and plan["simulation_execution_eligible"]
    reposition = next(step for step in plan["steps"] if step["phase"] == "reposition_with_payload_to_table_2")
    assert reposition["kind"] == "loaded_reposition_intent"
    assert reposition["start_pose"]["pose_id"] == "pickup_work_pose"
    assert reposition["target_pose"]["pose_id"] == "place_work_pose"
    assert reposition["loaded_translation_m"] > .26
    assert reposition["max_abs_linear_mps"] == .05
    assert reposition["max_abs_angular_z_rad_s"] == .10
    assert reposition["target_xy_tolerance_m"] == .02
    assert reposition["zero_velocity_confirmations_required"] == 5
    assert reposition["minimum_transport_timeout_sec"] > 22.
    assert reposition["requires"] == ["transport_safe_pose_confirmed", "both_grasps_confirmed", "full_transport_collision_sweep_validated", "tf_available", "odom_available"]
    assert plan["translation_commands_permitted_only_in_phase"] == "reposition_with_payload_to_table_2"
    assert all(step["commands_emitted"] is False for step in plan["steps"])


def test_full_replay_completes_with_no_commands_emitted():
    trace = run_dual_pen_transport_simulation(**_run_kwargs())
    assert trace["terminal_state"] == "succeeded"
    assert not trace["commands_emitted"]
    assert len(trace["events"]) == 14


def test_missing_transform_or_physical_claim_or_single_pen_rejects_before_navigation():
    assert _plan(_payload(camera_to_base=None))["reason"] == "camera_to_base_transform_missing"
    assert _plan(_payload(camera_to_base={**_payload()["camera_to_base"], "physical_validated": True}))["reason"] == "simulation_transform_must_not_claim_physical_validation"
    assert _plan(_payload(candidates=[_candidate("one", [.55, .25, 1.20])]))["reason"] == "at_least_two_table_1_pen_candidates_required"


def test_zone_and_pose_validation_fail_closed():
    pickup, place, zone = _route()
    assert _plan(yellow_work_zone=None)["reason"] == "yellow_work_zone_required"
    assert _plan(pickup_work_pose=replace(pickup, x_m=zone.max_x_m + .001))["reason"] == "pickup_work_pose_outside_yellow_work_zone"
    assert _plan(place_work_pose=replace(place, y_m=zone.min_y_m - .001))["reason"] == "place_work_pose_outside_yellow_work_zone"
    assert _plan(yellow_work_zone=replace(zone, min_x_m=zone.max_x_m))["reason"] == "yellow_work_zone_bbox_invalid"


def test_loaded_timeout_derives_from_distance_yaw_and_settling():
    assert _plan(profile=replace(_profile(), loaded_transport_timeout_sec=22.0))["reason"] == "loaded_transport_timeout_insufficient"
    assert _plan(profile=replace(_profile(), loaded_transport_feedback_deadman_sec=.201))["reason"] == "loaded_transport_feedback_deadman_sec_exceeds_0_20"


def test_safety_evidence_requires_full_scene_bound_transport_sweep_and_fresh_binding():
    assert _plan(loaded_transport_safety_contract={**_transport_safety(), "full_transport_collision_sweep_validated": False})["reason"] == "full_transport_collision_sweep_validated_required"
    assert _plan(loaded_transport_safety_contract={**_transport_safety(), "yellow_work_zone_id": "other"})["reason"] == "loaded_transport_yellow_work_zone_mismatch"
    assert _plan(loaded_transport_safety_contract={**_transport_safety(), "stamp_ns": NOW - 251_000_000})["reason"] == "loaded_transport_safety_contract_stale_or_invalid"
    assert _plan(simulation_world=replace(_world(), validated_at_ns=NOW - 251_000_000))["reason"] == "simulation_world_binding_stale_or_invalid"


def test_loaded_reposition_failure_injection_locks_manual_intervention():
    cases = [
        ({"path_within_yellow_work_zone": False}, "loaded_reposition_yellow_work_zone_exit"),
        ({"tf_available": False}, "loaded_reposition_tf_missing"),
        ({"odom_available": False}, "loaded_reposition_odom_missing"),
        ({"scene_binding_current": False}, "loaded_reposition_scene_binding_stale"),
        ({"transport_safe_pose_confirmed": False}, "transport_safe_pose_not_confirmed"),
        ({"both_grasps_confirmed": False}, "both_grasps_not_confirmed_for_transport"),
        ({"full_transport_collision_sweep_validated": False}, "full_transport_collision_sweep_not_validated"),
        ({"feedback_age_sec": .201}, "loaded_reposition_feedback_deadman_exceeded"),
        ({"feedback_age_sec": -.001}, "loaded_reposition_feedback_invalid"),
        ({"linear_x_mps": .051}, "loaded_reposition_linear_speed_exceeded"),
        ({"linear_x_mps": .04, "linear_y_mps": .04}, "loaded_reposition_linear_speed_exceeded"),
        ({"angular_z_rad_s": .101}, "loaded_reposition_angular_speed_exceeded"),
        ({"target_xy_error_m": .021}, "loaded_reposition_xy_tolerance_not_met"),
        ({"target_xy_error_m": -.001}, "loaded_reposition_feedback_invalid"),
        ({"yaw_error_rad": .03491}, "loaded_reposition_yaw_tolerance_not_met"),
        ({"zero_velocity_confirmations": 4}, "loaded_reposition_zero_confirmations_insufficient"),
        ({"feedback_age_sec": float("nan")}, "loaded_reposition_feedback_nonfinite"),
    ]
    for override, expected in cases:
        trace = run_dual_pen_transport_simulation(**_run_kwargs(), adapter=FakeDualPenTransportAdapter({"reposition_with_payload_to_table_2": override}))
        assert trace["failure_code"] == expected
        assert trace["terminal_state"] == "locked_manual_intervention"
        assert trace["failed_phase"] == "reposition_with_payload_to_table_2"


def test_downstream_failures_cancel_and_hold_lock_after_grasp():
    kwargs = _run_kwargs()
    trace = run_dual_pen_transport_simulation(**kwargs, adapter=FakeDualPenTransportAdapter({"right:approach": {"delay_sec": .2}}))
    assert trace["failure_code"] == "barrier_skew_exceeded"
    trace = run_dual_pen_transport_simulation(**kwargs, adapter=FakeDualPenTransportAdapter({"left:confirm_grasp": {"grasp_confirmed": False}}))
    assert trace["failure_code"] == "grasp_confirmation_missing"
    trace = run_dual_pen_transport_simulation(**kwargs, adapter=FakeDualPenTransportAdapter({"right:confirm_release": {"released_confirmed": False}}))
    assert trace["terminal_state"] == "locked_manual_intervention"
    trace = run_dual_pen_transport_simulation(**kwargs, cancel_at_phase="lift")
    assert trace["terminal_state"] == "cancelled" and trace["failed_phase"] == "lift"


def test_distinct_target_selection_translation_lock_and_navigation_failures_remain_fail_closed():
    plan = _plan()
    assert {plan["assignments"][side]["target_id"] for side in ("left", "right")} == {"pen-left", "pen-right"}
    assert plan["assignments"]["left"]["target_id"] != plan["assignments"]["right"]["target_id"]
    assert _plan(_payload(candidates=[_candidate("same", [.55, .25, 1.20]), _candidate("same", [.58, -.24, 1.20])]))["reason"] == "no_distinct_reachable_pen_pair"
    assert _plan(_payload(candidates=[_candidate("near-a", [.55, .25, 1.20]), _candidate("near-b", [.56, .24, 1.20])]))["reason"] == "no_distinct_reachable_pen_pair"

    kwargs = _run_kwargs()
    trace = run_dual_pen_transport_simulation(**kwargs, adapter=FakeDualPenTransportAdapter({"navigate_pickup": "timed_out"}))
    assert trace["failed_phase"] == "navigate_pickup" and len(trace["events"]) == 1
    trace = run_dual_pen_transport_simulation(**kwargs, adapter=FakeDualPenTransportAdapter({"lift": {"translation_dx_m": .0201}}))
    assert trace["failure_code"] == "translation_lock_x_drift_exceeded"
    assert trace["terminal_state"] == "locked_manual_intervention"
