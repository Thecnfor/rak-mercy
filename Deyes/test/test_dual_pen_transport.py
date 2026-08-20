"""Offline acceptance tests for the two-distinct-pen table transfer task."""

from dataclasses import replace
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "deyes_stereo"))

from deyes_stereo.dual_pen_cograsp_contract import WorkspaceBounds  # noqa: E402
from deyes_stereo.dual_pen_transport_contract import (  # noqa: E402
    DualPenTransportProfile,
    NavigationPose,
    SimulationWorldBinding,
    build_dual_pen_transport_plan,
)
from deyes_stereo.dual_pen_transport_simulation import (  # noqa: E402
    FakeDualPenTransportAdapter,
    run_dual_pen_transport_simulation,
)


NOW = 60_000_000_000


def _profile():
    left = WorkspaceBounds(0.1, 1.2, 0.0, 0.8, 0.7, 1.6)
    right = WorkspaceBounds(0.1, 1.2, -0.8, 0.0, 0.7, 1.6)
    return DualPenTransportProfile(
        validated_for_simulation=True, left_workspace=left, right_workspace=right,
        lift_vector_base_unit=(0.0, 0.0, 1.0), min_pen_separation_m=.10,
        min_tool_clearance_m=.08,
    )


def _route():
    return (
        NavigationPose("yellow_work_pose", "map", 1.6, 1.5, 0.20),
        NavigationPose("table_2_orientation_pose", "map", 1.6, 1.5, -1.20),
    )


def _world():
    return SimulationWorldBinding(
        simulation_execution_allowed=True, world_id="team_rak_finals_20260820",
        scene_path="/var/workspace/docker/isaac/scenes/team_rak_finals_20260820/outputs/team_rak_finals_20260820.usd",
        scene_sha256="11d59b9fff96304d263d2d6df4e4958b876b30fe0a1b03ca461098944a419cd6",
        random_seed=20260820, validated_at_ns=NOW,
    )


def _turn_safety():
    world = _world()
    return {
        "source": "isaac_sim_collision_sweep", "turn_safe_pose_confirmed": True,
        "collision_sweep_validated": True, "world_id": world.world_id,
        "scene_sha256": world.scene_sha256, "random_seed": world.random_seed,
        "stamp_ns": NOW,
    }


def _drops():
    return {
        "left": {"drop_id": "table_2_left_slot", "table_id": "table_2", "target_frame": "base_link", "position_base_m": [.45, .20, 1.20], "approach_normal_base_unit": [0., 0., 1.]},
        "right": {"drop_id": "table_2_right_slot", "table_id": "table_2", "target_frame": "base_link", "position_base_m": [.48, -.20, 1.20], "approach_normal_base_unit": [0., 0., 1.]},
    }


def _candidate(target_id, point, confidence=.95, **changes):
    value = {
        "target_id": target_id, "label": "pen", "source_table_id": "table_1",
        "valid": True, "trusted_for_grasp": True, "target_frame": "base_link",
        "stamp_ns": NOW, "confidence": confidence, "grasp_point_base_m": point,
        "axis_base_unit": [1., 0., 0.], "approach_normal_base_unit": [0., 0., 1.],
    }
    value.update(changes)
    return value


def _payload(candidates=None, **changes):
    value = {
        "camera_to_base": {
            "source": "isaac_sim_scene_tf", "simulation_validated": True,
            "physical_validated": False, "source_frame": "Left_camera",
            "target_frame": "base_link", "transform_id": "team_rak_tf", "stamp_ns": NOW,
        },
        "candidates": candidates or [
            _candidate("pen-left", [.55, .25, 1.20]),
            _candidate("pen-right", [.58, -.24, 1.20]),
            _candidate("low-confidence", [.70, .30, 1.20], confidence=.40),
        ],
    }
    value.update(changes)
    return value


def _plan(payload=None):
    yellow, table_2_orientation = _route()
    return build_dual_pen_transport_plan(
        payload or _payload(), now_stamp_ns=NOW, profile=_profile(),
        yellow_work_pose=yellow, table_2_orientation_pose=table_2_orientation,
        table_2_drop_targets=_drops(), simulation_world=_world(),
        turn_safety_contract=_turn_safety(),
    )


def _run_kwargs():
    yellow, table_2_orientation = _route()
    return dict(
        perception_payload=_payload(), now_stamp_ns=NOW, profile=_profile(),
        yellow_work_pose=yellow, table_2_orientation_pose=table_2_orientation,
        table_2_drop_targets=_drops(), simulation_world=_world(),
        turn_safety_contract=_turn_safety(),
    )


def test_simulation_plan_selects_two_different_pen_instances_and_is_never_physical():
    plan = _plan()
    assert plan["state"] == "simulation_plan_ready"
    assert plan["assignments"]["left"]["target_id"] != plan["assignments"]["right"]["target_id"]
    assert {plan["assignments"]["left"]["target_id"], plan["assignments"]["right"]["target_id"]} == {"pen-left", "pen-right"}
    assert not plan["physical_execution_eligible"] and plan["simulation_execution_eligible"]
    assert plan["simulation_execution_allowed"]
    assert plan["simulation_world"]["random_seed"] == 20260820
    assert [step["phase"] for step in plan["steps"]] == [
        "navigate_pickup", "verify_pickup_targets", "pregrasp", "approach", "contact", "close", "confirm_grasp", "lift", "rotate_in_place_to_table_2", "place_pregrasp", "place_approach", "release", "confirm_release", "retreat",
    ]
    assert [step["phase"] for step in plan["steps"] if step["kind"] == "navigation_gate"] == ["navigate_pickup"]
    assert all(step["base_translation_command_permitted"] is False for step in plan["steps"][1:])
    turn = next(step for step in plan["steps"] if step["phase"] == "rotate_in_place_to_table_2")
    assert turn["target_yaw_rad"] == _route()[1].yaw_rad
    assert turn["max_abs_angular_z_rad_s"] == .10
    assert turn["deadline_sec"] == 30.0
    assert turn["minimum_turn_timeout_sec"] > 15.0
    assert turn["zero_velocity_confirmations_required"] == 5
    assert turn["requires"] == ["turn_safe_pose_confirmed", "both_grasps_confirmed", "collision_sweep_validated"]


def test_full_replay_completes_with_no_commands_emitted():
    trace = run_dual_pen_transport_simulation(**_run_kwargs())
    assert trace["terminal_state"] == "succeeded"
    assert not trace["commands_emitted"]
    assert len(trace["events"]) == 14


def test_missing_transform_or_physical_claim_fails_before_navigation():
    missing = _plan(_payload(camera_to_base=None))
    assert missing["state"] == "rejected" and missing["reason"] == "camera_to_base_transform_missing"
    physical_claim = _plan(_payload(camera_to_base={**_payload()["camera_to_base"], "physical_validated": True}))
    assert physical_claim["reason"] == "simulation_transform_must_not_claim_physical_validation"


def test_single_pen_same_instance_or_unreachable_pair_rejects_fail_closed():
    plan = _plan(_payload(candidates=[_candidate("one", [.55, .25, 1.20])]))
    assert plan["reason"] == "at_least_two_table_1_pen_candidates_required"
    plan = _plan(_payload(candidates=[_candidate("same", [.55, .25, 1.20]), _candidate("same", [.58, -.24, 1.20])]))
    assert plan["reason"] == "no_distinct_reachable_pen_pair"
    plan = _plan(_payload(candidates=[_candidate("near-a", [.55, .25, 1.20]), _candidate("near-b", [.56, .24, 1.20])]))
    assert plan["reason"] == "no_distinct_reachable_pen_pair"


def test_skew_collision_target_loss_navigation_and_cancel_stop_downstream_work():
    kwargs = _run_kwargs()
    trace = run_dual_pen_transport_simulation(**kwargs, adapter=FakeDualPenTransportAdapter({"right:approach": {"delay_sec": .2}}))
    assert trace["failure_code"] == "barrier_skew_exceeded" and trace["failed_phase"] == "approach"
    trace = run_dual_pen_transport_simulation(**kwargs, adapter=FakeDualPenTransportAdapter({"left:contact": {"collision_free": False}}))
    assert trace["failure_code"] == "contact_collision_detected"
    trace = run_dual_pen_transport_simulation(**kwargs, adapter=FakeDualPenTransportAdapter({"verify_pickup_targets": {"feedback": "lost"}}))
    assert trace["failure_code"] == "verify_pickup_targets_feedback_missing"
    trace = run_dual_pen_transport_simulation(**kwargs, adapter=FakeDualPenTransportAdapter({"navigate_pickup": "timed_out"}))
    assert trace["failed_phase"] == "navigate_pickup" and len(trace["events"]) == 1
    trace = run_dual_pen_transport_simulation(**kwargs, cancel_at_phase="lift")
    assert trace["terminal_state"] == "cancelled" and trace["failed_phase"] == "lift"
    trace = run_dual_pen_transport_simulation(**kwargs, adapter=FakeDualPenTransportAdapter({"lift": {"translation_dx_m": .021}}))
    assert trace["failure_code"] == "translation_lock_x_drift_exceeded"
    assert trace["terminal_state"] == "locked_manual_intervention" and trace["failed_phase"] == "lift"


def test_grasp_and_release_feedback_are_explicit_interlocks():
    kwargs = _run_kwargs()
    trace = run_dual_pen_transport_simulation(**kwargs, adapter=FakeDualPenTransportAdapter({"left:confirm_grasp": {"grasp_confirmed": False}}))
    assert trace["failure_code"] == "grasp_confirmation_missing" and trace["failed_phase"] == "confirm_grasp"
    trace = run_dual_pen_transport_simulation(**kwargs, adapter=FakeDualPenTransportAdapter({"right:confirm_release": {"released_confirmed": False}}))
    assert trace["failure_code"] == "release_confirmation_missing" and trace["terminal_state"] == "locked_manual_intervention"


def test_rotation_is_yaw_only_and_all_motion_integrity_failures_lock_manual_intervention():
    cases = [
        ({"translation_dx_m": .0201}, "translation_lock_x_drift_exceeded"),
        ({"translation_dy_m": -.0201}, "translation_lock_y_drift_exceeded"),
        ({"linear_x_mps": .001}, "rotate_in_place_linear_velocity_nonzero"),
        ({"angular_z_rad_s": .1001}, "rotate_in_place_angular_speed_exceeded"),
        ({"yaw_error_rad": .03491}, "rotate_in_place_yaw_tolerance_not_met"),
        ({"zero_velocity_confirmations": 4}, "rotate_in_place_zero_confirmations_insufficient"),
        ({"tf_available": False}, "translation_lock_tf_missing"),
        ({"odom_available": False}, "translation_lock_odom_missing"),
        ({"yaw_error_rad": float("nan")}, "rotate_in_place_feedback_nonfinite"),
    ]
    for override, expected in cases:
        adapter = FakeDualPenTransportAdapter({"rotate_in_place_to_table_2": override})
        trace = run_dual_pen_transport_simulation(**_run_kwargs(), adapter=adapter)
        assert trace["failure_code"] == expected
        assert trace["terminal_state"] == "locked_manual_intervention"
        assert trace["failed_phase"] == "rotate_in_place_to_table_2"


def test_turn_pose_and_collision_contract_are_scene_bound_and_not_hard_coded():
    yellow, orientation = _route()
    shifted = NavigationPose("table_2_orientation_pose", "map", yellow.x_m + .021, yellow.y_m, orientation.yaw_rad)
    rejected = build_dual_pen_transport_plan(
        _payload(), now_stamp_ns=NOW, profile=_profile(), yellow_work_pose=yellow,
        table_2_orientation_pose=shifted, table_2_drop_targets=_drops(),
        simulation_world=_world(), turn_safety_contract=_turn_safety(),
    )
    assert rejected["reason"] == "table_2_orientation_pose_x_must_remain_at_yellow"
    unsafe = {**_turn_safety(), "collision_sweep_validated": False}
    rejected = build_dual_pen_transport_plan(
        _payload(), now_stamp_ns=NOW, profile=_profile(), yellow_work_pose=yellow,
        table_2_orientation_pose=orientation, table_2_drop_targets=_drops(),
        simulation_world=_world(), turn_safety_contract=unsafe,
    )
    assert rejected["reason"] == "collision_sweep_validated_required"


def test_turn_timeout_must_cover_shortest_yaw_at_limited_speed_plus_settling():
    yellow, orientation = _route()
    rejected = build_dual_pen_transport_plan(
        _payload(), now_stamp_ns=NOW, profile=replace(_profile(), turn_timeout_sec=15.0),
        yellow_work_pose=yellow, table_2_orientation_pose=orientation,
        table_2_drop_targets=_drops(), simulation_world=_world(),
        turn_safety_contract=_turn_safety(),
    )
    assert rejected["reason"] == "turn_timeout_insufficient"
