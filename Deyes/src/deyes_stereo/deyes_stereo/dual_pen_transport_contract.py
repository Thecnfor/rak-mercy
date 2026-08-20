"""Offline-only plan for carrying two *different* pens between two tables.

The finals task is not a co-grasp: each arm owns one pen instance.  This
module deliberately contains neither ROS nor Isaac imports and only returns
motion intents.  It is usable by the deterministic simulator, but cannot
authorize physical robot motion.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import permutations
from math import atan2, cos, isfinite, sin, sqrt
from typing import Any

from .dual_pen_cograsp_contract import WorkspaceBounds
from .motion_adapter_contract import required_motion_adapter_contract


PLAN_SCHEMA = "dual_pen_transport_plan/v1"
SIMULATION_SOURCE = "isaac_sim_scene_tf"
SIDES = ("left", "right")


@dataclass(frozen=True)
class NavigationPose:
    """A site-measured base navigation pose.  No coordinates are inferred."""

    pose_id: str
    frame_id: str
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class SimulationWorldBinding:
    """Evidence binding a plan to one generated Isaac scene instance."""

    simulation_execution_allowed: bool = False
    world_id: str = ""
    scene_path: str = ""
    scene_sha256: str = ""
    random_seed: int = 0
    validated_at_ns: int = 0


@dataclass(frozen=True)
class DualPenTransportProfile:
    """Simulation site limits; defaults intentionally reject every plan."""

    validated_for_simulation: bool = False
    left_workspace: WorkspaceBounds | None = None
    right_workspace: WorkspaceBounds | None = None
    min_confidence: float = 0.80
    max_candidate_age_ns: int = 250_000_000
    min_pen_separation_m: float = 0.10
    min_tool_clearance_m: float = 0.08
    pregrasp_offset_m: float = 0.08
    approach_offset_m: float = 0.025
    lift_distance_m: float = 0.10
    retreat_offset_m: float = 0.12
    phase_timeout_sec: float = 5.0
    navigation_timeout_sec: float = 45.0
    turn_timeout_sec: float = 30.0
    turn_settling_margin_sec: float = 2.0
    max_barrier_skew_sec: float = 0.10
    translation_x_tolerance_m: float = 0.02
    translation_y_tolerance_m: float = 0.02
    max_turn_angular_z_rad_s: float = 0.10
    turn_yaw_tolerance_rad: float = 0.03490658503988659
    turn_zero_confirmation_count: int = 5
    lift_vector_base_unit: tuple[float, float, float] | None = None


def _reject(reason: str, *, reasons: list[str] | None = None) -> dict[str, Any]:
    return {
        "schema": PLAN_SCHEMA,
        "state": "rejected",
        "reason": reason,
        "reasons": reasons or [reason],
        "commands_emitted": False,
        "physical_execution_eligible": False,
        "simulation_execution_eligible": False,
    }


def _point(value: Any, field: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{field}_must_be_three_finite_values")
    try:
        result = tuple(float(v) for v in value)
    except (TypeError, ValueError):
        raise ValueError(f"{field}_must_be_three_finite_values") from None
    if not all(isfinite(v) for v in result):
        raise ValueError(f"{field}_must_be_three_finite_values")
    return result  # type: ignore[return-value]


def _unit(value: Any, field: str) -> tuple[float, float, float]:
    result = _point(value, field)
    size = sqrt(sum(v * v for v in result))
    if abs(size - 1.0) > 1e-3:
        raise ValueError(f"{field}_must_be_unit")
    return tuple(v / size for v in result)  # type: ignore[return-value]


def _distance(first: tuple[float, float, float], second: tuple[float, float, float]) -> float:
    return sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def _offset(point: tuple[float, float, float], direction: tuple[float, float, float], amount: float) -> tuple[float, float, float]:
    return tuple(v + d * amount for v, d in zip(point, direction))  # type: ignore[return-value]


def _stamp_ns(payload: dict[str, Any]) -> int:
    if "stamp_ns" in payload:
        try:
            return int(payload["stamp_ns"])
        except (TypeError, ValueError):
            return 0
    try:
        return int(payload.get("stamp_sec", 0)) * 1_000_000_000 + int(payload.get("stamp_nanosec", 0))
    except (TypeError, ValueError):
        return 0


def validate_sim_camera_to_base_transform(evidence: Any, *, now_stamp_ns: int, max_age_ns: int) -> dict[str, Any]:
    """Accept a current Isaac camera transform, but never label it physical.

    The distinct flags are intentional: simulation coordinates are useful for
    testing the complete chain tonight, whereas a physical hand-eye result is
    still required before using the same chain on Mercury X1.
    """
    if not isinstance(evidence, dict):
        return {"valid": False, "reason": "camera_to_base_transform_missing", "reasons": ["camera_to_base_transform_missing"]}
    reasons: list[str] = []
    if evidence.get("source") != SIMULATION_SOURCE:
        reasons.append("transform_source_must_be_isaac_sim_scene_tf")
    if evidence.get("simulation_validated") is not True:
        reasons.append("simulation_transform_not_validated")
    if evidence.get("physical_validated") is not False:
        reasons.append("simulation_transform_must_not_claim_physical_validation")
    if evidence.get("source_frame") not in {"Left_camera", "left_camera_optical_frame"}:
        reasons.append("simulation_camera_frame_unexpected")
    if evidence.get("target_frame") != "base_link":
        reasons.append("transform_target_frame_must_be_base_link")
    if not str(evidence.get("transform_id") or "").strip():
        reasons.append("transform_id_missing")
    stamp = _stamp_ns(evidence)
    if stamp <= 0 or stamp > now_stamp_ns or now_stamp_ns - stamp > max_age_ns:
        reasons.append("camera_to_base_transform_stale_or_invalid")
    return {
        "valid": not reasons,
        "reason": "ok" if not reasons else reasons[0],
        "reasons": reasons,
        "transform_id": str(evidence.get("transform_id") or ""),
        "physical_validated": False,
        "simulation_only": True,
    }


def validate_simulation_world_binding(binding: SimulationWorldBinding | None, *, now_stamp_ns: int, max_age_ns: int) -> dict[str, Any]:
    """Require an explicit world/scene/seed binding before simulator execution.

    This gate is deliberately independent from the existing perception bridge:
    it permits only the future Isaac adapter to consume a plan.  It does not
    change the physical-calibration or physical-command prohibition.
    """
    if binding is None:
        return {"valid": False, "reason": "simulation_world_binding_missing", "physical_validated": False}
    reasons: list[str] = []
    if binding.simulation_execution_allowed is not True:
        reasons.append("simulation_execution_not_explicitly_allowed")
    if not binding.world_id.strip():
        reasons.append("simulation_world_id_missing")
    if not binding.scene_path.strip() or not binding.scene_path.endswith((".usd", ".usda")):
        reasons.append("simulation_scene_path_invalid")
    digest = binding.scene_sha256.strip().lower()
    if len(digest) != 64 or any(char not in "0123456789abcdef" for char in digest):
        reasons.append("simulation_scene_sha256_invalid")
    if binding.random_seed < 0:
        reasons.append("simulation_random_seed_invalid")
    if binding.validated_at_ns <= 0 or binding.validated_at_ns > now_stamp_ns or now_stamp_ns - binding.validated_at_ns > max_age_ns:
        reasons.append("simulation_world_binding_stale_or_invalid")
    return {
        "valid": not reasons,
        "reason": "ok" if not reasons else reasons[0],
        "reasons": reasons,
        "world_id": binding.world_id,
        "scene_path": binding.scene_path,
        "scene_sha256": digest,
        "random_seed": binding.random_seed,
        "simulation_execution_allowed": not reasons,
        "physical_validated": False,
    }


def _validate_pose(pose: NavigationPose, expected_id: str) -> str | None:
    if pose.pose_id != expected_id:
        return f"navigation_pose_id_must_be_{expected_id}"
    if pose.frame_id != "map":
        return "navigation_pose_frame_must_be_map"
    if not all(isfinite(float(v)) for v in (pose.x_m, pose.y_m, pose.yaw_rad)):
        return "navigation_pose_must_be_finite"
    return None


def _shortest_yaw_delta(start_rad: float, target_rad: float) -> float:
    return atan2(sin(target_rad - start_rad), cos(target_rad - start_rad))


def _validate_turn_contract(evidence: Any, world: dict[str, Any], *, now_stamp_ns: int, max_age_ns: int) -> dict[str, Any]:
    if not isinstance(evidence, dict):
        return {"valid": False, "reason": "turn_safety_contract_missing", "reasons": ["turn_safety_contract_missing"]}
    reasons: list[str] = []
    for field in ("turn_safe_pose_confirmed", "collision_sweep_validated"):
        if evidence.get(field) is not True:
            reasons.append(f"{field}_required")
    if evidence.get("source") != "isaac_sim_collision_sweep":
        reasons.append("turn_safety_source_invalid")
    if evidence.get("world_id") != world.get("world_id"):
        reasons.append("turn_safety_world_id_mismatch")
    if str(evidence.get("scene_sha256") or "").lower() != world.get("scene_sha256"):
        reasons.append("turn_safety_scene_sha256_mismatch")
    try:
        seed = int(evidence.get("random_seed"))
    except (TypeError, ValueError):
        seed = -1
    if seed != world.get("random_seed"):
        reasons.append("turn_safety_random_seed_mismatch")
    stamp = _stamp_ns(evidence)
    if stamp <= 0 or stamp > now_stamp_ns or now_stamp_ns - stamp > max_age_ns:
        reasons.append("turn_safety_contract_stale_or_invalid")
    return {
        "valid": not reasons,
        "reason": "ok" if not reasons else reasons[0],
        "reasons": reasons,
        "turn_safe_pose_confirmed": evidence.get("turn_safe_pose_confirmed") is True,
        "collision_sweep_validated": evidence.get("collision_sweep_validated") is True,
        "physical_validated": False,
    }


def _profile_failure(profile: DualPenTransportProfile) -> str | None:
    if not profile.validated_for_simulation:
        return "simulation_site_profile_not_validated"
    if profile.left_workspace is None or profile.right_workspace is None:
        return "dual_arm_workspace_bounds_missing"
    for value in (
        profile.min_confidence, profile.min_pen_separation_m, profile.min_tool_clearance_m,
        profile.pregrasp_offset_m, profile.approach_offset_m, profile.lift_distance_m,
        profile.retreat_offset_m, profile.phase_timeout_sec, profile.navigation_timeout_sec,
        profile.turn_timeout_sec, profile.turn_settling_margin_sec,
        profile.max_barrier_skew_sec, profile.translation_x_tolerance_m,
        profile.translation_y_tolerance_m, profile.max_turn_angular_z_rad_s,
        profile.turn_yaw_tolerance_rad,
    ):
        if not isfinite(float(value)) or float(value) < 0.0:
            return "simulation_site_profile_invalid"
    if not 0.0 <= profile.min_confidence <= 1.0 or profile.max_candidate_age_ns <= 0:
        return "simulation_site_profile_invalid"
    if not profile.pregrasp_offset_m > profile.approach_offset_m > 0.0:
        return "simulation_site_profile_invalid"
    if profile.translation_x_tolerance_m <= 0.0 or profile.translation_y_tolerance_m <= 0.0:
        return "simulation_site_profile_invalid"
    if profile.max_turn_angular_z_rad_s <= 0.0 or profile.turn_yaw_tolerance_rad <= 0.0:
        return "simulation_site_profile_invalid"
    if profile.max_turn_angular_z_rad_s > 0.10:
        return "max_turn_angular_z_rad_s_exceeds_0_10"
    if profile.turn_yaw_tolerance_rad > 0.03490658503988659:
        return "turn_yaw_tolerance_exceeds_two_degrees"
    if profile.turn_zero_confirmation_count != 5:
        return "turn_zero_confirmation_count_must_be_five"
    if profile.lift_vector_base_unit is None:
        return "lift_vector_base_unit_missing"
    try:
        _unit(profile.lift_vector_base_unit, "lift_vector_base_unit")
    except ValueError:
        return "lift_vector_base_unit_invalid"
    return None


def _candidate_intent(candidate: Any, side: str, profile: DualPenTransportProfile, now_stamp_ns: int) -> tuple[dict[str, Any] | None, str | None]:
    if not isinstance(candidate, dict):
        return None, "candidate_not_mapping"
    target_id = str(candidate.get("target_id") or candidate.get("pen_id") or "").strip()
    if not target_id:
        return None, "pen_instance_id_missing"
    if candidate.get("valid") is not True or candidate.get("trusted_for_grasp") is not True:
        return None, "pen_candidate_not_trusted"
    if str(candidate.get("label") or "pen") != "pen":
        return None, "candidate_label_must_be_pen"
    if str(candidate.get("source_table_id") or "") != "table_1":
        return None, "candidate_must_originate_from_table_1"
    if candidate.get("target_frame") != "base_link":
        return None, "candidate_frame_must_be_base_link"
    stamp = _stamp_ns(candidate)
    if stamp <= 0 or stamp > now_stamp_ns or now_stamp_ns - stamp > profile.max_candidate_age_ns:
        return None, "candidate_stale_or_stamp_missing"
    try:
        confidence = float(candidate.get("confidence", 0.0))
        point = _point(candidate.get("grasp_point_base_m"), "grasp_point_base_m")
        normal = _unit(candidate.get("approach_normal_base_unit"), "approach_normal_base_unit")
        axis = _unit(candidate.get("axis_base_unit"), "axis_base_unit")
    except (TypeError, ValueError) as exc:
        return None, str(exc)
    if confidence < profile.min_confidence or not isfinite(confidence):
        return None, "candidate_confidence_below_limit"
    workspace = profile.left_workspace if side == "left" else profile.right_workspace
    assert workspace is not None
    lift = _unit(profile.lift_vector_base_unit, "lift_vector_base_unit")
    poses = {
        "pregrasp": _offset(point, normal, profile.pregrasp_offset_m),
        "approach": _offset(point, normal, profile.approach_offset_m),
        "contact": point,
        "lift": _offset(point, lift, profile.lift_distance_m),
    }
    # The table-two pose is intentionally supplied by the scene/placement
    # adapter later.  Reusing these relative tool intents prevents fake world
    # coordinates from becoming a physical placement claim.
    if not all(workspace.contains(value) for value in poses.values()):
        return None, f"{side}_workspace_out_of_bounds"
    return {
        "target_id": target_id,
        "confidence": confidence,
        "source_stamp_ns": stamp,
        "grasp_point_base_m": list(point),
        "approach_normal_base_unit": list(normal),
        "axis_base_unit": list(axis),
        "poses": {name: list(value) for name, value in poses.items()},
    }, None


def _drop_intent(value: Any, side: str, profile: DualPenTransportProfile) -> tuple[dict[str, Any] | None, str | None]:
    """Validate a scene-defined table-two target; it is never inferred."""
    if not isinstance(value, dict):
        return None, "table_2_drop_target_missing"
    if value.get("target_frame") != "base_link" or str(value.get("table_id") or "") != "table_2":
        return None, "table_2_drop_target_frame_or_table_invalid"
    drop_id = str(value.get("drop_id") or "").strip()
    if not drop_id:
        return None, "table_2_drop_id_missing"
    try:
        point = _point(value.get("position_base_m"), "table_2_position_base_m")
        normal = _unit(value.get("approach_normal_base_unit"), "table_2_approach_normal_base_unit")
    except ValueError as exc:
        return None, str(exc)
    workspace = profile.left_workspace if side == "left" else profile.right_workspace
    assert workspace is not None
    poses = {
        "place_pregrasp": _offset(point, normal, profile.pregrasp_offset_m),
        "place_approach": _offset(point, normal, profile.approach_offset_m),
        "release": point,
        "retreat": _offset(point, normal, profile.retreat_offset_m),
    }
    if not all(workspace.contains(pose) for pose in poses.values()):
        return None, f"{side}_table_2_workspace_out_of_bounds"
    return {"drop_id": drop_id, "table_id": "table_2", "poses": {name: list(point) for name, point in poses.items()}}, None


def _select_distinct_pair(payload: dict[str, Any], profile: DualPenTransportProfile, now_stamp_ns: int) -> tuple[dict[str, Any] | None, str | None]:
    candidates = payload.get("candidates")
    if not isinstance(candidates, list) or len(candidates) < 2:
        return None, "at_least_two_table_1_pen_candidates_required"
    feasible: list[tuple[float, str, str, dict[str, Any], dict[str, Any]]] = []
    for left_raw, right_raw in permutations(candidates, 2):
        left, left_error = _candidate_intent(left_raw, "left", profile, now_stamp_ns)
        right, right_error = _candidate_intent(right_raw, "right", profile, now_stamp_ns)
        if left_error or right_error or left is None or right is None or left["target_id"] == right["target_id"]:
            continue
        if _distance(tuple(left["grasp_point_base_m"]), tuple(right["grasp_point_base_m"])) < profile.min_pen_separation_m:
            continue
        min_phase_distance = min(
            _distance(tuple(left["poses"][phase]), tuple(right["poses"][phase]))
            for phase in ("pregrasp", "approach", "contact", "lift")
        )
        if min_phase_distance < profile.min_tool_clearance_m:
            continue
        score = float(left["confidence"]) + float(right["confidence"])
        feasible.append((-score, str(left["target_id"]), str(right["target_id"]), left, right))
    if not feasible:
        return None, "no_distinct_reachable_pen_pair"
    feasible.sort(key=lambda item: item[:3])
    _, _, _, left, right = feasible[0]
    return {"left": left, "right": right}, None


def _dual_step(
    phase: str, name: str, assignments: dict[str, Any], timeout: float,
    profile: DualPenTransportProfile, *, placement: bool = False,
) -> dict[str, Any]:
    result = {
        "phase": phase,
        "name": name,
        "kind": "synchronized_dual_arm_intent",
        "barrier": True,
        "deadline_sec": timeout,
        "translation_lock_required": True,
        "translation_x_tolerance_m": profile.translation_x_tolerance_m,
        "translation_y_tolerance_m": profile.translation_y_tolerance_m,
        "base_translation_command_permitted": False,
        "base_rotation_command_permitted": False,
        "commands_emitted": False,
    }
    if phase not in {"close", "confirm_grasp", "release", "confirm_release"}:
        for side in SIDES:
            source = assignments[side]["drop_poses"] if placement else assignments[side]["poses"]
            result[side] = {
                "target_id": assignments[side]["target_id"],
                "frame_id": "base_link",
                "tool_point_m": source[phase],
            }
    return result


def build_dual_pen_transport_plan(
    perception_payload: dict[str, Any], *, now_stamp_ns: int,
    profile: DualPenTransportProfile = DualPenTransportProfile(),
    yellow_work_pose: NavigationPose | None = None,
    table_2_orientation_pose: NavigationPose | None = None,
    table_2_drop_targets: dict[str, Any] | None = None,
    simulation_world: SimulationWorldBinding | None = None,
    turn_safety_contract: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Build the complete table-1 to table-2 simulation intent sequence."""
    if not isinstance(perception_payload, dict):
        return _reject("perception_payload_invalid")
    profile_error = _profile_failure(profile)
    if profile_error:
        return _reject(profile_error)
    if yellow_work_pose is None or table_2_orientation_pose is None:
        return _reject("yellow_and_table_2_orientation_poses_required")
    error = _validate_pose(yellow_work_pose, "yellow_work_pose")
    if error:
        return _reject(error)
    error = _validate_pose(table_2_orientation_pose, "table_2_orientation_pose")
    if error:
        return _reject(error)
    if abs(table_2_orientation_pose.x_m - yellow_work_pose.x_m) > profile.translation_x_tolerance_m:
        return _reject("table_2_orientation_pose_x_must_remain_at_yellow")
    if abs(table_2_orientation_pose.y_m - yellow_work_pose.y_m) > profile.translation_y_tolerance_m:
        return _reject("table_2_orientation_pose_y_must_remain_at_yellow")
    shortest_yaw_delta = _shortest_yaw_delta(yellow_work_pose.yaw_rad, table_2_orientation_pose.yaw_rad)
    minimum_turn_timeout_sec = abs(shortest_yaw_delta) / profile.max_turn_angular_z_rad_s + profile.turn_settling_margin_sec
    if profile.turn_timeout_sec + 1e-9 < minimum_turn_timeout_sec:
        return _reject("turn_timeout_insufficient")
    transform = validate_sim_camera_to_base_transform(
        perception_payload.get("camera_to_base"), now_stamp_ns=now_stamp_ns,
        max_age_ns=profile.max_candidate_age_ns,
    )
    if not transform["valid"]:
        return _reject(transform["reason"], reasons=list(transform.get("reasons") or [transform["reason"]]))
    world = validate_simulation_world_binding(
        simulation_world, now_stamp_ns=now_stamp_ns, max_age_ns=profile.max_candidate_age_ns,
    )
    if not world["valid"]:
        return _reject(world["reason"], reasons=list(world.get("reasons") or [world["reason"]]))
    turn_safety = _validate_turn_contract(
        turn_safety_contract, world, now_stamp_ns=now_stamp_ns,
        max_age_ns=profile.max_candidate_age_ns,
    )
    if not turn_safety["valid"]:
        return _reject(turn_safety["reason"], reasons=list(turn_safety["reasons"]))
    assignments, selection_error = _select_distinct_pair(perception_payload, profile, now_stamp_ns)
    if selection_error or assignments is None:
        return _reject(selection_error or "candidate_selection_failed")
    if not isinstance(table_2_drop_targets, dict):
        return _reject("table_2_drop_targets_missing")
    for side in SIDES:
        drop, drop_error = _drop_intent(table_2_drop_targets.get(side), side, profile)
        if drop_error or drop is None:
            return _reject(drop_error or "table_2_drop_target_invalid")
        assignments[side]["drop_id"] = drop["drop_id"]
        assignments[side]["drop_poses"] = drop["poses"]
    steps = [
        {"phase": "navigate_pickup", "name": "navigate_red_start_to_yellow_work_pose", "kind": "navigation_gate", "goal": vars(yellow_work_pose), "deadline_sec": profile.navigation_timeout_sec, "translation_lock_required": False, "base_translation_command_permitted": True, "base_rotation_command_permitted": True, "commands_emitted": False},
        {"phase": "verify_pickup_targets", "name": "require_fresh_distinct_table_1_pens", "kind": "perception_gate", "target_ids": [assignments["left"]["target_id"], assignments["right"]["target_id"]], "translation_lock_required": True, "translation_x_tolerance_m": profile.translation_x_tolerance_m, "translation_y_tolerance_m": profile.translation_y_tolerance_m, "base_translation_command_permitted": False, "base_rotation_command_permitted": False, "commands_emitted": False},
        _dual_step("pregrasp", "both_arms_pregrasp", assignments, profile.phase_timeout_sec, profile),
        _dual_step("approach", "both_arms_approach", assignments, profile.phase_timeout_sec, profile),
        _dual_step("contact", "both_arms_contact_different_pens", assignments, profile.phase_timeout_sec, profile),
        _dual_step("close", "both_grippers_close", assignments, profile.phase_timeout_sec, profile),
        _dual_step("confirm_grasp", "require_both_grasp_feedback", assignments, profile.phase_timeout_sec, profile),
        _dual_step("lift", "both_arms_lift_to_turn_safe_pose", assignments, profile.phase_timeout_sec, profile),
        {
            "phase": "rotate_in_place_to_table_2", "name": "controlled_yaw_only_turn_at_yellow",
            "kind": "rotation_intent", "start_pose": vars(yellow_work_pose),
            "target_orientation_pose_evidence": vars(table_2_orientation_pose),
            "yaw_source": "scene_bound_explicit_pose_contract",
            "rotation_center_xy_m": [yellow_work_pose.x_m, yellow_work_pose.y_m],
            "target_yaw_rad": table_2_orientation_pose.yaw_rad,
            "shortest_yaw_delta_rad": shortest_yaw_delta,
            "requires": ["turn_safe_pose_confirmed", "both_grasps_confirmed", "collision_sweep_validated"],
            "turn_safe_pose_confirmed": turn_safety["turn_safe_pose_confirmed"],
            "collision_sweep_validated": turn_safety["collision_sweep_validated"],
            "translation_lock_required": True,
            "translation_x_tolerance_m": profile.translation_x_tolerance_m,
            "translation_y_tolerance_m": profile.translation_y_tolerance_m,
            "base_translation_command_permitted": False,
            "base_rotation_command_permitted": True,
            "required_linear_x_mps": 0.0, "required_linear_y_mps": 0.0,
            "max_abs_angular_z_rad_s": profile.max_turn_angular_z_rad_s,
            "yaw_tolerance_rad": profile.turn_yaw_tolerance_rad,
            "zero_velocity_confirmations_required": profile.turn_zero_confirmation_count,
            "minimum_turn_timeout_sec": minimum_turn_timeout_sec,
            "settling_margin_sec": profile.turn_settling_margin_sec,
            "deadline_sec": profile.turn_timeout_sec, "commands_emitted": False,
        },
        _dual_step("place_pregrasp", "both_arms_table_2_pregrasp", assignments, profile.phase_timeout_sec, profile, placement=True),
        _dual_step("place_approach", "both_arms_table_2_approach", assignments, profile.phase_timeout_sec, profile, placement=True),
        _dual_step("release", "both_grippers_open", assignments, profile.phase_timeout_sec, profile),
        _dual_step("confirm_release", "require_both_release_feedback", assignments, profile.phase_timeout_sec, profile),
        _dual_step("retreat", "both_arms_retreat", assignments, profile.phase_timeout_sec, profile, placement=True),
    ]
    return {
        "schema": PLAN_SCHEMA,
        "state": "simulation_plan_ready",
        "reason": "ok",
        "mode": "isaac_simulation_only",
        "commands_emitted": False,
        "physical_execution_eligible": False,
        "simulation_execution_eligible": True,
        "simulation_execution_allowed": True,
        "translation_lock_required_after_yellow": True,
        "translation_x_tolerance_m": profile.translation_x_tolerance_m,
        "translation_y_tolerance_m": profile.translation_y_tolerance_m,
        "translation_commands_permitted_after_yellow": False,
        "rotation_permitted_only_in_phase": "rotate_in_place_to_table_2",
        "physical_execution_block_reason": "simulation_camera_to_base_transform_cannot_be_used_on_physical_robot",
        "transform": transform,
        "simulation_world": world,
        "turn_safety": turn_safety,
        "assignments": assignments,
        "steps": steps,
        "required_motion_adapter_contract": required_motion_adapter_contract(),
    }
