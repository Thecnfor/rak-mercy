"""Fail-closed simulation-only bridge from RGB pen features to two grasp candidates.

This module is intentionally separate from :mod:`pen_grasp_contract`.  It
reuses its pixel/depth geometry, but a result from here is evidence for the
``team_rak_finals_20260820`` Isaac scene only; it is never physical calibration
or permission to command a real robot.
"""

from __future__ import annotations

from dataclasses import dataclass
import re
from typing import Any, Mapping

import numpy as np

from .ground_plane_contract import rectified_intrinsics, validate_dynamic_plane_for_depth, validate_rectified_depth_pair
from .pen_grasp_contract import build_pen_candidates, feature_matches_depth_stamp, pen_feature_stamp_ns


SIMULATION_SOURCE = "isaac_sim"
SIMULATION_CANDIDATE_SCHEMA = "sim_dual_pen_candidates/v1"
SIMULATION_GRASP_SCOPE = "simulation_only"


@dataclass(frozen=True)
class SimImageFrame:
    stamp_ns: int
    frame_id: str
    width: int
    height: int
    encoding: str
    depth_m: np.ndarray


@dataclass(frozen=True)
class SimCameraInfo:
    stamp_ns: int
    frame_id: str
    width: int
    height: int
    projection: tuple[float, ...]


@dataclass(frozen=True)
class SimTransform:
    """``base_link_T_Left_Camera`` queried at the image stamp."""

    stamp_ns: int
    parent_frame: str
    child_frame: str
    rotation: np.ndarray
    translation: np.ndarray


@dataclass(frozen=True)
class ExpectedSimulation:
    world_id: str
    scene_sha256: str
    seed: int
    camera_frame: str = "Left_Camera"
    pickup_table_id: str = "table_1"
    initial_scene_phase: str = "table_1_loaded_table_2_empty"


def _stamp_parts(stamp_ns: int) -> dict[str, int]:
    return {"stamp_sec": int(stamp_ns) // 1_000_000_000, "stamp_nanosec": int(stamp_ns) % 1_000_000_000}


def _rejected(reason: str, *, stamp_ns: int = 0, extra: Mapping[str, Any] | None = None) -> dict[str, Any]:
    result: dict[str, Any] = {
        "schema": SIMULATION_CANDIDATE_SCHEMA,
        **_stamp_parts(stamp_ns),
        "valid": False,
        "reason": reason,
        "candidate_count": 0,
        "candidates": [],
        "source": SIMULATION_SOURCE,
        "simulation_validated": False,
        "trusted_for_grasp": False,
        "trusted_for_grasp_scope": SIMULATION_GRASP_SCOPE,
        "physical_validated": False,
        "physical_execution_eligible": False,
    }
    if extra:
        result.update(dict(extra))
    return result


def _source_context(payload: Mapping[str, Any]) -> Mapping[str, Any] | None:
    context = payload.get("simulation")
    return context if isinstance(context, Mapping) else None


def _validate_simulation_context(payload: Mapping[str, Any], expected: ExpectedSimulation) -> str | None:
    if not expected.world_id or not re.fullmatch(r"[0-9a-f]{64}", expected.scene_sha256 or "") or int(expected.seed) < 0:
        return "simulation_binding_not_configured"
    context = _source_context(payload)
    if context is None:
        return "simulation_context_missing"
    if context.get("source") != SIMULATION_SOURCE:
        return "source_must_be_isaac_sim"
    if context.get("world_id") != expected.world_id:
        return "simulation_world_id_mismatch"
    if context.get("scene_sha256") != expected.scene_sha256:
        return "simulation_scene_sha256_mismatch"
    try:
        seed_matches = int(context.get("seed")) == int(expected.seed)
    except (TypeError, ValueError):
        seed_matches = False
    if not seed_matches:
        return "simulation_seed_mismatch"
    if context.get("initial_scene_phase") != expected.initial_scene_phase:
        return "simulation_scene_phase_mismatch"
    # A producer trying to upgrade simulation data to physical evidence is an
    # invalid input, not a value we silently overwrite.
    if (
        context.get("physical_validated") is not False
        or context.get("physical_execution_eligible") is not False
        or payload.get("physical_validated") not in (None, False)
        or payload.get("physical_execution_eligible") not in (None, False)
    ):
        return "physical_claim_forbidden_for_simulation"
    return None


def bind_generic_pen_features_to_simulation(
    payload: Mapping[str, Any], expected: ExpectedSimulation, *, assign_visible_pens_to_pickup_table: bool,
) -> tuple[dict[str, Any] | None, str | None]:
    """Bind a generic RGB feature batch to one explicitly selected scene.

    Detection producers cannot prove scene provenance, so only the launch-time
    boundary may add it.  This function has no permissive default: the initial
    scene phase and table assignment must both be explicitly selected.
    """
    if expected.initial_scene_phase != "table_1_loaded_table_2_empty":
        return None, "simulation_scene_phase_mismatch"
    if not assign_visible_pens_to_pickup_table:
        return None, "visible_pens_pickup_table_assignment_not_explicit"
    if not expected.world_id or not re.fullmatch(r"[0-9a-f]{64}", expected.scene_sha256 or "") or int(expected.seed) < 0:
        return None, "simulation_binding_not_configured"
    if payload.get("physical_validated") not in (None, False) or payload.get("physical_execution_eligible") not in (None, False):
        return None, "physical_claim_forbidden_for_simulation"
    features = payload.get("features")
    if not isinstance(features, list):
        return None, "pen_features_list_missing"
    bound_features: list[dict[str, Any]] = []
    for feature in features:
        if not isinstance(feature, Mapping):
            return None, "pen_feature_invalid"
        table_id = feature.get("source_table_id")
        if table_id not in (None, "", expected.pickup_table_id):
            return None, "visible_pen_table_assignment_conflict"
        marked = dict(feature)
        marked["source_table_id"] = expected.pickup_table_id
        bound_features.append(marked)
    bound = dict(payload)
    bound["features"] = bound_features
    bound["simulation"] = {
        "source": SIMULATION_SOURCE, "world_id": expected.world_id, "scene_sha256": expected.scene_sha256,
        "seed": expected.seed, "initial_scene_phase": expected.initial_scene_phase,
        "physical_validated": False, "physical_execution_eligible": False,
    }
    return bound, None


def _validate_plane(plane: Any, depth: SimImageFrame) -> str | None:
    contract = validate_dynamic_plane_for_depth(plane, depth_stamp_ns=depth.stamp_ns, depth_frame_id=depth.frame_id)
    if not contract.valid:
        return "table_plane_depth_contract:" + ",".join(contract.reasons)
    try:
        normal = np.asarray(plane["plane_normal"], dtype=np.float64).reshape(3)
        center = np.asarray(plane["plane_center_camera_m"], dtype=np.float64).reshape(3)
    except (KeyError, TypeError, ValueError):
        return "table_plane_invalid"
    if not np.all(np.isfinite(normal)) or not np.all(np.isfinite(center)) or float(np.linalg.norm(normal)) < 1e-6:
        return "table_plane_degenerate"
    return None


def _validate_transform(transform: SimTransform | None, depth: SimImageFrame, expected: ExpectedSimulation) -> str | None:
    if transform is None:
        return "base_link_T_Left_Camera_missing"
    if transform.stamp_ns != depth.stamp_ns:
        return "base_link_T_Left_Camera_stamp_mismatch"
    if transform.parent_frame != "base_link" or transform.child_frame != expected.camera_frame:
        return "base_link_T_Left_Camera_frame_mismatch"
    try:
        rotation = np.asarray(transform.rotation, dtype=np.float64).reshape(3, 3)
        translation = np.asarray(transform.translation, dtype=np.float64).reshape(3)
    except (TypeError, ValueError):
        return "base_link_T_Left_Camera_invalid"
    if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation)):
        return "base_link_T_Left_Camera_invalid"
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4) or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-4):
        return "base_link_T_Left_Camera_rotation_invalid"
    return None


def _feature_table_id(feature: Mapping[str, Any]) -> str:
    return str(feature.get("source_table_id") or "")


def build_simulation_dual_pen_candidates(
    feature_payload: Mapping[str, Any], depth: SimImageFrame, camera_info: SimCameraInfo, plane: Mapping[str, Any],
    transform: SimTransform | None, expected: ExpectedSimulation, *, min_depth_m: float = .20,
    max_depth_m: float = 2.50, min_plane_clearance_m: float = .004, edge_margin_px: int = 12,
) -> dict[str, Any]:
    """Build precisely two ``table_1`` candidates, or reject without a fallback.

    The function only accepts RGB-derived features, depth, camera model, plane,
    and TF with identical stamps/frames/sizes.  The resulting base-frame
    coordinates are valid for this Isaac world only.
    """
    stamp_ns = int(depth.stamp_ns)
    context_reason = _validate_simulation_context(feature_payload, expected)
    if context_reason:
        return _rejected(context_reason, stamp_ns=stamp_ns)
    if depth.encoding != "32FC1" or depth.frame_id != expected.camera_frame:
        return _rejected("depth_must_be_32FC1_in_Left_Camera", stamp_ns=stamp_ns)
    if depth.depth_m.shape[:2] != (depth.height, depth.width):
        return _rejected("depth_array_size_mismatch", stamp_ns=stamp_ns)
    try:
        feature_width, feature_height = int(feature_payload.get("image_width", 0)), int(feature_payload.get("image_height", 0))
    except (TypeError, ValueError):
        return _rejected("pen_features_metadata_invalid", stamp_ns=stamp_ns)
    if not feature_matches_depth_stamp(dict(feature_payload), stamp_ns):
        return _rejected("pen_features_depth_stamp_mismatch", stamp_ns=stamp_ns)
    if str(feature_payload.get("frame_id") or feature_payload.get("source_frame") or "") != depth.frame_id:
        return _rejected("pen_features_depth_frame_mismatch", stamp_ns=stamp_ns)
    if feature_width != depth.width or feature_height != depth.height:
        return _rejected("pen_features_depth_size_mismatch", stamp_ns=stamp_ns)
    camera_contract = validate_rectified_depth_pair(
        depth_stamp_ns=stamp_ns, depth_frame_id=depth.frame_id, depth_width=depth.width, depth_height=depth.height,
        depth_encoding=depth.encoding, info_stamp_ns=camera_info.stamp_ns, info_frame_id=camera_info.frame_id,
        info_width=camera_info.width, info_height=camera_info.height, projection=camera_info.projection,
    )
    if not camera_contract.valid:
        return _rejected("depth_camera_info_contract:" + ",".join(camera_contract.reasons), stamp_ns=stamp_ns)
    plane_reason = _validate_plane(plane, depth)
    if plane_reason:
        return _rejected(plane_reason, stamp_ns=stamp_ns)
    transform_reason = _validate_transform(transform, depth, expected)
    if transform_reason:
        return _rejected(transform_reason, stamp_ns=stamp_ns)
    assert transform is not None

    raw_features = feature_payload.get("features")
    if not isinstance(raw_features, list):
        return _rejected("pen_features_list_missing", stamp_ns=stamp_ns)
    table_features = [item for item in raw_features if isinstance(item, Mapping) and _feature_table_id(item) == expected.pickup_table_id]
    if len(table_features) < 2:
        return _rejected("two_table_1_candidates_required", stamp_ns=stamp_ns, extra={"detected_table_1_candidates": len(table_features)})
    # Preserve geometry semantics by giving the established builder a normal
    # payload.  It rejects duplicate ids and geometric conflicts itself.
    geometry_payload = dict(feature_payload)
    geometry_payload["features"] = [dict(item) for item in table_features]
    try:
        geometry = build_pen_candidates(
            geometry_payload, depth.depth_m, rectified_intrinsics(camera_info.projection), plane_payload=dict(plane),
            rotation=np.asarray(transform.rotation, dtype=np.float64), translation=np.asarray(transform.translation, dtype=np.float64),
            trusted_for_grasp=True, min_depth_m=min_depth_m, max_depth_m=max_depth_m,
            min_plane_clearance_m=min_plane_clearance_m, edge_margin_px=edge_margin_px,
        )
    except ValueError as exc:
        return _rejected(str(exc), stamp_ns=stamp_ns)
    source_table_by_id = {str(item.get("id") or item.get("target_id") or ""): _feature_table_id(item) for item in table_features}
    valid_candidates = [
        dict(item) for item in geometry.get("candidates", [])
        if item.get("valid") is True and source_table_by_id.get(str(item.get("target_id") or "")) == expected.pickup_table_id
    ]
    valid_candidates.sort(key=lambda item: (-float(item.get("confidence", 0.0)), str(item.get("target_id") or "")))
    selected = valid_candidates[:2]
    if len(selected) < 2:
        return _rejected("two_simulation_candidates_required", stamp_ns=stamp_ns, extra={"geometry_candidate_count": len(valid_candidates)})
    ids = [str(item.get("target_id") or "") for item in selected]
    if len(set(ids)) != 2 or not all(ids):
        return _rejected("simulation_target_ids_not_unique", stamp_ns=stamp_ns)
    for candidate in selected:
        candidate.update({
            "source_table_id": expected.pickup_table_id,
            "simulation_validated": True,
            "trusted_for_grasp": True,
            "trusted_for_grasp_scope": SIMULATION_GRASP_SCOPE,
            "physical_validated": False,
            "physical_execution_eligible": False,
        })
    return {
        "schema": SIMULATION_CANDIDATE_SCHEMA,
        **_stamp_parts(stamp_ns),
        "valid": True,
        "reason": "ok",
        "candidate_count": 2,
        "available_table_1_candidate_count": len(valid_candidates),
        "candidates": selected,
        "source": SIMULATION_SOURCE,
        "world_id": expected.world_id,
        "scene_sha256": expected.scene_sha256,
        "seed": expected.seed,
        "source_frame": depth.frame_id,
        "simulation_validated": True,
        "trusted_for_grasp": True,
        "trusted_for_grasp_scope": SIMULATION_GRASP_SCOPE,
        "physical_validated": False,
        "physical_execution_eligible": False,
    }
