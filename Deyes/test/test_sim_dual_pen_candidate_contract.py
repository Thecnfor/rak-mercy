import numpy as np

from deyes_stereo.sim_dual_pen_candidate_contract import (
    ExpectedSimulation,
    SimCameraInfo,
    SimImageFrame,
    SimTransform,
    bind_generic_pen_features_to_simulation,
    build_simulation_dual_pen_candidates,
)


STAMP = 17_000_000_123
EXPECTED = ExpectedSimulation(
    world_id="team_rak_finals_20260820",
    scene_sha256="11d59b9fff96304d263d2d6df4e4958b876b30fe0a1b03ca461098944a419cd6",
    seed=20260820,
)


def _feature(identifier: str, y: int) -> dict:
    return {
        "label": "pen", "id": identifier, "target_id": identifier, "confidence": .9,
        "axis_complete": True, "source_table_id": "table_1",
        "mask_pixels_px": [[u, row] for row in range(y, y + 3) for u in range(25, 58)],
        "axis_endpoints_px": [[25, y + 1], [57, y + 1]],
    }


def _payload(*features: dict, **context_changes: object) -> dict:
    context = {
        "source": "isaac_sim", "world_id": EXPECTED.world_id, "scene_sha256": EXPECTED.scene_sha256,
        "seed": EXPECTED.seed, "initial_scene_phase": "table_1_loaded_table_2_empty",
        "physical_validated": False, "physical_execution_eligible": False,
        **context_changes,
    }
    return {
        "stamp_sec": STAMP // 1_000_000_000, "stamp_nanosec": STAMP % 1_000_000_000,
        "frame_id": "Left_Camera", "image_width": 100, "image_height": 80,
        "simulation": context, "features": list(features),
    }


def _depth(**changes: object) -> SimImageFrame:
    values = np.full((80, 100), .50, dtype=np.float32)
    values[20:24, 25:58] = .515
    values[40:44, 25:58] = .515
    data = {"stamp_ns": STAMP, "frame_id": "Left_Camera", "width": 100, "height": 80, "encoding": "32FC1", "depth_m": values}
    data.update(changes)
    return SimImageFrame(**data)


def _camera(**changes: object) -> SimCameraInfo:
    data = {
        "stamp_ns": STAMP, "frame_id": "Left_Camera", "width": 100, "height": 80,
        "projection": (100.0, 0.0, 50.0, 0.0, 0.0, 100.0, 40.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    }
    data.update(changes)
    return SimCameraInfo(**data)


def _plane(**changes: object) -> dict:
    data = {
        "stamp_sec": STAMP // 1_000_000_000, "stamp_nanosec": STAMP % 1_000_000_000,
        "camera_frame": "Left_Camera", "coordinate_contract": "dynamic_table_plane_camera_relative_only",
        "valid_for_table_removal": True, "degraded": False, "plane_normal": [0.0, 0.0, 1.0],
        "plane_center_camera_m": [0.0, 0.0, .50],
    }
    data.update(changes)
    return data


def _transform(**changes: object) -> SimTransform:
    data = {"stamp_ns": STAMP, "parent_frame": "base_link", "child_frame": "Left_Camera", "rotation": np.eye(3), "translation": np.zeros(3)}
    data.update(changes)
    return SimTransform(**data)


def _build(payload: dict | None = None, **changes: object) -> dict:
    return build_simulation_dual_pen_candidates(
        payload or _payload(_feature("pen_a", 20), _feature("pen_b", 40)),
        _depth(**changes.pop("depth", {})), _camera(**changes.pop("camera", {})), _plane(**changes.pop("plane", {})),
        changes.pop("transform", _transform()), EXPECTED,
    )


def test_two_candidates_are_simulation_trusted_but_never_physical():
    result = _build()
    assert result["valid"] and result["candidate_count"] == 2
    assert result["simulation_validated"] and result["trusted_for_grasp"]
    assert not result["physical_validated"] and not result["physical_execution_eligible"]
    assert {candidate["source_table_id"] for candidate in result["candidates"]} == {"table_1"}
    assert len({candidate["target_id"] for candidate in result["candidates"]}) == 2
    assert all(candidate["target_frame"] == "base_link" for candidate in result["candidates"])


def test_old_world_and_physical_claim_fail_closed():
    assert _build(_payload(_feature("pen_a", 20), _feature("pen_b", 40), world_id="old_world"))["reason"] == "simulation_world_id_mismatch"
    assert _build(_payload(_feature("pen_a", 20), _feature("pen_b", 40), scene_sha256="old_scene"))["reason"] == "simulation_scene_sha256_mismatch"
    assert _build(_payload(_feature("pen_a", 20), _feature("pen_b", 40), physical_validated=True))["reason"] == "physical_claim_forbidden_for_simulation"


def test_default_or_wrong_scene_phase_binding_is_rejected():
    unbound = ExpectedSimulation(world_id="", scene_sha256="", seed=-1)
    result = build_simulation_dual_pen_candidates(_payload(_feature("pen_a", 20), _feature("pen_b", 40)), _depth(), _camera(), _plane(), _transform(), unbound)
    assert result["reason"] == "simulation_binding_not_configured"
    assert _build(_payload(_feature("pen_a", 20), _feature("pen_b", 40), initial_scene_phase="table_2_loaded"))["reason"] == "simulation_scene_phase_mismatch"


def test_generic_features_require_explicit_binding_then_gain_table_1_and_context():
    generic = _payload(_feature("pen_a", 20), _feature("pen_b", 40))
    generic.pop("simulation")
    rejected, reason = bind_generic_pen_features_to_simulation(generic, EXPECTED, assign_visible_pens_to_pickup_table=False)
    assert rejected is None and reason == "visible_pens_pickup_table_assignment_not_explicit"
    bound, reason = bind_generic_pen_features_to_simulation(generic, EXPECTED, assign_visible_pens_to_pickup_table=True)
    assert reason is None and bound is not None
    assert bound["simulation"]["scene_sha256"] == EXPECTED.scene_sha256
    assert {item["source_table_id"] for item in bound["features"]} == {"table_1"}
    assert _build(bound)["valid"]


def test_frame_size_tf_plane_and_insufficient_candidates_are_rejected():
    assert _build(depth={"frame_id": "sim_camera"})["reason"] == "depth_must_be_32FC1_in_Left_Camera"
    assert _build(camera={"width": 99})["reason"].startswith("depth_camera_info_contract:")
    assert _build(transform=None)["reason"] == "base_link_T_Left_Camera_missing"
    assert _build(plane={"plane_normal": [0.0, 0.0, 0.0]})["reason"] == "table_plane_degenerate"
    assert _build(_payload(_feature("pen_a", 20)))["reason"] == "two_table_1_candidates_required"


def test_feature_stamp_and_frame_mismatch_fail_closed():
    payload = _payload(_feature("pen_a", 20), _feature("pen_b", 40))
    payload["stamp_nanosec"] += 1
    assert _build(payload)["reason"] == "pen_features_depth_stamp_mismatch"
    payload = _payload(_feature("pen_a", 20), _feature("pen_b", 40))
    payload["frame_id"] = "sim_camera"
    assert _build(payload)["reason"] == "pen_features_depth_frame_mismatch"
