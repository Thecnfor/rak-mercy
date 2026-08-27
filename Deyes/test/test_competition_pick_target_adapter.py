from __future__ import annotations

import json
import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "deyes_stereo"))

from deyes_stereo.competition_pick_target_node import (  # noqa: E402
    ExactStampSnapshotJoiner,
    Snapshot,
    build_target_from_snapshot,
    load_venue_runtime,
)


STAMP = 8_000_000_123
FRAME = "left_camera_optical_frame"


def _profile() -> dict[str, object]:
    return {
        "schema": "competition_venue_profile/v1",
        "table_height_m": 0.650,
        "reference_table_height_m": 0.560,
        "reference_plane_distance_m": 0.559428925,
        "expected_plane_distance_m": 0.469428925,
        "touch_plane_z_m": 0.135,
        "orientation_deg": [179.99, -12.0, 0.0],
        "fallbacks": {
            "fixed_height": {"enabled": True, "reject_deviation_over_m": 0.025},
            "bbox_center": {"enabled": False, "edge_margin_px": 12},
            "fixed_xy": {"enabled": False, "xy_m": [0.400, 0.010]},
        },
    }


def _projector(*, usable: bool) -> dict[str, object]:
    # Camera origin in right-arm SDK is [0.4, 0.01, 0.5] and its +Z points down.
    forward = [
        [1.0, 0.0, 0.0, -0.4],
        [0.0, -1.0, 0.0, 0.01],
        [0.0, 0.0, -1.0, 0.5],
        [0.0, 0.0, 0.0, 1.0],
    ]
    inverse = [
        [1.0, 0.0, 0.0, 0.4],
        [0.0, -1.0, 0.0, 0.01],
        [0.0, 0.0, -1.0, 0.5],
        [0.0, 0.0, 0.0, 1.0],
    ]
    gates = {"reprojection_rms_px_lte_4": usable, "all_points_positive_camera_depth": True}
    return {
        "schema": "venue_touch_projector/v1",
        "coordinate_frame": "right_arm_sdk",
        "matrix_direction": "camera_from_right_arm_sdk",
        "publishes_tf": False,
        "is_base_link_hand_eye": False,
        "usable": usable,
        "camera_from_right_arm_sdk": {"matrix": forward},
        "right_arm_sdk_from_camera": {"matrix": inverse},
        "calibration_convex_hull_xy_m": [[0.35, -0.05], [0.45, -0.05], [0.45, 0.08], [0.35, 0.08]],
        "metrics": {
            "reprojection_rms_px": 3.0 if usable else 4.191726799699995,
            "reprojection_p95_px": 5.0,
            "loo_base_xy_rms_mm": 14.0,
            "loo_base_xy_p95_mm": 20.0,
        },
        "gates": gates,
    }


def _snapshot(*, encoding: str = "32FC1", detection_stamp: int = STAMP) -> Snapshot:
    detection = {
        "stamp_ns": detection_stamp,
        "frame_id": FRAME,
        "image_width": 100,
        "image_height": 80,
        "complete": True,
        "auto_grasp_permitted": True,
        "detections": [{"target_id": "pen-0", "bbox_xyxy": [40, 25, 60, 55]}],
    }
    features = {
        "stamp_ns": STAMP,
        "frame_id": FRAME,
        "image_width": 100,
        "image_height": 80,
        "features": [{"target_id": "pen-0", "axis_complete": True, "axis_endpoints_px": [[45, 40], [55, 40]]}],
    }
    camera = {
        "stamp_ns": STAMP,
        "depth_stamp_ns": STAMP,
        "frame_id": FRAME,
        "width": 100,
        "height": 80,
        "p": [100.0, 0.0, 50.0, 0.0, 0.0, 100.0, 40.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    }
    plane = {
        "stamp_ns": STAMP,
        "camera_frame": FRAME,
        "coordinate_contract": "dynamic_table_plane_camera_relative_only",
        "valid_for_table_removal": True,
        "degraded": False,
        "plane_distance_camera_m": 0.469428925,
        "residual_rms_m": 0.002,
    }
    return Snapshot(
        stamp_ns=STAMP,
        detection=detection,
        pen_features=features,
        depth_m=np.full((80, 100), 0.365, dtype=np.float32),
        depth_encoding=encoding,
        depth_frame_id=FRAME,
        camera_info=camera,
        ground_plane=plane,
    )


def _runtime(tmp_path: Path, *, usable: bool):
    profile_path = tmp_path / "competition_venue_65cm.yaml"
    projector_path = tmp_path / "venue_touch_projector.yaml"
    profile_path.write_text(yaml.safe_dump(_profile()), encoding="utf-8")
    projector_path.write_text(yaml.safe_dump(_projector(usable=usable)), encoding="utf-8")
    return load_venue_runtime(profile_path, projector_path)


def test_exact_joiner_never_combines_nearby_stamps() -> None:
    joiner = ExactStampSnapshotJoiner(capacity=8, max_age_ns=1_000_000_000)
    snapshot = _snapshot()
    values = {
        "detection": snapshot.detection,
        "pen_features": snapshot.pen_features,
        "depth": (snapshot.depth_m, snapshot.depth_encoding, snapshot.depth_frame_id),
        "camera_info": snapshot.camera_info,
        "ground_plane": snapshot.ground_plane,
    }
    assert joiner.add("detection", STAMP + 1, values["detection"], 1) is None
    for kind in ("pen_features", "depth", "camera_info", "ground_plane"):
        assert joiner.add(kind, STAMP, values[kind], 1) is None
    assert joiner.add("detection", STAMP, values["detection"], 1) is not None
    assert joiner.latched is True
    for kind in ("detection", "pen_features", "depth", "camera_info", "ground_plane"):
        assert joiner.add(kind, STAMP + 2, values[kind], 2) is None


def test_exact_joiner_does_not_rejuvenate_stale_partial_snapshot() -> None:
    joiner = ExactStampSnapshotJoiner(capacity=8, max_age_ns=100)
    snapshot = _snapshot()
    values = {
        "detection": snapshot.detection,
        "pen_features": snapshot.pen_features,
        "depth": (snapshot.depth_m, snapshot.depth_encoding, snapshot.depth_frame_id),
        "camera_info": snapshot.camera_info,
        "ground_plane": snapshot.ground_plane,
    }
    assert joiner.add("detection", STAMP, values["detection"], 0) is None
    for kind in ("pen_features", "depth", "camera_info", "ground_plane"):
        assert joiner.add(kind, STAMP, values[kind], 101) is None
    assert joiner.latched is False


def test_live_contract_uses_calibrated_projector_and_exact_65cm_subtraction(tmp_path: Path) -> None:
    runtime = _runtime(tmp_path, usable=True)
    assert runtime.expected_plane_distance_m == pytest.approx(0.559428925 - 0.090)
    result = build_target_from_snapshot(_snapshot(), runtime, allow_bbox_center=False, force_fixed_target=False)
    assert result["valid"] is True
    assert result["trusted_for_venue_execution"] is True
    assert result["selection_source"] == "axis_midpoint"
    assert result["right_arm_sdk_target_m"] == pytest.approx([0.4, 0.01, 0.135])


def test_real_unusable_pnp_fails_random_xy_but_explicit_force_is_marked_degraded(tmp_path: Path) -> None:
    runtime = _runtime(tmp_path, usable=False)
    rejected = build_target_from_snapshot(_snapshot(), runtime, allow_bbox_center=True, force_fixed_target=False)
    assert rejected["valid"] is False
    assert rejected["trusted_for_venue_execution"] is False
    assert "projector_not_usable_and_validated" in rejected["reason"]

    forced = build_target_from_snapshot(_snapshot(), runtime, allow_bbox_center=True, force_fixed_target=True)
    assert forced["valid"] is True
    assert forced["trusted_for_venue_execution"] is False
    assert forced["selection_source"] == "fixed_xy_fallback"
    assert forced["right_arm_sdk_target_m"] == pytest.approx([0.4, 0.01, 0.135])
    assert forced["pixel_uv"] == pytest.approx([50.0, 40.0])
    assert forced["degraded"] is True
    assert forced["execution_allowed"] is True
    assert "[400,10]mm" in forced["manual_action_required"]

    no_pixel = _snapshot()
    no_pixel.pen_features["features"] = []
    missing = build_target_from_snapshot(no_pixel, runtime, allow_bbox_center=True, force_fixed_target=True)
    assert missing["valid"] is False
    assert missing["reason"] == "fixed_target_requires_observed_single_pen_pixel"


@pytest.mark.parametrize(
    ("snapshot", "reason"),
    [
        (_snapshot(encoding="16UC1"), "depth_encoding_must_be_32FC1"),
        (_snapshot(detection_stamp=STAMP + 1), "exact_stamp_mismatch"),
    ],
)
def test_snapshot_sensor_contract_fails_closed(tmp_path: Path, snapshot: Snapshot, reason: str) -> None:
    result = build_target_from_snapshot(snapshot, _runtime(tmp_path, usable=True), allow_bbox_center=False, force_fixed_target=False)
    assert result["valid"] is False
    assert reason in result["reason"]


def test_profile_requires_measured_reference_and_expected_distance_direction(tmp_path: Path) -> None:
    projector_path = tmp_path / "projector.yaml"
    projector_path.write_text(yaml.safe_dump(_projector(usable=False)), encoding="utf-8")
    profile = _profile()
    profile["reference_plane_distance_m"] = None
    profile_path = tmp_path / "profile.yaml"
    profile_path.write_text(yaml.safe_dump(profile), encoding="utf-8")
    with pytest.raises(ValueError, match="reference_plane_distance_m"):
        load_venue_runtime(profile_path, projector_path)

    profile["reference_plane_distance_m"] = 0.559428925
    profile["expected_plane_distance_m"] = 0.649428925
    profile_path.write_text(yaml.safe_dump(profile), encoding="utf-8")
    with pytest.raises(ValueError, match="expected_plane_distance_m_must_equal_reference_minus_90mm"):
        load_venue_runtime(profile_path, projector_path)


def test_repository_pnp_evidence_remains_unusable_at_4_1917px(tmp_path: Path) -> None:
    profile_path = tmp_path / "profile.yaml"
    profile_path.write_text(yaml.safe_dump(_profile()), encoding="utf-8")
    projector_path = ROOT / "config/camera/venue_20260827_touch_projector.yaml"
    evidence = yaml.safe_load(projector_path.read_text(encoding="utf-8"))
    assert evidence["metrics"]["reprojection_rms_px"] == pytest.approx(4.191726799699995)
    assert evidence["thresholds"]["reprojection_rms_px_max"] == 4.0
    runtime = load_venue_runtime(profile_path, projector_path)
    assert runtime.projector.usable is False


def test_fixture_shape_is_json_serializable_for_offline_cli(tmp_path: Path) -> None:
    snapshot = _snapshot()
    fixture = snapshot.to_fixture()
    round_trip = json.loads(json.dumps(fixture))
    assert round_trip["schema"] == "competition_pick_target_fixture/v1"
    assert round_trip["depth"]["encoding"] == "32FC1"
    assert len(round_trip["depth"]["depth_m"]) == 80


def test_node_source_no_longer_publishes_waiting_placeholder() -> None:
    source = (ROOT / "src/deyes_stereo/deyes_stereo/competition_pick_target_node.py").read_text(encoding="utf-8")
    assert "waiting_for_exact_stamp_projector_adapter" not in source
    assert "build_competition_pick_target(" in source
    assert "ExactStampSnapshotJoiner" in source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in source
    assert "ReliabilityPolicy.RELIABLE" in source
    yolo_source = (ROOT / "src/deyes_stereo/deyes_stereo/yolo_detector_node.py").read_text(encoding="utf-8")
    assert '"complete": True' in yolo_source


@pytest.mark.parametrize("field", ["complete", "auto_grasp_permitted"])
def test_target_rejects_detection_when_required_completion_gate_is_missing(
    tmp_path: Path, field: str
) -> None:
    snapshot = _snapshot()
    del snapshot.detection[field]
    result = build_target_from_snapshot(
        snapshot, _runtime(tmp_path, usable=True),
        allow_bbox_center=False, force_fixed_target=False,
    )
    assert result["valid"] is False
    assert result["reason"] == "detection_not_complete_or_auto_grasp_permitted"


def test_snapshot_adapter_rejects_untrusted_random_xy_and_accepts_only_forced_marker() -> None:
    adapter_path = ROOT.parent / "tools/competition_target_snapshot_adapter.py"
    spec = importlib.util.spec_from_file_location("competition_snapshot_adapter", adapter_path)
    assert spec and spec.loader
    adapter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(adapter)
    random_target = {
        "schema": "competition_pick_target/v1",
        "valid": True,
        "trusted_for_venue_execution": False,
        "selection_source": "axis_midpoint",
        "right_arm_sdk_target_m": [0.39, 0.02, 0.135],
        "orientation_deg": [179.99, -12.0, 0.0],
        "pixel_uv": [50.0, 40.0],
        "commands_emitted": False,
    }
    assert adapter.validate_target(random_target)[2] == 3
    fixed = {
        **random_target,
        "selection_source": "fixed_xy_fallback",
        "right_arm_sdk_target_m": [0.4, 0.01, 0.135],
        "pixel_uv": [50.0, 40.0],
        "degraded": True,
        "force_fixed_target": True,
        "execution_allowed": True,
        "manual_action_required": "DEGRADED: place pen at [400,10]mm marker",
    }
    assert adapter.validate_target(fixed)[0] == fixed
    malformed_trusted = {
        **random_target,
        "trusted_for_venue_execution": True,
        "commands_emitted": True,
    }
    assert adapter.validate_target(malformed_trusted)[2] == 3


def test_snapshot_adapter_uses_reliable_transient_local_qos() -> None:
    source = (ROOT.parent / "tools/competition_target_snapshot_adapter.py").read_text(encoding="utf-8")
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in source
    assert "ReliabilityPolicy.RELIABLE" in source


def test_offline_fixture_cli_uses_same_contract_and_writes_one_target(tmp_path: Path) -> None:
    adapter_path = ROOT.parent / "tools/competition_target_snapshot_adapter.py"
    spec = importlib.util.spec_from_file_location("competition_fixture_cli", adapter_path)
    assert spec and spec.loader
    adapter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(adapter)
    fixture = tmp_path / "snapshot.json"
    profile = tmp_path / "profile.yaml"
    projector = tmp_path / "projector.yaml"
    output = tmp_path / "target.json"
    fixture.write_text(json.dumps(_snapshot().to_fixture()), encoding="utf-8")
    profile.write_text(yaml.safe_dump(_profile()), encoding="utf-8")
    projector.write_text(yaml.safe_dump(_projector(usable=True)), encoding="utf-8")
    code = adapter.main([
        "--fixture", str(fixture), "--venue-profile", str(profile),
        "--projector", str(projector), "--output", str(output),
    ])
    assert code == 0
    result = json.loads(output.read_text(encoding="utf-8"))
    assert result["schema"] == "competition_pick_target/v1"
    assert result["valid"] is True
    assert result["selection_source"] == "axis_midpoint"


def test_offline_fixed_fixture_requires_force_environment(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    adapter_path = ROOT.parent / "tools/competition_target_snapshot_adapter.py"
    spec = importlib.util.spec_from_file_location("competition_fixed_fixture_cli", adapter_path)
    assert spec and spec.loader
    adapter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(adapter)
    fixture = tmp_path / "snapshot.json"
    profile = tmp_path / "profile.yaml"
    projector = tmp_path / "projector.yaml"
    output = tmp_path / "target.json"
    fixture.write_text(json.dumps(_snapshot().to_fixture()), encoding="utf-8")
    profile.write_text(yaml.safe_dump(_profile()), encoding="utf-8")
    projector.write_text(yaml.safe_dump(_projector(usable=False)), encoding="utf-8")
    argv = [
        "--fixture", str(fixture), "--venue-profile", str(profile),
        "--projector", str(projector), "--output", str(output), "--force-fixed-target",
    ]
    monkeypatch.delenv("FORCE_FIXED_TARGET", raising=False)
    assert adapter.main(argv) == 4
    assert not output.exists()
    monkeypatch.setenv("FORCE_FIXED_TARGET", "1")
    assert adapter.main(argv) == 0
    assert json.loads(output.read_text(encoding="utf-8"))["degraded"] is True
