"""Low-cost synthetic, ROS-free end-to-end co-grasp replay."""

import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "deyes_stereo"))

from deyes_stereo.dual_pen_cograsp_contract import (  # noqa: E402
    DualPenCograspSiteProfile,
    WorkspaceBounds,
    build_dual_pen_cograsp_plan,
)
from deyes_stereo.dual_pen_cograsp_simulation import (  # noqa: E402
    run_dual_pen_cograsp_simulation,
)
from deyes_stereo.ground_plane_contract import (  # noqa: E402
    validate_dynamic_plane_for_depth,
    validate_rectified_depth_pair,
)
from deyes_stereo.pen_feature_extractor import (  # noqa: E402
    RectifiedImage,
    extract_one_pen,
)
from deyes_stereo.pen_grasp_contract import (  # noqa: E402
    build_pen_candidate,
    build_pen_candidates,
    feature_matches_depth_stamp,
)


STAMP_NS = 12_000_000_345
FRAME = "left_camera_optical_frame"
WIDTH, HEIGHT = 160, 120
P = [100.0, 0.0, 80.0, 0.0, 0.0, 100.0, 60.0, 0.0, 0.0, 0.0, 1.0, 0.0]
INTRINSICS = (100.0, 100.0, 80.0, 60.0)


def _scene(stamp_ns: int = STAMP_NS, detections=None):
    gray = np.full((HEIGHT, WIDTH), 150, dtype=np.uint8)
    cv2.line(gray, (45, 75), (115, 45), 30, 5)
    detection = {
        "class_name": "pen", "confidence": 0.95, "target_id": "p1",
        "bbox_xyxy": [35, 35, 125, 85],
    }
    payload = {
        "stamp_sec": stamp_ns // 1_000_000_000,
        "stamp_nanosec": stamp_ns % 1_000_000_000,
        "frame_id": FRAME, "image_width": WIDTH, "image_height": HEIGHT,
        "detections": [detection] if detections is None else detections,
    }
    return gray, payload


def _plane(stamp_ns: int = STAMP_NS, *, degraded=False):
    return {
        "stamp_sec": stamp_ns // 1_000_000_000,
        "stamp_nanosec": stamp_ns % 1_000_000_000,
        "camera_frame": FRAME,
        "coordinate_contract": "dynamic_table_plane_camera_relative_only",
        "valid_for_table_removal": True, "degraded": degraded,
        "plane_normal": [0, 0, 1], "plane_center_camera_m": [0, 0, 0.5],
    }


def _replay(*, depth_value=0.515, trusted=True, plane=None, stamp_ns=STAMP_NS):
    gray, yolo = _scene(stamp_ns)
    feature, reason = extract_one_pen(RectifiedImage(stamp_ns, FRAME, WIDTH, HEIGHT, gray), yolo)
    assert feature is not None, reason
    feature = {**feature, "stamp_sec": stamp_ns // 1_000_000_000, "stamp_nanosec": stamp_ns % 1_000_000_000}
    depth = np.full((HEIGHT, WIDTH), 0.5, dtype=np.float32)
    if np.isfinite(depth_value):
        cv2.line(depth, (45, 75), (115, 45), float(depth_value), 5)
    candidate = build_pen_candidate(
        feature, depth, INTRINSICS, plane_payload=plane or _plane(stamp_ns),
        rotation=np.eye(3), translation=np.zeros(3), trusted_for_grasp=trusted,
    )
    candidate.update({"stamp_ns": stamp_ns, "source_stamp_ns": stamp_ns})
    return feature, depth, candidate


def _profile():
    bounds = WorkspaceBounds(-1, 1, -1, 1, 0, 1)
    return DualPenCograspSiteProfile(
        validated=True, left_workspace=bounds, right_workspace=bounds,
        lift_vector_base_unit=(0, 0, 1), max_contact_distance_m=0.40, hold_sec=0.05,
    )


def test_synthetic_replay_reaches_dual_hold_with_exact_contract_wiring():
    feature, depth, candidate = _replay()
    assert feature["stamp_sec"] * 1_000_000_000 + feature["stamp_nanosec"] == STAMP_NS
    assert feature["axis_complete"] and len(feature["mask_pixels_px"]) >= 12
    assert depth.dtype == np.float32 and depth.shape == (HEIGHT, WIDTH)
    assert feature_matches_depth_stamp(feature, STAMP_NS)
    assert candidate["valid"] and candidate["trusted_for_grasp"]
    assert candidate["target_frame"] == "base_link"
    envelope = {"candidate_count": 1, "candidates": [candidate], "valid": True, "trusted_for_grasp": True, "stamp_ns": STAMP_NS}
    plan = build_dual_pen_cograsp_plan(envelope, now_stamp_ns=STAMP_NS, profile=_profile())
    assert plan["state"] == "ready" and plan["commands_emitted"] is False
    assert plan["assignments"]["left"][1] > plan["assignments"]["right"][1]
    assert all(step.get("commands_emitted") is False for step in plan["steps"])
    trace = run_dual_pen_cograsp_simulation(envelope, now_stamp_ns=STAMP_NS, profile=_profile())
    assert trace["terminal_state"] == "succeeded" and trace["state"] == "holding_complete"
    assert trace["commands_emitted"] is False


@pytest.mark.parametrize("detections", [[], [{"class_name": "pen", "confidence": .9, "target_id": "a", "bbox_xyxy": [35,35,80,80]}, {"class_name": "pen", "confidence": .9, "target_id": "b", "bbox_xyxy": [80,35,125,80]}]])
def test_empty_or_multiple_yolo_detections_never_make_cograsp_candidate(detections):
    gray, payload = _scene(detections=detections)
    feature, reason = extract_one_pen(RectifiedImage(STAMP_NS, FRAME, WIDTH, HEIGHT, gray), payload)
    assert feature is None
    assert reason in {"waiting_for_one_pen", "ambiguous_multi_target"}


@pytest.mark.parametrize("kind", ["nan", "insufficient", "invalid_plane"])
def test_bad_depth_or_plane_withholds_candidate(kind):
    plane = _plane()
    if kind == "invalid_plane":
        plane = _plane(degraded=True)
    _, _, candidate = _replay(depth_value=float("nan") if kind == "nan" else (0.1 if kind == "insufficient" else 0.515), plane=plane)
    assert candidate["valid"] is False
    assert "grasp_point_base_m" not in candidate


def test_untrusted_extrinsics_withhold_base_candidate():
    _, _, candidate = _replay(trusted=False)
    assert candidate["valid"] is False and candidate["reason"] == "untrusted_extrinsics"
    assert "grasp_point_base_m" not in candidate and "target_frame" not in candidate


def test_one_nanosecond_stamp_skew_fails_each_relevant_contract():
    feature, depth, candidate = _replay()
    assert not feature_matches_depth_stamp(feature, STAMP_NS + 1)
    pair = validate_rectified_depth_pair(
        depth_stamp_ns=STAMP_NS, depth_frame_id=FRAME, depth_width=WIDTH,
        depth_height=HEIGHT, depth_encoding="32FC1", info_stamp_ns=STAMP_NS + 1,
        info_frame_id=FRAME, info_width=WIDTH, info_height=HEIGHT, projection=P,
    )
    assert not pair.valid and "depth_camera_info_stamp_mismatch" in pair.reasons
    assert not validate_dynamic_plane_for_depth(_plane(STAMP_NS + 1), depth_stamp_ns=STAMP_NS, depth_frame_id=FRAME).valid
    rejected = build_dual_pen_cograsp_plan({**candidate, "stamp_ns": STAMP_NS + 1}, now_stamp_ns=STAMP_NS, profile=_profile())
    assert rejected["state"] == "rejected" and rejected["reason"] == "candidate_stale_or_stamp_missing"


def test_pen_batch_contract_rejects_multi_target_without_co_grasp():
    feature, depth, _ = _replay()
    second = {**feature, "id": "p2", "mask_pixels_px": [[u, y + 2] for u, y in feature["mask_pixels_px"]]}
    with pytest.raises(ValueError, match="ambiguous_multi_target"):
        build_pen_candidates({"features": [feature, second]}, depth, INTRINSICS, plane_payload=_plane(), rotation=np.eye(3), translation=np.zeros(3), trusted_for_grasp=True)
