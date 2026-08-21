import numpy as np

from deyes_stereo.vision_grasp_candidate_contract import CAMERA_CANDIDATE_SCHEMA, build_camera_optical_pen_candidates, coordinate_chain_templates


STAMP = 9_000_000_123


def _feature(identifier="p1", y=20):
    return {"id": identifier, "label": "pen", "confidence": .9, "axis_complete": True,
            "mask_pixels_px": [[u, row] for row in range(y, y + 3) for u in range(20, 55)],
            "axis_endpoints_px": [[20, y + 1], [54, y + 1]]}


def _kwargs(*features, **changes):
    depth = np.full((80, 80), .50, np.float32)
    for feature in features:
        y = feature["axis_endpoints_px"][0][1] - 1
        depth[y:y + 4, 20:55] = .515
    values = dict(feature_payload={"stamp_sec": 9, "stamp_nanosec": 123, "frame_id": "left_camera_optical_frame", "image_width": 80, "image_height": 80, "features": list(features)}, depth_m=depth, depth_stamp_ns=STAMP, depth_frame_id="left_camera_optical_frame", depth_width=80, depth_height=80, depth_encoding="32FC1", camera_stamp_ns=STAMP, camera_frame_id="left_camera_optical_frame", camera_width=80, camera_height=80, projection=(100., 0., 40., 0., 0., 100., 40., 0., 0., 0., 1., 0.), plane_payload={"stamp_sec": 9, "stamp_nanosec": 123, "camera_frame": "left_camera_optical_frame", "coordinate_contract": "dynamic_table_plane_camera_relative_only", "valid_for_table_removal": True, "degraded": False, "plane_normal": [0., 0., 1.], "plane_center_camera_m": [0., 0., .50]}, source="replay")
    values.update(changes)
    return values


def test_single_and_two_pens_share_one_camera_optical_message_contract():
    one = build_camera_optical_pen_candidates(**_kwargs(_feature(), source="physical_replay"))
    two = build_camera_optical_pen_candidates(**_kwargs(_feature("p1", 20), _feature("p2", 40), source="isaac_sim"))
    for result, count in ((one, 1), (two, 2)):
        assert result["schema"] == CAMERA_CANDIDATE_SCHEMA and result["valid"]
        assert result["candidate_count"] == count and not result["physical_execution_eligible"]
        assert all(item["target_frame"] == "left_camera_optical_frame" and item["coordinate_chain_point"]["source_frame"] == "left_camera_optical_frame" and item["valid"] for item in result["candidates"])


def test_yolo_merged_box_feature_splits_are_consumed_as_independent_candidates():
    left, right = _feature("merged__split_00", 20), _feature("merged__split_01", 40)
    result = build_camera_optical_pen_candidates(**_kwargs(left, right))
    assert result["valid"] and {item["target_id"] for item in result["candidates"]} == {left["id"], right["id"]}


def test_edge_invalid_depth_and_join_mismatches_fail_closed():
    edge = _feature(); edge["axis_endpoints_px"][0][0] = 1
    assert build_camera_optical_pen_candidates(**_kwargs(edge))["reason"] == "candidate_invalid"
    bad_depth = _kwargs(_feature()); bad_depth["depth_m"][:] = np.nan
    assert build_camera_optical_pen_candidates(**bad_depth)["valid"] is False
    for key, value, reason in (("depth_stamp_ns", STAMP + 1, "pen_features_depth_stamp_mismatch"), ("depth_frame_id", "other_optical", "camera_optical_frame_mismatch"), ("depth_width", 79, "depth_array_size_mismatch")):
        values = _kwargs(_feature()); values[key] = value
        assert build_camera_optical_pen_candidates(**values)["reason"] == reason


def test_target_age_is_an_explicit_fail_closed_gate():
    values = _kwargs(_feature(), now_stamp_ns=STAMP + 500_000_001)
    assert build_camera_optical_pen_candidates(**values)["reason"] == "target_expired"


def test_camera_frame_alias_is_not_silently_accepted():
    values = _kwargs(_feature(), depth_frame_id="Left_Camera")
    assert build_camera_optical_pen_candidates(**values)["reason"] == "camera_optical_frame_mismatch"


def test_coordinate_chain_templates_keep_only_valid_camera_points():
    accepted = coordinate_chain_templates(build_camera_optical_pen_candidates(**_kwargs(_feature())))
    assert accepted["valid"] and accepted["requests"][0]["kind"] == "point"
    assert accepted["requests"][0]["source_frame"] == "left_camera_optical_frame"
    rejected = coordinate_chain_templates(build_camera_optical_pen_candidates(**_kwargs(_feature(), now_stamp_ns=STAMP + 1_000_000_000)))
    assert not rejected["valid"] and rejected["requests"] == [] and not rejected["physical_execution_eligible"]
    values = _kwargs(_feature(), now_stamp_ns=STAMP - 1)
    assert build_camera_optical_pen_candidates(**values)["reason"] == "target_expired"
