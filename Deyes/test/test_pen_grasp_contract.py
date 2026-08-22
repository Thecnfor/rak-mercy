import numpy as np

from deyes_stereo.pen_grasp_contract import build_pen_candidate, build_pen_candidates, feature_matches_depth_stamp, parse_pen_features, pen_feature_stamp_ns


def _feature():
    pixels = [[u, 20] for u in range(20, 55)] + [[u, 21] for u in range(20, 55)] + [[u, 22] for u in range(20, 55)]
    return {"label": "pen", "id": "p1", "confidence": .9, "axis_complete": True, "mask_pixels_px": pixels, "axis_endpoints_px": [[20, 20], [54, 20]]}


def _plane():
    return {"coordinate_contract": "dynamic_table_plane_camera_relative_only", "valid_for_table_removal": True, "degraded": False, "plane_normal": [0, 0, 1], "plane_center_camera_m": [0, 0, .50]}


def test_pen_candidate_removes_table_and_withholds_untrusted_base_point():
    depth = np.full((80, 80), .50, dtype=np.float32)
    depth[20:22, 20:55] = .515
    result = build_pen_candidate(_feature(), depth, (100, 100, 40, 40), plane_payload=_plane(), rotation=np.eye(3), translation=np.zeros(3), trusted_for_grasp=False)
    assert result["valid"] is False
    assert result["reason"] == "untrusted_extrinsics"
    assert "grasp_point_base_m" not in result
    assert result["table_removed_count"] > 0


def test_pen_candidate_requires_mask_endpoints_and_table_plane():
    depth = np.full((80, 80), .515, dtype=np.float32)
    feature = _feature()
    del feature["mask_pixels_px"]
    try:
        build_pen_candidate(feature, depth, (100, 100, 40, 40), plane_payload=_plane(), rotation=np.eye(3), translation=np.zeros(3), trusted_for_grasp=True)
    except ValueError as exc:
        assert "mask_pixels" in str(exc)
    else:
        raise AssertionError("mask contract must be mandatory")


def test_two_pen_features_are_parsed_as_distinct_batch_candidates():
    second = {**_feature(), "id": "p2", "mask_pixels_px": [[u, 35] for u in range(20, 55)]}
    multiple = {"features": [_feature(), second]}
    assert [item["id"] for item in parse_pen_features(multiple)] == ["p1", "p2"]


def test_no_target_and_duplicate_ids_fail_closed():
    for payload, reason in (({"features": []}, "waiting/no_target"), ({"features": [_feature(), {**_feature()}]}, "geometric_conflict_or_indistinguishable")):
        try:
            parse_pen_features(payload)
        except ValueError as exc:
            assert str(exc) == reason
        else:
            raise AssertionError("invalid pen feature identities must fail closed")


def test_edge_truncation_or_incomplete_axis_is_never_executable():
    depth = np.full((80, 80), .515, dtype=np.float32)
    edge = {**_feature(), "axis_endpoints_px": [[1, 20], [54, 20]]}
    result = build_pen_candidate(edge, depth, (100, 100, 40, 40), plane_payload=_plane(), rotation=np.eye(3), translation=np.zeros(3), trusted_for_grasp=True)
    assert result["reason"] == "edge_truncation"
    assert result["target_visibility"] == "edge_truncated"
    assert "grasp_point_base_m" not in result


def test_batch_wrapper_returns_one_candidate_per_distinct_pen():
    depth = np.full((80, 80), .50, dtype=np.float32)
    depth[20:23, 20:55] = .515
    depth[35:38, 20:55] = .515
    second = {**_feature(), "id": "p2", "mask_pixels_px": [[u, y] for y in range(35, 38) for u in range(20, 55)], "axis_endpoints_px": [[20, 36], [54, 36]]}
    result = build_pen_candidates({"features": [_feature(), second]}, depth, (100, 100, 40, 40), plane_payload=_plane(), rotation=None, translation=None, trusted_for_grasp=False)
    assert result["candidate_count"] == 2
    assert {candidate["target_id"] for candidate in result["candidates"]} == {"p1", "p2"}


def test_pen_feature_must_have_the_exact_depth_stamp():
    feature = {"stamp_sec": 12, "stamp_nanosec": 34}
    exact = 12_000_000_034
    assert feature_matches_depth_stamp(feature, exact)
    assert not feature_matches_depth_stamp(feature, exact + 1)
    assert not feature_matches_depth_stamp({"stamp_sec": 0, "stamp_nanosec": 0}, exact)
    assert pen_feature_stamp_ns(feature) == exact
    assert pen_feature_stamp_ns({"stamp_sec": "bad"}) == 0
