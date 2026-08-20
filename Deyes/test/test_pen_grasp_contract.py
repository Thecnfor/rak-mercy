import numpy as np

from deyes_stereo.pen_grasp_contract import build_pen_candidate, build_pen_candidates, parse_pen_features


def _feature():
    pixels = [[u, 20] for u in range(20, 55)] + [[u, 21] for u in range(20, 55)] + [[u, 22] for u in range(20, 55)]
    return {"label": "pen", "id": "p1", "confidence": .9, "axis_complete": True, "mask_pixels_px": pixels, "axis_endpoints_px": [[20, 20], [54, 20]]}


def _plane():
    return {"plane_normal": [0, 0, 1], "plane_center_camera_m": [0, 0, .50]}


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


def test_two_pen_features_fail_closed_instead_of_selecting_one():
    second = {**_feature(), "id": "p2", "mask_pixels_px": [[u, 35] for u in range(20, 55)]}
    multiple = {"features": [_feature(), second]}
    try:
        parse_pen_features(multiple)
    except ValueError as exc:
        assert str(exc) == "ambiguous_multi_target"
    else:
        raise AssertionError("two targets must inhibit grasping")


def test_no_target_and_more_than_two_targets_fail_closed():
    for payload, reason in (({"features": []}, "waiting/no_target"), ({"features": [_feature(), {**_feature(), "id": "p2"}, {**_feature(), "id": "p3"}]}, "ambiguous_multi_target")):
        try:
            parse_pen_features(payload)
        except ValueError as exc:
            assert str(exc) == reason
        else:
            raise AssertionError("invalid target multiplicity must fail closed")


def test_edge_truncation_or_incomplete_axis_is_never_executable():
    depth = np.full((80, 80), .515, dtype=np.float32)
    edge = {**_feature(), "axis_endpoints_px": [[1, 20], [54, 20]]}
    result = build_pen_candidate(edge, depth, (100, 100, 40, 40), plane_payload=_plane(), rotation=np.eye(3), translation=np.zeros(3), trusted_for_grasp=True)
    assert result["reason"] == "edge_truncation"
    assert result["target_visibility"] == "edge_truncated"
    assert "grasp_point_base_m" not in result


def test_batch_wrapper_reports_multi_target_without_candidates():
    depth = np.full((80, 80), .50, dtype=np.float32)
    depth[20:23, 20:55] = .515
    depth[35:38, 20:55] = .515
    second = {**_feature(), "id": "p2", "mask_pixels_px": [[u, y] for y in range(35, 38) for u in range(20, 55)], "axis_endpoints_px": [[20, 36], [54, 36]]}
    try:
        build_pen_candidates({"features": [_feature(), second]}, depth, (100, 100, 40, 40), plane_payload=_plane(), rotation=None, translation=None, trusted_for_grasp=False)
    except ValueError as exc:
        assert str(exc) == "ambiguous_multi_target"
    else:
        raise AssertionError("batch wrapper must not select either target")
