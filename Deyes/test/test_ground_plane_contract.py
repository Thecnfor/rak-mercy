import numpy as np

from deyes_stereo.ground_plane_contract import (
    fit_plane_ransac,
    normal_delta_deg,
    project_rectified_depth_pixels,
    validate_dynamic_plane_for_depth,
    validate_rectified_depth_pair,
)


def _projection():
    return [100.0, 0.0, 50.0, 0.0, 0.0, 200.0, 30.0, 0.0, 0.0, 0.0, 1.0, 0.0]


def test_rectified_pair_requires_exact_stamp_size_frame_and_32fc1():
    result = validate_rectified_depth_pair(depth_stamp_ns=10, depth_frame_id="left_camera_optical_frame", depth_width=640, depth_height=360, depth_encoding="32FC1", info_stamp_ns=11, info_frame_id="other", info_width=640, info_height=359, projection=_projection())
    assert not result.valid
    assert set(result.reasons) >= {"depth_camera_info_stamp_mismatch", "depth_camera_info_frame_mismatch", "depth_camera_info_size_mismatch"}
    translated = _projection()
    translated[3] = .1
    assert "rectified_left_projection_translation_must_be_zero" in validate_rectified_depth_pair(depth_stamp_ns=10, depth_frame_id="left_camera_optical_frame", depth_width=640, depth_height=360, depth_encoding="32FC1", info_stamp_ns=10, info_frame_id="left_camera_optical_frame", info_width=640, info_height=360, projection=translated).reasons


def test_rectified_projection_uses_p_not_raw_k():
    points = project_rectified_depth_pixels(np.asarray([150.0]), np.asarray([230.0]), np.asarray([2.0]), _projection())
    assert np.allclose(points, [[2.0, 2.0, 2.0]])


def test_dynamic_plane_must_match_depth_and_not_be_degraded():
    plane = {"stamp_sec": 0, "stamp_nanosec": 10, "camera_frame": "left_camera_optical_frame", "coordinate_contract": "dynamic_table_plane_camera_relative_only", "valid_for_table_removal": True, "degraded": False}
    assert validate_dynamic_plane_for_depth(plane, depth_stamp_ns=10, depth_frame_id="left_camera_optical_frame").valid
    stale = {**plane, "degraded": True}
    result = validate_dynamic_plane_for_depth(stale, depth_stamp_ns=11, depth_frame_id="other")
    assert not result.valid
    assert set(result.reasons) >= {"table_plane_depth_stamp_mismatch", "table_plane_depth_frame_mismatch", "table_plane_not_fresh"}


def test_plane_fit_has_residual_evidence_and_normal_continuity():
    grid_x, grid_y = np.meshgrid(np.linspace(-.2, .2, 16), np.linspace(-.2, .2, 16))
    points = np.column_stack([grid_x.ravel(), grid_y.ravel(), np.full(grid_x.size, .5)])
    points = np.vstack([points, [[.7, .7, .1], [-.7, .3, .9]]])
    fit = fit_plane_ransac(points, .005, 80, seed=7)
    assert fit is not None
    assert fit.inlier_ratio > .98
    assert fit.residual_rms_m < .001
    assert fit.residual_p95_m < .001
    assert normal_delta_deg(fit.normal, fit.normal) == 0.0
