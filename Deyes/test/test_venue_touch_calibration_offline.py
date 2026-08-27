import importlib.util
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml


TOOLS = Path(__file__).parents[1] / "tools"


def _load(name):
    spec = importlib.util.spec_from_file_location(name, TOOLS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


calibration = _load("calibrate_venue_touch_projector")
height = _load("recompute_venue_ground_height")


def test_solvepnp_recovers_known_planar_transform_and_retains_ippe_candidates():
    objects = np.array([
        [0.36, -0.04, 0.045], [0.42, -0.04, 0.045], [0.39, 0.02, 0.045],
        [0.36, 0.09, 0.045], [0.42, 0.09, 0.045], [0.39, 0.14, 0.045],
    ])
    camera_matrix = np.array([[441.0, 0.0, 325.0], [0.0, 441.0, 180.0], [0.0, 0.0, 1.0]])
    expected = np.eye(4)
    expected[:3, :3] = np.diag([1.0, -1.0, -1.0])
    expected[:3, 3] = [0.0, 0.0, 0.75]
    rvec = cv2.Rodrigues(expected[:3, :3])[0]
    images = cv2.projectPoints(objects, rvec, expected[:3, 3], camera_matrix, np.zeros(5))[0][:, 0]
    candidates = calibration.solve_candidates(objects, images, camera_matrix)
    assert {item["method"].split("[")[0] for item in candidates} >= {"IPPE", "ITERATIVE"}
    selected = calibration.select_positive_candidate(candidates)
    assert selected["reprojection_rms_px"] < 5e-6
    for point, pixel in zip(objects, images):
        recovered = calibration.intersect_pixel(selected["camera_from_right_arm_sdk"], camera_matrix, pixel, 0.045)
        assert np.allclose(recovered, point, atol=1e-6)


def test_collinear_planar_touch_set_is_rejected_without_relaxing_six_point_gate():
    objects = np.array([[0.35 + i * 0.01, 0.0, 0.045] for i in range(6)])
    images = np.array([[200.0 + i, 220.0] for i in range(6)])
    with pytest.raises(ValueError, match="collinear"):
        calibration.solve_candidates(objects, images, np.eye(3))


def test_560_to_650_parallel_table_relation_is_90mm_closer():
    assert height.expected_parallel_plane_distance_m(0.559, 560.0, 650.0) == pytest.approx(0.469)
    path = Path(__file__).parents[1] / "config" / "camera" / "venue_20260827_old_table_height.yaml"
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert document["schema"] == "venue_ground_height_evidence/v1"
    assert document["source_frame_count"] == 5
    assert document["new_table_expectation"]["height_delta_mm"] == 90.0
    assert document["new_table_expectation"]["expected_plane_distance_camera_m"] == pytest.approx(
        document["old_table"]["plane_distance_median_m"] - 0.09
    )
