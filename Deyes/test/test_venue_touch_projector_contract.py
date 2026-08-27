from pathlib import Path

import numpy as np
import yaml

from deyes_stereo.venue_touch_projector_contract import (
    project_pixel_to_fixed_z,
    validate_evidence_document,
)


def _projection():
    return np.array([[500.0, 0.0, 320.0, 0.0], [0.0, 500.0, 180.0, 0.0], [0.0, 0.0, 1.0, 0.0]])


def _downlooking_camera_from_right():
    # X camera == X SDK, Y camera == -Y SDK, Z camera == -Z SDK.
    transform = np.eye(4)
    transform[:3, :3] = np.diag([1.0, -1.0, -1.0])
    transform[:3, 3] = [0.0, 0.0, 0.80]
    return transform


def test_fixed_z_ray_recovers_known_right_arm_sdk_point():
    transform = _downlooking_camera_from_right()
    expected = np.array([0.40, 0.05, 0.045])
    camera = transform[:3, :3] @ expected + transform[:3, 3]
    pixel = [500.0 * camera[0] / camera[2] + 320.0, 500.0 * camera[1] / camera[2] + 180.0]
    result = project_pixel_to_fixed_z(
        pixel, _projection(), transform, 0.045,
        workspace_xyz_m=[[0.30, 0.50], [-0.10, 0.20], [0.045, 0.045]],
        calibration_hull_xy_m=[[0.30, -0.10], [0.50, -0.10], [0.50, 0.20], [0.30, 0.20]],
    )
    assert result.usable, result.reasons
    assert np.allclose(result.point_right_arm_sdk_m, expected, atol=1e-9)
    assert result.camera_depth_m > 0.0


def test_wrong_matrix_direction_is_rejected_even_if_numeric_matrix_is_rigid():
    result = project_pixel_to_fixed_z(
        [320.0, 180.0], _projection(), np.linalg.inv(_downlooking_camera_from_right()), 0.045,
        matrix_direction="right_arm_sdk_from_camera",
        calibration_hull_xy_m=[[0.3, -0.1], [0.5, -0.1], [0.5, 0.2]],
    )
    assert not result.usable
    assert result.reasons == ("matrix_direction_must_be_camera_from_right_arm_sdk",)


def test_planar_ray_degeneracy_and_convex_hull_extrapolation_are_rejected():
    # Camera optical Z maps to SDK X, hence the principal ray is parallel to SDK Z plane.
    parallel = np.eye(4)
    parallel[:3, :3] = [[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]]
    parallel_result = project_pixel_to_fixed_z(
        [320.0, 180.0], _projection(), parallel, 0.045,
        calibration_hull_xy_m=[[0.3, -0.1], [0.5, -0.1], [0.5, 0.2]],
    )
    assert not parallel_result.usable
    assert "camera_ray_parallel_to_fixed_z_plane" in parallel_result.reasons

    transform = _downlooking_camera_from_right()
    outside = np.array([0.60, 0.0, 0.045])
    camera = transform[:3, :3] @ outside + transform[:3, 3]
    pixel = [500.0 * camera[0] / camera[2] + 320.0, 500.0 * camera[1] / camera[2] + 180.0]
    hull_result = project_pixel_to_fixed_z(
        pixel, _projection(), transform, 0.045,
        workspace_xyz_m=[[0.2, 0.8], [-0.2, 0.2], [0.045, 0.045]],
        calibration_hull_xy_m=[[0.3, -0.1], [0.5, -0.1], [0.5, 0.1], [0.3, 0.1]],
    )
    assert not hull_result.usable
    assert "intersection_outside_calibration_hull" in hull_result.reasons


def test_committed_evidence_has_contract_schema_and_truthful_failed_gate():
    path = Path(__file__).parents[1] / "config" / "camera" / "venue_20260827_touch_projector.yaml"
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert validate_evidence_document(document) == ()
    assert document["usable"] is False
    assert document["failed_gates"] == ["reprojection_rms_px_lte_4"]
    assert document["metrics"]["point_count"] == 6
    assert document["publishes_tf"] is False
    assert document["is_base_link_hand_eye"] is False
