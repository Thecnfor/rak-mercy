import numpy as np

from deyes_stereo.extrinsics_contract import matrix_to_quaternion, solve_base_from_camera, validate_extrinsics
from deyes_stereo.handeye_calibration import build_document


def _stereo():
    return {"validated": True, "calibration_id": "stereo-real-1", "robot_id": "x1-7", "camera_pair_id": "cam-2"}


def test_point_correspondence_solution_binds_stereo_identity():
    camera = np.array([[0, 0, .4], [.1, 0, .4], [0, .1, .4], [.1, .1, .5], [.2, 0, .6], [.2, .1, .5]])
    rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
    translation = np.array([.3, -.2, .7])
    base = (rotation @ camera.T).T + translation
    solved_r, solved_t, residuals = solve_base_from_camera(camera, base)
    assert np.allclose(solved_r, rotation)
    assert np.allclose(solved_t, translation)
    assert float(residuals.max()) < 1e-10
    payload = {"calibration_id": "handeye-real-1", "robot_id": "x1-7", "camera_pair_id": "cam-2", "stereo_calibration_id": "stereo-real-1", "operator_confirmation": True,
               "correspondences": [{"camera_point_m": c.tolist(), "base_point_m": b.tolist()} for c, b in zip(camera, base)]}
    result = build_document(payload)
    assert result["validated"] is True
    assert validate_extrinsics(result, stereo_document=_stereo()).valid is True


def test_unvalidated_or_identity_mismatch_never_passes_gate():
    document = {"calibration_id": "h", "validated": False, "source": "physical_point_correspondences", "source_frame": "left_camera_optical_frame", "target_frame": "base_link", "robot_id": "x1-7", "camera_pair_id": "cam-2", "stereo_calibration_id": "other", "translation_m": [0, 0, 0], "quaternion_xyzw": matrix_to_quaternion(np.eye(3)), "metrics": {"correspondence_count": 6, "rms_m": .001, "p95_m": .002}}
    result = validate_extrinsics(document, stereo_document=_stereo())
    assert result.valid is False
    assert "extrinsics_not_validated" in result.reasons
    assert "stereo_calibration_identity_mismatch" in result.reasons
