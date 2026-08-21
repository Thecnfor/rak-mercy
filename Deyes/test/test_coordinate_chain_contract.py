import numpy as np
import pytest

from deyes_stereo.coordinate_chain_contract import (
    transform_request, trusted_for_execution, validate_request,
)


def test_synthetic_camera_base_tool_chain_uses_rigid_geometry_for_point_and_pose():
    # Synthetic-only geometry: this is not a calibration document and cannot
    # produce an execution token by itself.
    camera_to_base_r = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    camera_to_base_t = np.array([.30, -.20, .70])
    request = {"kind": "pose", "source_frame": "left_camera_optical_frame", "target_frame": "left_tool_frame", "stamp_ns": 42, "position_m": [.10, .20, .40], "quaternion_xyzw": [0., 0., 0., 1.]}
    # In a real graph TF2 composes camera->base and base->left_tool; the
    # composed transform here represents that same lookup result.
    base_to_tool_t = np.array([.05, .10, -.10])
    result = transform_request(request, camera_to_base_r, camera_to_base_t + base_to_tool_t, tf_quaternion_xyzw=[0., 0., 0.70710678, 0.70710678])
    assert result["frame_id"] == "left_tool_frame"
    assert result["position_m"] == pytest.approx([.15, 0., 1.0])
    assert result["quaternion_xyzw"] == pytest.approx([0., 0., 0.70710678, 0.70710678])
    assert result["transform_interface"] == "tf2"


def test_execution_gate_fails_closed_for_missing_simulated_or_unpublished_status():
    assert trusted_for_execution(None) == (False, "extrinsics_status_missing")
    assert trusted_for_execution({"trusted_for_grasp": True, "physical_validated": False, "tf_published": True}) == (False, "extrinsics_not_physically_validated")
    assert trusted_for_execution({"trusted_for_grasp": True, "physical_validated": True, "tf_published": False}) == (False, "validated_extrinsics_tf_not_published")
    assert trusted_for_execution({"trusted_for_grasp": True, "physical_validated": True, "tf_published": True}) == (True, "ok")


def test_only_camera_frame_requests_are_accepted_and_embedded_transforms_are_ignored():
    with pytest.raises(ValueError, match="source_frame_must_be_left_camera_optical_frame"):
        validate_request({"kind": "point", "source_frame": "Left_Camera", "target_frame": "base_link", "position_m": [0, 0, 1]})
    request = validate_request({"kind": "point", "source_frame": "left_camera_optical_frame", "target_frame": "right_gripper_frame", "position_m": [0, 0, 1], "manual_translation_m": [100, 100, 100]})
    assert "manual_translation_m" not in request


def test_navigation_identity_is_all_or_nothing_and_survives_tf_transform():
    raw={"kind":"point","source_frame":"left_camera_optical_frame","target_frame":"base_link","position_m":[0,0,1],"mission_id":"m-1","nav_epoch":2}
    result=transform_request(raw,np.eye(3),np.zeros(3),tf_quaternion_xyzw=[0,0,0,1])
    assert result["mission_id"]=="m-1" and result["nav_epoch"]==2
    with pytest.raises(ValueError,match="navigation_identity_incomplete"):
        validate_request({k:v for k,v in raw.items() if k!="nav_epoch"})
