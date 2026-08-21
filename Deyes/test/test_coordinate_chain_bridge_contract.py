from deyes_stereo.coordinate_chain_bridge_contract import build_coordinate_chain_requests
from deyes_stereo.tf_frame_probe_contract import build_site_template
from pathlib import Path


VALID_STATUS = {"trusted_for_grasp": True, "physical_validated": True, "tf_published": True}
PAYLOAD = {"valid": True, "candidates": [{"target_id": "pen-1", "valid": True, "trusted_for_grasp": False, "coordinate_chain_point": {"kind": "point", "source_frame": "left_camera_optical_frame", "stamp_ns": 9, "position_m": [.1, .2, .3]}}]}


def test_visual_candidate_bridge_requires_physical_validation_and_never_leaks_request_on_rejection():
    rejected = build_coordinate_chain_requests(PAYLOAD, target_frame="official_tool", extrinsics_status={"trusted_for_grasp": False, "physical_validated": False, "tf_published": False})
    assert rejected == {"state": "rejected", "reason": "extrinsics_not_trusted_for_grasp", "requests": [], "published": False}
    assert build_coordinate_chain_requests(PAYLOAD, target_frame="", extrinsics_status=VALID_STATUS)["reason"] == "target_frame_not_configured"


def test_visual_candidate_bridge_only_creates_a_tf2_request_after_all_gates_pass():
    result = build_coordinate_chain_requests(PAYLOAD, target_frame="official_tool", extrinsics_status=VALID_STATUS)
    assert result["published"] and result["requests"][0]["target_frame"] == "official_tool"
    assert result["requests"][0]["candidate_id"] == "pen-1"


def test_visual_candidate_bridge_preserves_complete_navigation_identity_and_rejects_partial_identity():
    payload={**PAYLOAD,"mission_id":"m-1","nav_epoch":4}
    result=build_coordinate_chain_requests(payload,target_frame="official_tool",extrinsics_status=VALID_STATUS)
    assert result["published"] and result["requests"][0]["mission_id"]=="m-1" and result["requests"][0]["nav_epoch"]==4
    assert build_coordinate_chain_requests({**PAYLOAD,"mission_id":"m-1"},target_frame="official_tool",extrinsics_status=VALID_STATUS)["reason"]=="navigation_identity_incomplete"


def test_tf_probe_template_exposes_discovery_without_using_it_as_a_default():
    template = build_site_template(["base_link", "left_arm_tool_real", "right_gripper_real"])
    assert template["tf_chain_audit_node"]["ros__parameters"]["required_end_effector_frames"] == []
    assert template["unverified_discovery_hints"] == ["left_arm_tool_real", "right_gripper_real"]


def test_coordinate_bridge_subscribes_to_the_vision_candidate_output_topic():
    root = Path(__file__).resolve().parents[1]
    config = (root / "config" / "stereo" / "coordinate_chain_tf2.defaults.yaml").read_text(encoding="utf-8")
    node = (root / "src" / "deyes_stereo" / "deyes_stereo" / "coordinate_chain_candidate_bridge_node.py").read_text(encoding="utf-8")
    assert 'candidate_topic: "/x1/grasp/candidates_camera"' in config
    assert '"candidate_topic": "/x1/grasp/candidates_camera"' in node
