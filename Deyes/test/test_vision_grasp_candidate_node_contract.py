"""Static launch/config guardrails for the perception-only ROS wrapper."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_camera_candidate_node_uses_required_topics_and_has_no_motion_clients():
    node = (ROOT / "src" / "deyes_stereo" / "deyes_stereo" / "vision_grasp_candidate_node.py").read_text(encoding="utf-8")
    for required in ("/x1/detection/pen_features", "/x1/stereo/depth", "coordinate_chain_templates_topic", "ExactStampPairCache", "max_target_age_sec", "build_camera_optical_pen_candidates"):
        assert required in node
    for forbidden in ("pymycobot", "ActionClient", "create_client", "cmd_vel", "joint_states"):
        assert forbidden not in node


def test_launch_and_defaults_publish_camera_candidates_only():
    config = (ROOT / "config" / "stereo" / "vision_grasp_candidate.defaults.yaml").read_text(encoding="utf-8")
    launch = (ROOT / "src" / "deyes_bringup" / "launch" / "vision_grasp_candidate.launch.py").read_text(encoding="utf-8")
    assert 'output_topic: "/x1/grasp/candidates_camera"' in config
    assert 'coordinate_chain_templates_topic: "/x1/grasp/candidates_camera/coordinate_chain_templates"' in config
    assert "executable=\"vision_grasp_candidate\"" in launch
