"""ROS-free rendering for a read-only TF discovery report and site template."""

from __future__ import annotations


def build_site_template(discovered_frames: list[str]) -> dict[str, object]:
    """Leave official names blank; discovered names are evidence, not defaults."""
    hints = [frame for frame in discovered_frames if any(token in frame.lower() for token in ("left", "right", "arm", "tool", "tcp", "gripper"))]
    return {"tf_chain_audit_node": {"ros__parameters": {"base_frame": "base_link", "camera_frame": "left_camera_optical_frame", "required_end_effector_frames": []}}, "site_operator_instructions": "Copy only confirmed official frame IDs into required_end_effector_frames; do not use discovery_hints as defaults.", "unverified_discovery_hints": hints}
