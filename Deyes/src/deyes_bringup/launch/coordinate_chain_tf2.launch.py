from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_dir = Path(get_package_share_directory("deyes_bringup")) / "config"
    params = str(config_dir / "coordinate_chain_tf2.defaults.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("extrinsics_path", default_value=""),
        DeclareLaunchArgument("stereo_calibration_path", default_value=""),
        DeclareLaunchArgument("site_frames_path", default_value=str(config_dir / "coordinate_chain_frames.site.template.yaml")),
        DeclareLaunchArgument("tf_probe_report_path", default_value=""),
        DeclareLaunchArgument("tf_probe_template_path", default_value=""),
        Node(package="deyes_stereo", executable="validated_extrinsics_tf", name="validated_extrinsics_tf_node", output="screen", parameters=[params, {"extrinsics_path": LaunchConfiguration("extrinsics_path"), "stereo_calibration_path": LaunchConfiguration("stereo_calibration_path")}]),
        Node(package="deyes_stereo", executable="coordinate_chain_tf2", name="coordinate_chain_tf2_node", output="screen", parameters=[params, LaunchConfiguration("site_frames_path")]),
        Node(package="deyes_stereo", executable="tf_chain_audit", name="tf_chain_audit_node", output="screen", parameters=[params, LaunchConfiguration("site_frames_path")]),
        Node(package="deyes_stereo", executable="coordinate_chain_candidate_bridge", name="coordinate_chain_candidate_bridge_node", output="screen", parameters=[params, LaunchConfiguration("site_frames_path")]),
        Node(package="deyes_stereo", executable="tf_frame_probe", name="tf_frame_probe_node", output="screen", parameters=[params, {"report_path": LaunchConfiguration("tf_probe_report_path"), "template_path": LaunchConfiguration("tf_probe_template_path")}]),
    ])
