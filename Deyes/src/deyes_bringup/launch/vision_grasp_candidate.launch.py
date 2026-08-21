from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = str(Path(get_package_share_directory("deyes_bringup")) / "config" / "vision_grasp_candidate.defaults.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("pen_features_topic", default_value="/x1/detection/pen_features"),
        DeclareLaunchArgument("depth_topic", default_value="/x1/stereo/depth"),
        DeclareLaunchArgument("source", default_value="physical_topic"),
        Node(package="deyes_stereo", executable="vision_grasp_candidate", name="vision_grasp_candidate_node", output="screen", parameters=[params, {
            "pen_features_topic": LaunchConfiguration("pen_features_topic"), "depth_topic": LaunchConfiguration("depth_topic"), "source": LaunchConfiguration("source"),
        }]),
    ])
