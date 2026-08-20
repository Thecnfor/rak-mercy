from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = str(Path(get_package_share_directory("deyes_bringup")) / "config" / "pen_grasp.defaults.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("extrinsics_path", default_value=""),
        DeclareLaunchArgument("stereo_calibration_path", default_value=""),
        DeclareLaunchArgument("pen_features_topic", default_value="/x1/detection/pen_features"),
        Node(package="deyes_stereo", executable="pen_grasp", name="pen_grasp_node", output="screen", parameters=[params, {
            "extrinsics_path": LaunchConfiguration("extrinsics_path"), "stereo_calibration_path": LaunchConfiguration("stereo_calibration_path"),
            "pen_features_topic": LaunchConfiguration("pen_features_topic"),
        }]),
    ])
