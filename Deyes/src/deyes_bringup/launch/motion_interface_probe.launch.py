from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = str(Path(get_package_share_directory("deyes_bringup")) / "config" / "motion_interface_probe.defaults.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("enable_execution", default_value="false"),
        Node(package="deyes_stereo", executable="motion_interface_probe", name="motion_interface_probe_node", output="screen", parameters=[params, {
            "enable_execution": LaunchConfiguration("enable_execution"),
        }]),
    ])
