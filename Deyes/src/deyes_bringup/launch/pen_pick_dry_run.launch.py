from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = str(Path(get_package_share_directory("deyes_bringup")) / "config" / "pen_pick_dry_run.defaults.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("enable_execution", default_value="false"),
        DeclareLaunchArgument("operator_approved", default_value="false"),
        Node(package="deyes_stereo", executable="pen_pick_dry_run", name="pen_pick_dry_run_node", output="screen", parameters=[params, {
            "enable_execution": LaunchConfiguration("enable_execution"),
            "operator_approved": LaunchConfiguration("operator_approved"),
        }]),
    ])
