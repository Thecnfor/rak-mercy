from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = str(Path(get_package_share_directory("deyes_bringup")) / "config" / "isaac_right_arm_stage_executor.defaults.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("enable_execution", default_value="false"),
        DeclareLaunchArgument("simulation_only", default_value="false"),
        DeclareLaunchArgument("motion_enabled", default_value="false"),
        DeclareLaunchArgument("ros_domain_id", default_value="46"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
        Node(package="deyes_stereo", executable="isaac_right_arm_stage_executor", name="isaac_right_arm_stage_executor", output="screen", parameters=[config, {
            "enable_execution": LaunchConfiguration("enable_execution"),
            "simulation_only": LaunchConfiguration("simulation_only"),
            "motion_enabled": LaunchConfiguration("motion_enabled"),
            "ros_domain_id": LaunchConfiguration("ros_domain_id"),
        }]),
    ])
