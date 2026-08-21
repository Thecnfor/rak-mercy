from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = str(Path(get_package_share_directory("deyes_bringup")) / "config" / "isaac_single_pen_candidate.defaults.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("ros_domain_id", default_value="46"),
        DeclareLaunchArgument("expected_scene_sha256", default_value=""),
        SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
        Node(package="deyes_stereo", executable="isaac_single_pen_candidate", name="isaac_single_pen_candidate_node", output="screen", parameters=[config, {"expected_scene_sha256": LaunchConfiguration("expected_scene_sha256")}]),
    ])
