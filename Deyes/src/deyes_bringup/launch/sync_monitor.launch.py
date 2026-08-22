from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_params_file = str(
        Path(get_package_share_directory("deyes_bringup")) / "config" / "sync_monitor.defaults.yaml"
    )

    launch_arguments = [
        DeclareLaunchArgument("params_file", default_value=default_params_file),
        DeclareLaunchArgument("camera_id", default_value="unknown_camera"),
        DeclareLaunchArgument("left_image_topic", default_value="/x1/left_camera/image_raw"),
        DeclareLaunchArgument("right_image_topic", default_value="/x1/right_camera/image_raw"),
        DeclareLaunchArgument("left_camera_info_topic", default_value="/x1/left_camera/camera_info"),
        DeclareLaunchArgument("right_camera_info_topic", default_value="/x1/right_camera/camera_info"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]

    node = Node(
        package="deyes_stereo",
        executable="sync_monitor",
        name="deyes_sync_monitor",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "camera_id": LaunchConfiguration("camera_id"),
                "left_image_topic": LaunchConfiguration("left_image_topic"),
                "right_image_topic": LaunchConfiguration("right_image_topic"),
                "left_camera_info_topic": LaunchConfiguration("left_camera_info_topic"),
                "right_camera_info_topic": LaunchConfiguration("right_camera_info_topic"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
    )

    return LaunchDescription(launch_arguments + [node])
