from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = Path(get_package_share_directory("deyes_bringup"))
    defaults = str(pkg_share / "config" / "pointcloud.defaults.yaml")
    arguments = [
        DeclareLaunchArgument("pointcloud_config", default_value=defaults),
        DeclareLaunchArgument("stereo_depth_topic", default_value="/x1/stereo/depth"),
        DeclareLaunchArgument(
            "stereo_left_rect_camera_info_topic",
            default_value="/x1/stereo/left/camera_info_rect",
        ),
        DeclareLaunchArgument("stereo_points_topic", default_value="/x1/stereo/points"),
        DeclareLaunchArgument("stereo_points_status_topic", default_value="/x1/stereo/points_status"),
        DeclareLaunchArgument("pointcloud_calibration_id", default_value="unassigned"),
        DeclareLaunchArgument("pointcloud_calibration_validated", default_value="false"),
    ]
    node = Node(
        package="deyes_capture_cpp",
        executable="stereo_pointcloud_node",
        name="stereo_pointcloud_node",
        output="screen",
        parameters=[
            LaunchConfiguration("pointcloud_config"),
            {
                "depth_topic": LaunchConfiguration("stereo_depth_topic"),
                "rectified_camera_info_topic": LaunchConfiguration(
                    "stereo_left_rect_camera_info_topic"
                ),
                "points_topic": LaunchConfiguration("stereo_points_topic"),
                "status_topic": LaunchConfiguration("stereo_points_status_topic"),
                "calibration_id": LaunchConfiguration("pointcloud_calibration_id"),
                "calibration_validated": LaunchConfiguration("pointcloud_calibration_validated"),
            },
        ],
    )
    return LaunchDescription(arguments + [node])
