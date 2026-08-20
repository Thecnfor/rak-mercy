from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = Path(get_package_share_directory("deyes_bringup"))
    config_dir = pkg_share / "config"
    publisher_params = str(config_dir / "imx219_publisher.yaml")
    monitor_params = str(config_dir / "sync_monitor.defaults.yaml")
    sgbm_params = str(config_dir / "sgbm.defaults.yaml")

    launch_arguments = [
        DeclareLaunchArgument("left_image_topic", default_value="/x1/left_camera/image_raw"),
        DeclareLaunchArgument("right_image_topic", default_value="/x1/right_camera/image_raw"),
        DeclareLaunchArgument("left_info_topic", default_value="/x1/left_camera/camera_info"),
        DeclareLaunchArgument("right_info_topic", default_value="/x1/right_camera/camera_info"),
        DeclareLaunchArgument(
            "calib_path", default_value="/home/elephant/mercury_grasp/config/stereo_calib.yaml"
        ),
        DeclareLaunchArgument("camera_id", default_value="imx219_stereo_pair"),
    ]

    publisher = Node(
        package="deyes_stereo",
        executable="imx219_stereo_publisher",
        name="imx219_stereo_publisher",
        additional_env={"LD_PRELOAD": "/lib/aarch64-linux-gnu/libgomp.so.1"},
        output="screen",
        parameters=[
            publisher_params,
            {
                "left_image_topic": LaunchConfiguration("left_image_topic"),
                "right_image_topic": LaunchConfiguration("right_image_topic"),
                "left_info_topic": LaunchConfiguration("left_info_topic"),
                "right_info_topic": LaunchConfiguration("right_info_topic"),
                "calib_path": LaunchConfiguration("calib_path"),
            },
        ],
    )

    monitor = Node(
        package="deyes_stereo",
        executable="sync_monitor",
        name="deyes_sync_monitor",
        additional_env={"LD_PRELOAD": "/lib/aarch64-linux-gnu/libgomp.so.1"},
        output="screen",
        parameters=[
            monitor_params,
            {
                "camera_id": LaunchConfiguration("camera_id"),
                "left_image_topic": LaunchConfiguration("left_image_topic"),
                "right_image_topic": LaunchConfiguration("right_image_topic"),
                "left_camera_info_topic": LaunchConfiguration("left_info_topic"),
                "right_camera_info_topic": LaunchConfiguration("right_info_topic"),
            },
        ],
    )

    sgbm = Node(
        package="deyes_stereo",
        executable="sgbm_baseline",
        name="sgbm_baseline",
        additional_env={"LD_PRELOAD": "/lib/aarch64-linux-gnu/libgomp.so.1"},
        output="screen",
        parameters=[
            sgbm_params,
            {
                "calib_path": LaunchConfiguration("calib_path"),
                "left_image_topic": LaunchConfiguration("left_image_topic"),
                "right_image_topic": LaunchConfiguration("right_image_topic"),
                "left_camera_info_topic": LaunchConfiguration("left_info_topic"),
                "right_camera_info_topic": LaunchConfiguration("right_info_topic"),
            },
        ],
    )

    return LaunchDescription(launch_arguments + [publisher, monitor, sgbm])
