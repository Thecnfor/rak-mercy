from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_defaults(config_file: Path) -> dict:
    with config_file.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data["stereo_image_proc_baseline"]["ros__parameters"]


def generate_launch_description() -> LaunchDescription:
    pkg_share = Path(get_package_share_directory("deyes_bringup"))
    config_dir = pkg_share / "config"
    defaults = _load_defaults(config_dir / "stereo_image_proc.yaml")
    publisher_params = str(config_dir / "imx219_publisher.yaml")
    monitor_params = str(config_dir / "sync_monitor.defaults.yaml")
    debug_calib = str(config_dir / "camera" / "stereo_calib.yaml")

    launch_arguments = [
        DeclareLaunchArgument("namespace", default_value=str(defaults["namespace"])),
        DeclareLaunchArgument("left_namespace", default_value=str(defaults["left_namespace"])),
        DeclareLaunchArgument("right_namespace", default_value=str(defaults["right_namespace"])),
        DeclareLaunchArgument("approx_sync", default_value=str(defaults["approx_sync"]).lower()),
        DeclareLaunchArgument("use_color", default_value=str(defaults["use_color"]).lower()),
        DeclareLaunchArgument(
            "enable_monitor", default_value=str(defaults["enable_monitor"]).lower()
        ),
        DeclareLaunchArgument("calib_path", default_value=debug_calib),
        DeclareLaunchArgument("camera_id", default_value="imx219_stereo_pair"),
    ]

    namespace = LaunchConfiguration("namespace")
    left_namespace = LaunchConfiguration("left_namespace")
    right_namespace = LaunchConfiguration("right_namespace")

    publisher = Node(
        package="deyes_stereo",
        executable="imx219_stereo_publisher",
        name="imx219_stereo_publisher",
        additional_env={"LD_PRELOAD": "/lib/aarch64-linux-gnu/libgomp.so.1"},
        output="screen",
        parameters=[
            publisher_params,
            {
                "left_image_topic": ["/", namespace, "/", left_namespace, "/image_raw"],
                "right_image_topic": ["/", namespace, "/", right_namespace, "/image_raw"],
                "left_info_topic": ["/", namespace, "/", left_namespace, "/camera_info"],
                "right_info_topic": ["/", namespace, "/", right_namespace, "/camera_info"],
                "calib_path": LaunchConfiguration("calib_path"),
            },
        ],
    )

    monitor = Node(
        package="deyes_stereo",
        executable="sync_monitor",
        name="deyes_sync_monitor",
        condition=IfCondition(LaunchConfiguration("enable_monitor")),
        additional_env={"LD_PRELOAD": "/lib/aarch64-linux-gnu/libgomp.so.1"},
        output="screen",
        parameters=[
            monitor_params,
            {
                "camera_id": LaunchConfiguration("camera_id"),
                "left_image_topic": ["/", namespace, "/", left_namespace, "/image_raw"],
                "right_image_topic": ["/", namespace, "/", right_namespace, "/image_raw"],
                "left_camera_info_topic": ["/", namespace, "/", left_namespace, "/camera_info"],
                "right_camera_info_topic": ["/", namespace, "/", right_namespace, "/camera_info"],
            },
        ],
    )

    stereo_image_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("stereo_image_proc"))
                / "launch"
                / "stereo_image_proc.launch.py"
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "left_namespace": left_namespace,
            "right_namespace": right_namespace,
            "approximate_sync": LaunchConfiguration("approx_sync"),
            "use_color": LaunchConfiguration("use_color"),
        }.items(),
    )

    return LaunchDescription(launch_arguments + [publisher, monitor, stereo_image_proc])
