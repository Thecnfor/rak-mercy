"""Optional Nav2-gated wrapper for the one-shot pick pipeline.

It starts only the observation-only coordinator and the existing dry-run
pipeline.  It never starts ROS 1, ``move_base``, a Nav2 ActionClient, or a
velocity publisher.  A separately reviewed site adapter must supply
navigation evidence before the coordinator can arm the snapshot.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share_dir = Path(get_package_share_directory("deyes_bringup"))
    coordinator_params = str(share_dir / "config" / "pick_nav_coordinator.defaults.yaml")
    single_shot = str(share_dir / "launch" / "single_shot_pick.launch.py")
    arguments = (
        ("arm_side", "right"),
        ("autonomous_once", "true"),
        ("dry_run", "true"),
        ("enable_live_execution", "false"),
        ("operator_confirmed", "false"),
        ("live_navigation_action", "false"),
        ("expected_target_count", "1"),
        ("model_path", ""),
        ("model_id", ""),
        ("model_sha256", ""),
        ("site_profile_path", ""),
        ("stereo_calibration_path", ""),
        ("extrinsics_path", ""),
        ("log_root", ""),
    )
    declarations = [DeclareLaunchArgument(name, default_value=default) for name, default in arguments]
    # live_navigation_action belongs only to the coordinator; do not pass an
    # undeclared argument into the included single-shot launch.
    forwarded = {name: LaunchConfiguration(name) for name, _ in arguments if name != "live_navigation_action"}
    forwarded["require_nav_gate"] = "true"
    return LaunchDescription(declarations + [
        Node(
            package="deyes_stereo",
            executable="pick_nav_coordinator",
            name="pick_nav_coordinator_node",
            output="screen",
            parameters=[coordinator_params, {"live_navigation_action": LaunchConfiguration("live_navigation_action")}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(single_shot),
            launch_arguments=forwarded.items(),
        ),
    ])
