"""One-command real-robot perception-to-pick dry-run bringup.

This launch starts the physical stereo source, CUDA depth, table-plane
estimation, and the navigation-gated one-shot pipeline.  Hardware execution
is deliberately locked off here; live motion remains a separate, staged
operator action after calibration and hover checks pass.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    share_dir = Path(get_package_share_directory("deyes_bringup"))
    stereo_launch = str(share_dir / "launch" / "imx219_stereo.launch.py")
    pick_launch = str(share_dir / "launch" / "navigation_single_shot_pick.launch.py")

    arguments = (
        ("calib_path", ""),
        ("extrinsics_path", ""),
        ("site_profile_path", ""),
        ("model_path", ""),
        ("model_id", "pen-yolov5-student-01875-416-v1"),
        ("model_sha256", ""),
        ("log_root", "/home/elephant/temp/deyes/single_shot_pick"),
        ("width", "640"),
        ("height", "360"),
        ("fps", "30"),
    )
    declarations = [DeclareLaunchArgument(name, default_value=value) for name, value in arguments]

    stereo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stereo_launch),
        launch_arguments={
            "calib_path": LaunchConfiguration("calib_path"),
            "width": LaunchConfiguration("width"),
            "height": LaunchConfiguration("height"),
            "fps": LaunchConfiguration("fps"),
            "use_cpp_capture": "true",
            "enable_cuda_depth": "true",
            "cuda_depth_max_sync_diff_ms": "10.0",
            "cuda_depth_publish_debug_rect": "true",
            "enable_ground_plane": "true",
            "enable_detector": "false",
            "enable_pen_features": "false",
        }.items(),
    )

    pick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pick_launch),
        launch_arguments={
            "arm_side": "right",
            "autonomous_once": "true",
            "dry_run": "true",
            "enable_live_execution": "false",
            "operator_confirmed": "false",
            "live_navigation_action": "false",
            "expected_target_count": "1",
            "model_path": LaunchConfiguration("model_path"),
            "model_id": LaunchConfiguration("model_id"),
            "model_sha256": LaunchConfiguration("model_sha256"),
            "site_profile_path": LaunchConfiguration("site_profile_path"),
            "stereo_calibration_path": LaunchConfiguration("calib_path"),
            "extrinsics_path": LaunchConfiguration("extrinsics_path"),
            "log_root": LaunchConfiguration("log_root"),
        }.items(),
    )

    return LaunchDescription(declarations + [stereo, pick])
