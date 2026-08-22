from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = Path(get_package_share_directory("deyes_bringup"))
    cuda_depth_params = str(pkg_share / "config" / "cuda_depth.defaults.yaml")
    debug_calib = str(pkg_share / "config" / "camera" / "stereo_calib.yaml")

    launch_arguments = [
        DeclareLaunchArgument(
            "cuda_depth_config",
            default_value=cuda_depth_params,
        ),
        DeclareLaunchArgument("calib_path", default_value=debug_calib),
        DeclareLaunchArgument("left_image_topic", default_value="/x1/left_camera/image_raw"),
        DeclareLaunchArgument("right_image_topic", default_value="/x1/right_camera/image_raw"),
        DeclareLaunchArgument("stereo_disparity_topic", default_value="/x1/stereo/disparity"),
        DeclareLaunchArgument("stereo_depth_topic", default_value="/x1/stereo/depth"),
        DeclareLaunchArgument(
            "stereo_left_rect_camera_info_topic",
            default_value="/x1/stereo/left/camera_info_rect",
        ),
        DeclareLaunchArgument("cuda_depth_max_sync_diff_ms", default_value="10.0"),
        DeclareLaunchArgument("cuda_depth_publish_period_sec", default_value="0.07"),
        DeclareLaunchArgument("cuda_depth_min_depth_m", default_value="0.20"),
        DeclareLaunchArgument("cuda_depth_max_depth_m", default_value="1.00"),
        DeclareLaunchArgument("cuda_depth_enable_wls_filter", default_value="false"),
        DeclareLaunchArgument("cuda_depth_wls_lambda", default_value="8000.0"),
        DeclareLaunchArgument("cuda_depth_wls_sigma_color", default_value="2.0"),
        DeclareLaunchArgument("cuda_depth_texture_threshold", default_value="0"),
        DeclareLaunchArgument("cuda_depth_uniqueness_ratio", default_value="0"),
        DeclareLaunchArgument("cuda_depth_speckle_window_size", default_value="0"),
        DeclareLaunchArgument("cuda_depth_speckle_range", default_value="0"),
        DeclareLaunchArgument("cuda_depth_disp12_max_diff", default_value="0"),
        DeclareLaunchArgument("cuda_depth_publish_debug_rect", default_value="false"),
        DeclareLaunchArgument("cuda_depth_publish_debug_mask", default_value="false"),
    ]

    cuda_depth = Node(
        package="deyes_capture_cpp",
        executable="cuda_stereo_depth_node",
        name="cuda_stereo_depth_node",
        output="screen",
        parameters=[
            LaunchConfiguration("cuda_depth_config"),
            {
                "calib_path": LaunchConfiguration("calib_path"),
                "left_image_topic": LaunchConfiguration("left_image_topic"),
                "right_image_topic": LaunchConfiguration("right_image_topic"),
                "disparity_topic": LaunchConfiguration("stereo_disparity_topic"),
                "depth_topic": LaunchConfiguration("stereo_depth_topic"),
                "left_rect_camera_info_topic": LaunchConfiguration(
                    "stereo_left_rect_camera_info_topic"
                ),
                "max_sync_diff_ms": LaunchConfiguration("cuda_depth_max_sync_diff_ms"),
                "publish_period_sec": LaunchConfiguration("cuda_depth_publish_period_sec"),
                "min_depth_m": LaunchConfiguration("cuda_depth_min_depth_m"),
                "max_depth_m": LaunchConfiguration("cuda_depth_max_depth_m"),
                "enable_wls_filter": LaunchConfiguration("cuda_depth_enable_wls_filter"),
                "wls_lambda": LaunchConfiguration("cuda_depth_wls_lambda"),
                "wls_sigma_color": LaunchConfiguration("cuda_depth_wls_sigma_color"),
                "texture_threshold": LaunchConfiguration("cuda_depth_texture_threshold"),
                "uniqueness_ratio": LaunchConfiguration("cuda_depth_uniqueness_ratio"),
                "speckle_window_size": LaunchConfiguration("cuda_depth_speckle_window_size"),
                "speckle_range": LaunchConfiguration("cuda_depth_speckle_range"),
                "disp12_max_diff": LaunchConfiguration("cuda_depth_disp12_max_diff"),
                "publish_debug_rect": LaunchConfiguration("cuda_depth_publish_debug_rect"),
                "publish_debug_mask": LaunchConfiguration("cuda_depth_publish_debug_mask"),
            },
        ],
    )

    return LaunchDescription(launch_arguments + [cuda_depth])
