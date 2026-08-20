from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = Path(get_package_share_directory("deyes_bringup"))
    publisher_params = str(pkg_share / "config" / "imx219_publisher.yaml")
    cpp_capture_params = str(pkg_share / "config" / "imx219_capture_cpp.yaml")
    monitor_params = str(pkg_share / "config" / "sync_monitor.defaults.yaml")
    cuda_depth_params = str(pkg_share / "config" / "cuda_depth.defaults.yaml")
    pointcloud_params = str(pkg_share / "config" / "pointcloud.defaults.yaml")
    depth_coordinate_params = str(pkg_share / "config" / "depth_coordinate.defaults.yaml")
    yolo_detector_params = str(pkg_share / "config" / "yolo_detector.defaults.yaml")
    object_fusion_params = str(pkg_share / "config" / "object_fusion.defaults.yaml")
    ground_plane_params = str(pkg_share / "config" / "ground_plane.defaults.yaml")
    debug_calib = str(pkg_share / "config" / "camera" / "stereo_calib.yaml")

    # 左右图像话题统一使用 /x1/... 命名，与实机 x1_vision 约定对齐。
    launch_arguments = [
        DeclareLaunchArgument("left_image_topic", default_value="/x1/left_camera/image_raw"),
        DeclareLaunchArgument("right_image_topic", default_value="/x1/right_camera/image_raw"),
        DeclareLaunchArgument("left_info_topic", default_value="/x1/left_camera/camera_info"),
        DeclareLaunchArgument("right_info_topic", default_value="/x1/right_camera/camera_info"),
        DeclareLaunchArgument("width", default_value="640"),
        DeclareLaunchArgument("height", default_value="360"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument("target_publish_hz", default_value="30.0"),
        DeclareLaunchArgument("pair_max_skew_ms", default_value="20.0"),
        DeclareLaunchArgument("frame_stale_sec", default_value="0.2"),
        DeclareLaunchArgument("history_size", default_value="8"),
        DeclareLaunchArgument("publish_period_sec", default_value="0.033333333"),
        DeclareLaunchArgument("output_encoding", default_value="mono8"),
        DeclareLaunchArgument("rotate_180", default_value="true"),
        DeclareLaunchArgument("mirror_horizontal", default_value="true"),
        DeclareLaunchArgument("swap_left_right", default_value="false"),
        DeclareLaunchArgument("monitor_expected_min_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("monitor_hard_sync_max_ms", default_value="3.0"),
        DeclareLaunchArgument("monitor_soft_sync_max_ms", default_value="10.0"),
        DeclareLaunchArgument("monitor_allow_soft_sync", default_value="false"),
        DeclareLaunchArgument("calib_path", default_value=debug_calib),
        DeclareLaunchArgument("enable_monitor", default_value="true"),
        DeclareLaunchArgument("enable_cuda_depth", default_value="false"),
        DeclareLaunchArgument("cuda_depth_config", default_value=cuda_depth_params),
        DeclareLaunchArgument("enable_pointcloud", default_value="false"),
        DeclareLaunchArgument("pointcloud_config", default_value=pointcloud_params),
        DeclareLaunchArgument("enable_depth_coordinate", default_value="false"),
        DeclareLaunchArgument("depth_coordinate_config", default_value=depth_coordinate_params),
        DeclareLaunchArgument("enable_detector", default_value="false"),
        DeclareLaunchArgument("detector_config", default_value=yolo_detector_params),
        DeclareLaunchArgument("enable_object_fusion", default_value="false"),
        DeclareLaunchArgument("object_fusion_config", default_value=object_fusion_params),
        DeclareLaunchArgument("object_fusion_target_frame", default_value="base_link"),
        DeclareLaunchArgument("enable_ground_plane", default_value="false"),
        DeclareLaunchArgument("ground_plane_config", default_value=ground_plane_params),
        DeclareLaunchArgument("stereo_disparity_topic", default_value="/x1/stereo/disparity"),
        DeclareLaunchArgument("stereo_depth_topic", default_value="/x1/stereo/depth"),
        DeclareLaunchArgument(
            "stereo_left_rect_camera_info_topic",
            default_value="/x1/stereo/left/camera_info_rect",
        ),
        DeclareLaunchArgument("stereo_points_topic", default_value="/x1/stereo/points"),
        DeclareLaunchArgument("stereo_points_status_topic", default_value="/x1/stereo/points_status"),
        DeclareLaunchArgument("pointcloud_calibration_id", default_value="unassigned"),
        DeclareLaunchArgument("pointcloud_calibration_validated", default_value="false"),
        DeclareLaunchArgument("pointcloud_min_depth_m", default_value="0.20"),
        DeclareLaunchArgument("pointcloud_max_depth_m", default_value="1.00"),
        DeclareLaunchArgument("pointcloud_publish_period_sec", default_value="0.07"),
        DeclareLaunchArgument("pointcloud_sample_step", default_value="2"),
        DeclareLaunchArgument("stereo_base_heatmap_topic", default_value="/x1/stereo/base_heatmap"),
        DeclareLaunchArgument(
            "stereo_base_heatmap_status_topic", default_value="/x1/stereo/base_heatmap_status"
        ),
        DeclareLaunchArgument("depth_coordinate_target_frame", default_value="base_link"),
        DeclareLaunchArgument("detection_boxes_topic", default_value="/x1/detection/boxes"),
        DeclareLaunchArgument("detection_boxes_status_topic", default_value="/x1/detection/boxes_status"),
        DeclareLaunchArgument("detection_debug_image_topic", default_value="/x1/detection/debug_image"),
        DeclareLaunchArgument("detection_objects_3d_topic", default_value="/x1/detection/objects_3d"),
        DeclareLaunchArgument(
            "detection_objects_3d_status_topic",
            default_value="/x1/detection/objects_3d_status",
        ),
        DeclareLaunchArgument("detector_backend", default_value="tensorrt"),
        DeclareLaunchArgument("detector_model_path", default_value=""),
        DeclareLaunchArgument("detector_image_topic", default_value="/x1/stereo/debug/left_rect"),
        DeclareLaunchArgument("detector_device", default_value="cuda:0"),
        DeclareLaunchArgument("detector_conf_threshold", default_value="0.35"),
        DeclareLaunchArgument("detector_iou_threshold", default_value="0.45"),
        DeclareLaunchArgument("detector_input_width", default_value="640"),
        DeclareLaunchArgument("detector_input_height", default_value="640"),
        DeclareLaunchArgument("detector_max_detections", default_value="20"),
        DeclareLaunchArgument("detector_publish_period_sec", default_value="0.12"),
        DeclareLaunchArgument("detector_publish_debug_image", default_value="false"),
        DeclareLaunchArgument("detector_model_id", default_value=""),
        DeclareLaunchArgument("detector_expected_model_sha256", default_value=""),
        DeclareLaunchArgument("detector_expected_class_count", default_value="80"),
        DeclareLaunchArgument("detector_expected_max_targets", default_value="0"),
        DeclareLaunchArgument("detector_duplicate_iou", default_value="0.80"),
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
        DeclareLaunchArgument("use_cpp_capture", default_value="true"),
        DeclareLaunchArgument("camera_id", default_value="imx219_stereo_pair"),
    ]

    cpp_capture = Node(
        package="deyes_capture_cpp",
        executable="imx219_stereo_capture_node",
        name="imx219_stereo_capture_node",
        condition=IfCondition(LaunchConfiguration("use_cpp_capture")),
        output="screen",
        parameters=[
            cpp_capture_params,
            {
                "width": LaunchConfiguration("width"),
                "height": LaunchConfiguration("height"),
                "fps": LaunchConfiguration("fps"),
                "target_publish_hz": LaunchConfiguration("target_publish_hz"),
                "pair_max_skew_ms": LaunchConfiguration("pair_max_skew_ms"),
                "frame_stale_sec": LaunchConfiguration("frame_stale_sec"),
                "history_size": LaunchConfiguration("history_size"),
                "output_encoding": LaunchConfiguration("output_encoding"),
                "rotate_180": LaunchConfiguration("rotate_180"),
                "mirror_horizontal": LaunchConfiguration("mirror_horizontal"),
                "swap_left_right": LaunchConfiguration("swap_left_right"),
                "left_image_topic": LaunchConfiguration("left_image_topic"),
                "right_image_topic": LaunchConfiguration("right_image_topic"),
                "left_info_topic": LaunchConfiguration("left_info_topic"),
                "right_info_topic": LaunchConfiguration("right_info_topic"),
                "calib_path": LaunchConfiguration("calib_path"),
            },
        ],
    )

    python_publisher = Node(
        package="deyes_stereo",
        executable="imx219_stereo_publisher",
        name="imx219_stereo_publisher",
        condition=UnlessCondition(LaunchConfiguration("use_cpp_capture")),
        additional_env={"LD_PRELOAD": "/lib/aarch64-linux-gnu/libgomp.so.1"},
        output="screen",
        parameters=[
            publisher_params,
            {
                "width": LaunchConfiguration("width"),
                "height": LaunchConfiguration("height"),
                "fps": LaunchConfiguration("fps"),
                "target_publish_hz": LaunchConfiguration("target_publish_hz"),
                "pair_max_skew_ms": LaunchConfiguration("pair_max_skew_ms"),
                "frame_stale_sec": LaunchConfiguration("frame_stale_sec"),
                "publish_period_sec": LaunchConfiguration("publish_period_sec"),
                "output_encoding": LaunchConfiguration("output_encoding"),
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
        condition=IfCondition(LaunchConfiguration("enable_monitor")),
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
                "expected_min_rate_hz": LaunchConfiguration("monitor_expected_min_rate_hz"),
                "hard_sync_max_ms": LaunchConfiguration("monitor_hard_sync_max_ms"),
                "soft_sync_max_ms": LaunchConfiguration("monitor_soft_sync_max_ms"),
                "allow_soft_sync": LaunchConfiguration("monitor_allow_soft_sync"),
            },
        ],
    )

    cuda_depth = Node(
        package="deyes_capture_cpp",
        executable="cuda_stereo_depth_node",
        name="cuda_stereo_depth_node",
        condition=IfCondition(LaunchConfiguration("enable_cuda_depth")),
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

    pointcloud = Node(
        package="deyes_capture_cpp",
        executable="stereo_pointcloud_node",
        name="stereo_pointcloud_node",
        condition=IfCondition(LaunchConfiguration("enable_pointcloud")),
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
                "min_depth_m": LaunchConfiguration("pointcloud_min_depth_m"),
                "max_depth_m": LaunchConfiguration("pointcloud_max_depth_m"),
                "publish_period_sec": LaunchConfiguration("pointcloud_publish_period_sec"),
                "sample_step": LaunchConfiguration("pointcloud_sample_step"),
            },
        ],
    )

    depth_coordinate = Node(
        package="deyes_stereo",
        executable="depth_coordinate",
        name="depth_coordinate_node",
        condition=IfCondition(LaunchConfiguration("enable_depth_coordinate")),
        output="screen",
        parameters=[
            LaunchConfiguration("depth_coordinate_config"),
            {
                "depth_topic": LaunchConfiguration("stereo_depth_topic"),
                "camera_info_topic": LaunchConfiguration("stereo_left_rect_camera_info_topic"),
                "target_frame": LaunchConfiguration("depth_coordinate_target_frame"),
                "heatmap_topic": LaunchConfiguration("stereo_base_heatmap_topic"),
                "status_topic": LaunchConfiguration("stereo_base_heatmap_status_topic"),
                "min_depth_m": LaunchConfiguration("cuda_depth_min_depth_m"),
                "max_depth_m": LaunchConfiguration("cuda_depth_max_depth_m"),
            },
        ],
    )

    detector = Node(
        package="deyes_stereo",
        executable="yolo_detector",
        name="yolo_detector_node",
        condition=IfCondition(LaunchConfiguration("enable_detector")),
        output="screen",
        parameters=[
            LaunchConfiguration("detector_config"),
            {
                "image_topic": LaunchConfiguration("detector_image_topic"),
                "output_topic": LaunchConfiguration("detection_boxes_topic"),
                "status_topic": LaunchConfiguration("detection_boxes_status_topic"),
                "debug_image_topic": LaunchConfiguration("detection_debug_image_topic"),
                "backend": LaunchConfiguration("detector_backend"),
                "model_path": LaunchConfiguration("detector_model_path"),
                "device": LaunchConfiguration("detector_device"),
                "conf_threshold": LaunchConfiguration("detector_conf_threshold"),
                "iou_threshold": LaunchConfiguration("detector_iou_threshold"),
                "input_width": LaunchConfiguration("detector_input_width"),
                "input_height": LaunchConfiguration("detector_input_height"),
                "max_detections": LaunchConfiguration("detector_max_detections"),
                "publish_period_sec": LaunchConfiguration("detector_publish_period_sec"),
                "publish_debug_image": LaunchConfiguration("detector_publish_debug_image"),
                "model_id": LaunchConfiguration("detector_model_id"),
                "expected_model_sha256": LaunchConfiguration("detector_expected_model_sha256"),
                "expected_class_count": LaunchConfiguration("detector_expected_class_count"),
                "expected_max_targets": LaunchConfiguration("detector_expected_max_targets"),
                "duplicate_iou": LaunchConfiguration("detector_duplicate_iou"),
            },
        ],
    )

    object_fusion = Node(
        package="deyes_stereo",
        executable="object_fusion",
        name="object_fusion_node",
        condition=IfCondition(LaunchConfiguration("enable_object_fusion")),
        output="screen",
        parameters=[
            LaunchConfiguration("object_fusion_config"),
            {
                "detection_topic": LaunchConfiguration("detection_boxes_topic"),
                "depth_topic": LaunchConfiguration("stereo_depth_topic"),
                "camera_info_topic": LaunchConfiguration("stereo_left_rect_camera_info_topic"),
                "target_frame": LaunchConfiguration("object_fusion_target_frame"),
                "output_topic": LaunchConfiguration("detection_objects_3d_topic"),
                "status_topic": LaunchConfiguration("detection_objects_3d_status_topic"),
                "min_depth_m": LaunchConfiguration("cuda_depth_min_depth_m"),
                "max_depth_m": LaunchConfiguration("cuda_depth_max_depth_m"),
            },
        ],
    )

    ground_plane = Node(
        package="deyes_stereo",
        executable="ground_plane",
        name="ground_plane_node",
        condition=IfCondition(LaunchConfiguration("enable_ground_plane")),
        output="screen",
        parameters=[
            LaunchConfiguration("ground_plane_config"),
            {
                "depth_topic": LaunchConfiguration("stereo_depth_topic"),
                "camera_info_topic": LaunchConfiguration("stereo_left_rect_camera_info_topic"),
            },
        ],
    )

    return LaunchDescription(
        launch_arguments
        + [
            cpp_capture,
            python_publisher,
            monitor,
            cuda_depth,
            pointcloud,
            depth_coordinate,
            detector,
            object_fusion,
            ground_plane,
        ]
    )
