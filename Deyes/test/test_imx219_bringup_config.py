from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "config" / "stereo" / "imx219_capture_cpp.yaml"
CUDA_DEPTH_CONFIG_PATH = ROOT / "config" / "stereo" / "cuda_depth.defaults.yaml"
POINTCLOUD_CONFIG_PATH = ROOT / "config" / "stereo" / "pointcloud.defaults.yaml"
DEPTH_COORDINATE_CONFIG_PATH = ROOT / "config" / "stereo" / "depth_coordinate.defaults.yaml"
YOLO_DETECTOR_CONFIG_PATH = ROOT / "config" / "stereo" / "yolo_detector.defaults.yaml"
OBJECT_FUSION_CONFIG_PATH = ROOT / "config" / "stereo" / "object_fusion.defaults.yaml"
GROUND_PLANE_CONFIG_PATH = ROOT / "config" / "stereo" / "ground_plane.defaults.yaml"
LAUNCH_PATH = ROOT / "src" / "deyes_bringup" / "launch" / "imx219_stereo.launch.py"
CUDA_DEPTH_LAUNCH_PATH = ROOT / "src" / "deyes_bringup" / "launch" / "cuda_depth.launch.py"
POINTCLOUD_LAUNCH_PATH = ROOT / "src" / "deyes_bringup" / "launch" / "pointcloud.launch.py"
DEPTH_COORDINATE_NODE_PATH = (
    ROOT / "src" / "deyes_stereo" / "deyes_stereo" / "depth_coordinate_node.py"
)
OBJECT_FUSION_NODE_PATH = ROOT / "src" / "deyes_stereo" / "deyes_stereo" / "object_fusion_node.py"
YOLO_DETECTOR_NODE_PATH = ROOT / "src" / "deyes_stereo" / "deyes_stereo" / "yolo_detector_node.py"
GROUND_PLANE_NODE_PATH = ROOT / "src" / "deyes_stereo" / "deyes_stereo" / "ground_plane_node.py"
CUDA_DEPTH_NODE_PATH = ROOT / "src" / "deyes_capture_cpp" / "src" / "cuda_stereo_depth_node.cpp"
CUDA_DEPTH_CMAKE_PATH = ROOT / "src" / "deyes_capture_cpp" / "CMakeLists.txt"
POINTCLOUD_NODE_PATH = ROOT / "src" / "deyes_capture_cpp" / "src" / "stereo_pointcloud_node.cpp"
POINTCLOUD_HELPER_PATH = ROOT / "src" / "deyes_capture_cpp" / "include" / "deyes_capture_cpp" / "depth_projection.hpp"
POINTCLOUD_CONTRACT_PATH = ROOT / "src" / "deyes_capture_cpp" / "include" / "deyes_capture_cpp" / "stereo_pair_contract.hpp"
SETUP_PY_PATH = ROOT / "src" / "deyes_stereo" / "setup.py"
BRINGUP_SETUP_PATH = ROOT / "src" / "deyes_bringup" / "setup.py"
DEBUG_CALIB_PATH = ROOT / "config" / "camera" / "stereo_calib.yaml"


def test_cpp_capture_config_exists() -> None:
    assert CONFIG_PATH.is_file()


def test_launch_defaults_to_cpp_capture() -> None:
    content = LAUNCH_PATH.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("use_cpp_capture", default_value="true")' in content
    assert 'DeclareLaunchArgument("rotate_180", default_value="true")' in content
    assert 'DeclareLaunchArgument("mirror_horizontal", default_value="true")' in content
    assert 'DeclareLaunchArgument("swap_left_right", default_value="false")' in content
    assert 'package="deyes_capture_cpp"' in content
    assert 'executable="imx219_stereo_capture_node"' in content


def test_bringup_owns_portable_debug_calibration() -> None:
    launch_content = LAUNCH_PATH.read_text(encoding="utf-8")
    setup_content = BRINGUP_SETUP_PATH.read_text(encoding="utf-8")
    config_contents = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (ROOT / "config" / "stereo").glob("*.yaml")
    )

    assert DEBUG_CALIB_PATH.is_file()
    assert 'pkg_share / "config" / "camera" / "stereo_calib.yaml"' in launch_content
    assert '"share/" + package_name + "/config/camera"' in setup_content
    assert "/home/elephant/mercury_grasp" not in launch_content
    assert "/home/elephant/mercury_grasp" not in config_contents


def test_cpp_capture_config_enables_rotation() -> None:
    content = CONFIG_PATH.read_text(encoding="utf-8")
    assert "rotate_180: true" in content
    assert "mirror_horizontal: true" in content
    assert "swap_left_right: false" in content


def test_cpp_capture_watchdog_is_exposed_with_conservative_defaults() -> None:
    config_content = CONFIG_PATH.read_text(encoding="utf-8")
    launch_content = LAUNCH_PATH.read_text(encoding="utf-8")
    node_content = (
        ROOT / "src" / "deyes_capture_cpp" / "src" / "imx219_stereo_capture_node.cpp"
    ).read_text(encoding="utf-8")

    assert "capture_stall_sec: 2.0" in config_content
    assert "capture_startup_grace_sec: 8.0" in config_content
    assert 'DeclareLaunchArgument("capture_stall_sec", default_value="2.0")' in launch_content
    assert 'DeclareLaunchArgument("capture_startup_grace_sec", default_value="8.0")' in launch_content
    assert "respawn=True" in launch_content
    assert "respawn_delay=5.0" in launch_content
    assert "fail_fast_if_capture_stalled();" in node_content
    assert "pull_timeout_count_" in node_content
    assert '"capture_stall_recovery"' in node_content
    assert "std::_Exit(75);" in node_content


def test_cuda_depth_defaults_prioritize_stable_output() -> None:
    content = CUDA_DEPTH_CONFIG_PATH.read_text(encoding="utf-8")
    assert "max_sync_diff_ms: 10.0" in content
    assert "publish_period_sec: 0.07" in content
    assert "min_depth_m: 0.20" in content
    assert "max_depth_m: 1.00" in content
    assert "enable_wls_filter: false" in content
    assert "wls_lambda: 8000.0" in content
    assert "wls_sigma_color: 2.0" in content
    assert "block_size: 11" in content
    assert "texture_threshold: 0" in content
    assert "uniqueness_ratio: 0" in content
    assert "speckle_window_size: 0" in content
    assert "speckle_range: 0" in content
    assert "disp12_max_diff: 0" in content
    assert "median_ksize: 3" in content
    assert "publish_debug_rect: false" in content
    assert "publish_debug_mask: false" in content
    assert 'left_rect_camera_info_topic: "/x1/stereo/left/camera_info_rect"' in content


def test_cuda_depth_launch_defaults_are_consistent() -> None:
    integrated = LAUNCH_PATH.read_text(encoding="utf-8")
    standalone = CUDA_DEPTH_LAUNCH_PATH.read_text(encoding="utf-8")

    expected_arguments = [
        'DeclareLaunchArgument("cuda_depth_max_sync_diff_ms", default_value="10.0")',
        'DeclareLaunchArgument("cuda_depth_publish_period_sec", default_value="0.07")',
        'DeclareLaunchArgument("cuda_depth_min_depth_m", default_value="0.20")',
        'DeclareLaunchArgument("cuda_depth_max_depth_m", default_value="1.00")',
        'DeclareLaunchArgument("cuda_depth_enable_wls_filter", default_value="false")',
        'DeclareLaunchArgument("cuda_depth_wls_lambda", default_value="8000.0")',
        'DeclareLaunchArgument("cuda_depth_wls_sigma_color", default_value="2.0")',
        'DeclareLaunchArgument("cuda_depth_texture_threshold", default_value="0")',
        'DeclareLaunchArgument("cuda_depth_uniqueness_ratio", default_value="0")',
        'DeclareLaunchArgument("cuda_depth_speckle_window_size", default_value="0")',
        'DeclareLaunchArgument("cuda_depth_speckle_range", default_value="0")',
        'DeclareLaunchArgument("cuda_depth_disp12_max_diff", default_value="0")',
        'DeclareLaunchArgument("cuda_depth_publish_debug_rect", default_value="false")',
        'DeclareLaunchArgument("cuda_depth_publish_debug_mask", default_value="false")',
    ]
    for expected in expected_arguments:
        assert expected in integrated
        assert expected in standalone


def test_pointcloud_defaults_and_launch_are_explicitly_debug_only() -> None:
    config_content = POINTCLOUD_CONFIG_PATH.read_text(encoding="utf-8")
    launch_content = LAUNCH_PATH.read_text(encoding="utf-8")
    standalone = POINTCLOUD_LAUNCH_PATH.read_text(encoding="utf-8")
    node_content = POINTCLOUD_NODE_PATH.read_text(encoding="utf-8")
    helper_content = POINTCLOUD_HELPER_PATH.read_text(encoding="utf-8")
    contract_content = POINTCLOUD_CONTRACT_PATH.read_text(encoding="utf-8")
    cmake_content = CUDA_DEPTH_CMAKE_PATH.read_text(encoding="utf-8")

    assert "calibration_validated: false" in config_content
    assert 'depth_topic: "/x1/stereo/depth"' in config_content
    assert 'rectified_camera_info_topic: "/x1/stereo/left/camera_info_rect"' in config_content
    assert 'points_topic: "/x1/stereo/points"' in config_content
    assert 'status_topic: "/x1/stereo/points_status"' in config_content
    assert "sample_step: 2" in config_content
    assert 'DeclareLaunchArgument("enable_pointcloud", default_value="false")' in launch_content
    assert 'DeclareLaunchArgument("pointcloud_config", default_value=pointcloud_params)' in launch_content
    assert 'executable="stereo_pointcloud_node"' in launch_content
    assert '"rectified_camera_info_topic": LaunchConfiguration(' in launch_content
    assert '"stereo_left_rect_camera_info_topic"' in launch_content
    assert 'executable="stereo_pointcloud_node"' in standalone
    assert "rclcpp::SensorDataQoS()" in node_content
    assert "validate_stereo_pair_contract(contract_input)" in node_content
    assert 'input.depth_encoding != "32FC1"' in contract_content
    assert "input.depth_stamp_ns != input.camera_info_stamp_ns" in contract_content
    assert "debug_rviz_only" in node_content
    assert "calibration_validated=true requires a non-empty calibration_id" in node_content
    assert "sensor_msgs::msg::PointField::FLOAT32" in node_content
    assert "make_point_field" in node_content
    assert "make_key_value" in node_content
    assert "sensor_msgs::msg::PointField{" not in node_content
    assert "diagnostic_msgs::msg::KeyValue{" not in node_content
    assert "project_depth_pixel" in node_content
    assert '"no_valid_points"' in node_content
    assert "has_valid_points(valid_points)" in node_content
    assert "organized_cloud_layout" in helper_content
    assert "stereo_pointcloud_node" in cmake_content
    assert "ament_add_gtest(test_depth_projection" in cmake_content


def test_depth_coordinate_launch_and_defaults_exist() -> None:
    launch_content = LAUNCH_PATH.read_text(encoding="utf-8")
    config_content = DEPTH_COORDINATE_CONFIG_PATH.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("enable_depth_coordinate", default_value="false")' in launch_content
    assert 'DeclareLaunchArgument("depth_coordinate_target_frame", default_value="base_link")' in launch_content
    assert 'DeclareLaunchArgument("stereo_base_heatmap_topic", default_value="/x1/stereo/base_heatmap")' in launch_content
    assert 'executable="depth_coordinate"' in launch_content

    assert "target_frame: \"base_link\"" in config_content
    assert "heatmap_topic: \"/x1/stereo/base_heatmap\"" in config_content
    assert "status_topic: \"/x1/stereo/base_heatmap_status\"" in config_content
    assert "sample_step: 2" in config_content
    assert "max_points: 4000" in config_content


def test_depth_coordinate_node_supports_runtime_tuning_updates() -> None:
    content = DEPTH_COORDINATE_NODE_PATH.read_text(encoding="utf-8")

    assert "self.add_on_set_parameters_callback(self._on_parameters_set)" in content
    assert 'normalize_vec3(manual_translation_m, "manual_translation_m")' in content
    assert 'normalize_vec3(manual_rpy_deg, "manual_rpy_deg")' in content
    assert 'if parameter.name in {"manual_translation_m", "manual_rpy_deg"}:' in content
    assert 'elif parameter.name == "sample_step":' in content
    assert 'elif parameter.name == "max_points":' in content


def test_object_fusion_launch_and_defaults_exist() -> None:
    launch_content = LAUNCH_PATH.read_text(encoding="utf-8")
    config_content = OBJECT_FUSION_CONFIG_PATH.read_text(encoding="utf-8")
    node_content = OBJECT_FUSION_NODE_PATH.read_text(encoding="utf-8")
    setup_content = SETUP_PY_PATH.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("enable_object_fusion", default_value="false")' in launch_content
    assert 'DeclareLaunchArgument("object_fusion_config", default_value=object_fusion_params)' in launch_content
    assert 'DeclareLaunchArgument("object_fusion_target_frame", default_value="base_link")' in launch_content
    assert 'DeclareLaunchArgument("detection_boxes_topic", default_value="/x1/detection/boxes")' in launch_content
    assert 'DeclareLaunchArgument("detection_objects_3d_topic", default_value="/x1/detection/objects_3d")' in launch_content
    assert '"detection_objects_3d_status_topic"' in launch_content
    assert 'default_value="/x1/detection/objects_3d_status"' in launch_content
    assert 'executable="object_fusion"' in launch_content

    assert 'detection_topic: "/x1/detection/boxes"' in config_content
    assert 'output_topic: "/x1/detection/objects_3d"' in config_content
    assert 'status_topic: "/x1/detection/objects_3d_status"' in config_content
    assert 'target_frame: "base_link"' in config_content

    assert "object_fusion = deyes_stereo.object_fusion_node:main" in setup_content
    assert "class ObjectFusionNode(Node):" in node_content
    assert 'self.create_subscription(' in node_content
    assert '"detection_topic": "/x1/detection/boxes"' in node_content
    assert '"output_topic": "/x1/detection/objects_3d"' in node_content


def test_yolo_detector_launch_and_defaults_exist() -> None:
    launch_content = LAUNCH_PATH.read_text(encoding="utf-8")
    config_content = YOLO_DETECTOR_CONFIG_PATH.read_text(encoding="utf-8")
    node_content = YOLO_DETECTOR_NODE_PATH.read_text(encoding="utf-8")
    setup_content = SETUP_PY_PATH.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("enable_detector", default_value="false")' in launch_content
    assert 'DeclareLaunchArgument("detector_config", default_value=yolo_detector_params)' in launch_content
    assert 'DeclareLaunchArgument("detection_boxes_status_topic", default_value="/x1/detection/boxes_status")' in launch_content
    assert 'DeclareLaunchArgument("detection_debug_image_topic", default_value="/x1/detection/debug_image")' in launch_content
    assert 'DeclareLaunchArgument("detector_backend", default_value="tensorrt")' in launch_content
    assert 'DeclareLaunchArgument("detector_model_path", default_value="")' in launch_content
    assert 'executable="yolo_detector"' in launch_content

    assert 'image_topic: "/x1/stereo/debug/left_rect"' in config_content
    assert 'output_topic: "/x1/detection/boxes"' in config_content
    assert 'status_topic: "/x1/detection/boxes_status"' in config_content
    assert 'debug_image_topic: "/x1/detection/debug_image"' in config_content
    assert 'backend: "tensorrt"' in config_content
    assert 'device: "cuda:0"' in config_content
    assert 'publish_debug_image: false' in config_content

    assert "yolo_detector = deyes_stereo.yolo_detector_node:main" in setup_content
    assert "class YoloDetectorNode(Node):" in node_content
    assert '"output_topic": "/x1/detection/boxes"' in node_content
    assert '"status_topic": "/x1/detection/boxes_status"' in node_content
    assert 'from ultralytics import YOLO' in node_content
    assert 'if self._backend_name == "opencv_dnn":' in node_content
    assert 'if self._backend_name == "tensorrt":' in node_content
    assert "cv2.dnn.readNetFromONNX" in node_content
    assert "import tensorrt as trt" in node_content
    assert "torch.cuda.is_available()" in node_content
    assert "execute_v2(bindings)" in node_content
    assert "self._model.predict(" in node_content


def test_cuda_depth_node_enables_wls_and_quality_metrics() -> None:
    content = CUDA_DEPTH_NODE_PATH.read_text(encoding="utf-8")
    cmake_content = CUDA_DEPTH_CMAKE_PATH.read_text(encoding="utf-8")

    assert 'declare_parameter<bool>("enable_wls_filter", true)' in content
    assert 'declare_parameter<double>("wls_lambda", 8000.0)' in content
    assert 'declare_parameter<double>("wls_sigma_color", 2.0)' in content
    assert 'restart_wls_filter();' in content
    assert "compute_filtered_disparity(" in content
    assert "compute_depth_quality_metrics(" in content
    assert "valid_ratio_1m" in content
    assert "coverage_ratio_center_roi" in content
    assert "p95_processing_ms" in content
    assert "ximgproc" in cmake_content


def test_depth_geometry_uses_cuda_rectified_camera_info() -> None:
    content = CUDA_DEPTH_NODE_PATH.read_text(encoding="utf-8")
    integrated = LAUNCH_PATH.read_text(encoding="utf-8")

    assert '"/x1/stereo/left/camera_info_rect"' in content
    assert '"stereo_left_rect_camera_info_topic"' in integrated
    assert '"camera_info_topic": LaunchConfiguration("stereo_left_rect_camera_info_topic")' in integrated


def test_ground_plane_launch_and_defaults_exist() -> None:
    launch_content = LAUNCH_PATH.read_text(encoding="utf-8")
    config_content = GROUND_PLANE_CONFIG_PATH.read_text(encoding="utf-8")
    node_content = GROUND_PLANE_NODE_PATH.read_text(encoding="utf-8")
    setup_content = SETUP_PY_PATH.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("enable_ground_plane", default_value="false")' in launch_content
    assert 'DeclareLaunchArgument("ground_plane_config", default_value=ground_plane_params)' in launch_content
    assert 'DeclareLaunchArgument("ground_plane_publish_debug_tf", default_value="false")' in launch_content
    assert 'executable="ground_plane"' in launch_content

    assert 'dynamic_plane_frame: "table_plane_dynamic_debug"' in config_content
    assert 'depth_topic: "/x1/stereo/depth"' in config_content
    assert 'camera_info_topic: "/x1/stereo/left/camera_info_rect"' in config_content
    assert 'ransac_distance_threshold: 0.02' in config_content
    assert 'publish_debug_tf: false' in config_content

    assert "ground_plane = deyes_stereo.ground_plane_node:main" in setup_content
    assert "class GroundPlaneNode(Node):" in node_content
    assert "validate_rectified_depth_pair" in node_content
    assert "project_rectified_depth_pixels" in node_content
    assert "dynamic_table_plane_camera_relative_only" in node_content
    assert "normal_discontinuity_fallback" in node_content
    assert '"valid_for_table_removal": not degraded' in node_content
    assert "TransformBroadcaster" in node_content
    assert "sendTransform" in node_content


def test_pen_grasp_requires_rectified_pair_and_fresh_plane_contract() -> None:
    pen_node = (ROOT / "src" / "deyes_stereo" / "deyes_stereo" / "pen_grasp_node.py").read_text(encoding="utf-8")
    assert 'camera_info_topic: "/x1/stereo/left/camera_info_rect"' in (ROOT / "config" / "stereo" / "pen_grasp.defaults.yaml").read_text(encoding="utf-8")
    assert "validate_rectified_depth_pair" in pen_node
    assert "validate_dynamic_plane_for_depth" in pen_node
    assert "rectified_intrinsics(info.p)" in pen_node
    assert ".k" not in pen_node
    assert "ExactStampPairCache" in pen_node
    assert "_unmatched_planes" in pen_node
    assert "_ready.pop_oldest" in pen_node


def test_ground_plane_uses_exact_pair_cache_before_timer_processing() -> None:
    ground_node = GROUND_PLANE_NODE_PATH.read_text(encoding="utf-8")
    assert "ExactStampPairCache" in ground_node
    assert "_pending_pairs.pop_newest" in ground_node
    assert "waiting_for_exact_rectified_depth_camera_info_pair" in ground_node
