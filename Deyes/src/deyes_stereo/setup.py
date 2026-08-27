from pathlib import Path

from setuptools import find_packages, setup

package_name = "deyes_stereo"
# colcon requires data_files sources to stay relative to this setup.py package.
tools_dir = Path("..") / ".." / "tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/tools", [
            str(tools_dir / "launch_stereo_calibration_gui.sh"),
            str(tools_dir / "install_stereo_calibration_desktop.sh"),
            str(tools_dir / "mercury-x1-stereo-calibration.desktop"),
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TRAE Agent",
    maintainer_email="devnull@example.com",
    description="Deyes stereo synchronization and quality monitoring nodes.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sync_monitor = deyes_stereo.sync_monitor_node:main",
            "imx219_stereo_publisher = deyes_stereo.imx219_stereo_publisher:main",
            "sgbm_baseline = deyes_stereo.sgbm_baseline_node:main",
            "depth_coordinate = deyes_stereo.depth_coordinate_node:main",
            "yolo_detector = deyes_stereo.yolo_detector_node:main",
            "object_fusion = deyes_stereo.object_fusion_node:main",
            "ground_plane = deyes_stereo.ground_plane_node:main",
            "validated_extrinsics_tf = deyes_stereo.validated_extrinsics_tf_node:main",
            "coordinate_chain_tf2 = deyes_stereo.coordinate_chain_tf2_node:main",
            "tf_chain_audit = deyes_stereo.tf_chain_audit_node:main",
            "coordinate_chain_candidate_bridge = deyes_stereo.coordinate_chain_candidate_bridge_node:main",
            "tf_frame_probe = deyes_stereo.tf_frame_probe_node:main",
            "handeye_calibration = deyes_stereo.handeye_calibration:main",
            "handeye_multiview_contract = deyes_stereo.handeye_multiview_contract:main",
            "pen_grasp = deyes_stereo.pen_grasp_node:main",
            "vision_grasp_candidate = deyes_stereo.vision_grasp_candidate_node:main",
            "pen_pick_dry_run = deyes_stereo.pen_pick_dry_run_node:main",
            "pick_ros2_execution = deyes_stereo.pick_ros2_execution_node:main",
            "motion_interface_probe = deyes_stereo.motion_interface_probe_node:main",
            "pen_feature = deyes_stereo.pen_feature_node:main",
            "physical_stereo_calibration = deyes_stereo.physical_stereo_calibration:main",
            "stereo_calibration_gui = deyes_stereo.stereo_calibration_gui:main",
            "stereo_acceptance = deyes_stereo.stereo_acceptance:main",
            "runtime_acceptance_monitor = deyes_stereo.runtime_acceptance_monitor:main",
            "pen_dataset_capture = deyes_stereo.pen_dataset_capture_node:main",
            "sim_dual_pen_candidate = deyes_stereo.sim_dual_pen_candidate_node:main",
            "single_shot_snapshot = deyes_stereo.single_shot_snapshot_node:main",
            "single_shot_pick_planner = deyes_stereo.single_shot_pick_planner_node:main",
            "mercury_right_arm_action_server = deyes_stereo.mercury_right_arm_action_server:main",
            "single_shot_pick_executor = deyes_stereo.single_shot_pick_executor_node:main",
            "isaac_right_arm_stage_executor = deyes_stereo.isaac_right_arm_stage_executor_node:main",
            "isaac_single_pen_candidate = deyes_stereo.isaac_single_pen_candidate_node:main",
            "pick_nav_coordinator = deyes_stereo.pick_nav_coordinator_node:main",
            "competition_perception_gate = deyes_stereo.competition_perception_gate_node:main",
            "competition_pick_target = deyes_stereo.competition_pick_target_node:main",
        ],
    },
)
