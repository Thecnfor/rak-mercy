from setuptools import find_packages, setup

package_name = "deyes_stereo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
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
            "handeye_calibration = deyes_stereo.handeye_calibration:main",
            "pen_grasp = deyes_stereo.pen_grasp_node:main",
            "pen_pick_dry_run = deyes_stereo.pen_pick_dry_run_node:main",
            "motion_interface_probe = deyes_stereo.motion_interface_probe_node:main",
            "pen_feature = deyes_stereo.pen_feature_node:main",
            "physical_stereo_calibration = deyes_stereo.physical_stereo_calibration:main",
            "stereo_acceptance = deyes_stereo.stereo_acceptance:main",
            "runtime_acceptance_monitor = deyes_stereo.runtime_acceptance_monitor:main",
            "pen_dataset_capture = deyes_stereo.pen_dataset_capture_node:main",
            "sim_dual_pen_candidate = deyes_stereo.sim_dual_pen_candidate_node:main",
        ],
    },
)
