from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = str(Path(get_package_share_directory("deyes_bringup")) / "config" / "validated_extrinsics_tf.defaults.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("extrinsics_path", default_value=""),
        DeclareLaunchArgument("stereo_calibration_path", default_value=""),
        Node(package="deyes_stereo", executable="validated_extrinsics_tf", name="validated_extrinsics_tf_node", output="screen", parameters=[params, {
            "extrinsics_path": LaunchConfiguration("extrinsics_path"),
            "stereo_calibration_path": LaunchConfiguration("stereo_calibration_path"),
        }]),
    ])
