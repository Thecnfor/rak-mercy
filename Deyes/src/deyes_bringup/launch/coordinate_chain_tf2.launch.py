from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = str(Path(get_package_share_directory("deyes_bringup")) / "config" / "coordinate_chain_tf2.defaults.yaml")
    return LaunchDescription([
        Node(package="deyes_stereo", executable="coordinate_chain_tf2", name="coordinate_chain_tf2_node", output="screen", parameters=[params]),
        Node(package="deyes_stereo", executable="tf_chain_audit", name="tf_chain_audit_node", output="screen", parameters=[params]),
    ])
