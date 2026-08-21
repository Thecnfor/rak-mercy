from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_dir=Path(get_package_share_directory("deyes_bringup"))/"config"
    params=str(config_dir/"single_shot_pick.defaults.yaml")
    common={"dry_run":LaunchConfiguration("dry_run"),"enable_live_execution":LaunchConfiguration("enable_live_execution"),"autonomous_once":LaunchConfiguration("autonomous_once")}
    return LaunchDescription([
        DeclareLaunchArgument("arm_side",default_value="right"),
        DeclareLaunchArgument("autonomous_once",default_value="true"),
        DeclareLaunchArgument("dry_run",default_value="true"),
        DeclareLaunchArgument("enable_live_execution",default_value="false"),
        DeclareLaunchArgument("operator_confirmed",default_value="false"),
        DeclareLaunchArgument("expected_target_count",default_value="1"),
        DeclareLaunchArgument("model_path",default_value=""),DeclareLaunchArgument("model_id",default_value=""),DeclareLaunchArgument("model_sha256",default_value=""),
        DeclareLaunchArgument("site_profile_path",default_value=""),DeclareLaunchArgument("stereo_calibration_path",default_value=""),DeclareLaunchArgument("extrinsics_path",default_value=""),
        DeclareLaunchArgument("log_root",default_value=""),
        Node(package="deyes_stereo",executable="single_shot_snapshot",name="single_shot_snapshot_node",output="screen",parameters=[params,common,{"log_root":LaunchConfiguration("log_root")}]),
        Node(package="deyes_stereo",executable="yolo_detector",name="yolo_detector_node",output="screen",parameters=[params,{"model_path":LaunchConfiguration("model_path"),"model_id":LaunchConfiguration("model_id"),"expected_model_sha256":LaunchConfiguration("model_sha256"),"expected_max_targets":LaunchConfiguration("expected_target_count")}]),
        Node(package="deyes_stereo",executable="pen_feature",name="pen_feature_node",output="screen",parameters=[params]),
        Node(package="deyes_stereo",executable="vision_grasp_candidate",name="vision_grasp_candidate_node",output="screen",parameters=[params]),
        Node(package="deyes_stereo",executable="validated_extrinsics_tf",name="validated_extrinsics_tf_node",output="screen",parameters=[{"extrinsics_path":LaunchConfiguration("extrinsics_path"),"stereo_calibration_path":LaunchConfiguration("stereo_calibration_path")}]),
        Node(package="deyes_stereo",executable="coordinate_chain_tf2",name="coordinate_chain_tf2_node",output="screen"),
        Node(package="deyes_stereo",executable="coordinate_chain_candidate_bridge",name="coordinate_chain_candidate_bridge_node",output="screen",parameters=[params]),
        Node(package="deyes_stereo",executable="single_shot_pick_planner",name="single_shot_pick_planner_node",output="screen",parameters=[params,{"site_profile_path":LaunchConfiguration("site_profile_path")}]),
        Node(package="deyes_stereo",executable="mercury_right_arm_action_server",name="mercury_right_arm_action_server",output="screen",parameters=[params,common,{"operator_confirmed":LaunchConfiguration("operator_confirmed"),"site_profile_path":LaunchConfiguration("site_profile_path"),"stereo_calibration_path":LaunchConfiguration("stereo_calibration_path"),"extrinsics_path":LaunchConfiguration("extrinsics_path")}]),
        Node(package="deyes_stereo",executable="single_shot_pick_executor",name="single_shot_pick_executor_node",output="screen",parameters=[params,common,{"operator_confirmed":LaunchConfiguration("operator_confirmed"),"arm_side":LaunchConfiguration("arm_side"),"log_root":LaunchConfiguration("log_root")}]),
    ])
