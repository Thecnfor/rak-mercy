from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("solver_type", default_value="ikpy",
                              description="'ikpy' uses the URDF solver; 'mock' always returns a fixed pose."),
        DeclareLaunchArgument("arm_sides", default_value="[right, left]",
                              description="JSON-style list of arms to load solvers for."),
        DeclareLaunchArgument("joint_state_topic", default_value="/joint_states"),
        DeclareLaunchArgument("publish_result_topic", default_value="/x1/ik/last_solution"),
        Node(
            package="deyes_ik_server",
            executable="ik_action_server",
            name="execute_cartesian_stage_ik",
            output="screen",
            parameters=[{
                "solver_type": LaunchConfiguration("solver_type"),
                "arm_sides": LaunchConfiguration("arm_sides"),
                "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                "publish_result_topic": LaunchConfiguration("publish_result_topic"),
            }],
        ),
    ])