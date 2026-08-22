"""ROS2 execution adapter, disabled by default and safe without a robot.

It listens for a trusted coordinate-chain result and a dry-run plan.  Live
goals can only be submitted one stage at a time after the three confirmation
gates, action-server/joint-feedback checks, and a per-stage confirmation.
"""

from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .pick_ros2_execution_contract import LiveExecutionGates, validate_execution_admission, validate_single_step


class PickRos2ExecutionNode(Node):
    def __init__(self) -> None:
        super().__init__("pick_ros2_execution_node")
        for name, value in {
            "plan_topic": "/x1/pick/dry_run_plan", "coordinate_result_topic": "/x1/coordinate_chain/result",
            "step_confirmation_topic": "/x1/pick/step_confirmation", "status_topic": "/x1/pick/execution_status",
            "joint_state_topic": "/joint_states", "follow_joint_trajectory_action": "/x1/arm_controller/follow_joint_trajectory",
            "gripper_set_bool_service": "/x1/gripper/close", "dry_run": True, "enable_live_execution": False,
            "operator_confirmed": False, "validated_calibration": False, "max_joint_state_age_sec": .25,
            "step_timeout_sec": 8.0,
        }.items(): self.declare_parameter(name, value)
        self._coordinate: dict[str, Any] | None = None; self._plan: dict[str, Any] | None = None; self._joint_stamp_ns = 0
        self._active_goal: Any = None
        self._status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), qos_profile_sensor_data)
        self.create_subscription(String, str(self.get_parameter("coordinate_result_topic").value), self._on_coordinate, qos_profile_sensor_data)
        self.create_subscription(String, str(self.get_parameter("plan_topic").value), self._on_plan, qos_profile_sensor_data)
        self.create_subscription(String, str(self.get_parameter("step_confirmation_topic").value), self._on_confirmation, qos_profile_sensor_data)
        self.create_subscription(JointState, str(self.get_parameter("joint_state_topic").value), self._on_joint_state, qos_profile_sensor_data)
        self._trajectory = ActionClient(self, FollowJointTrajectory, str(self.get_parameter("follow_joint_trajectory_action").value))
        self._gripper = self.create_client(SetBool, str(self.get_parameter("gripper_set_bool_service").value))
        self.get_logger().warn("pick execution adapter starts dry_run=true; no goal is sent until all live gates pass")

    def _publish(self, **payload: Any) -> None:
        message = String(); message.data = json.dumps({"hardware_commands_emitted": False, **payload}, separators=(",", ":")); self._status_pub.publish(message)

    def _on_coordinate(self, msg: String) -> None:
        try: self._coordinate = json.loads(msg.data)
        except json.JSONDecodeError: self._coordinate = None

    def _on_plan(self, msg: String) -> None:
        try: self._plan = json.loads(msg.data)
        except json.JSONDecodeError: self._plan = None
        self._publish(state="awaiting_step_confirmation", reason="dry_run_plan_received")

    def _on_joint_state(self, msg: JointState) -> None:
        self._joint_stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _gates(self) -> LiveExecutionGates:
        return LiveExecutionGates(dry_run=bool(self.get_parameter("dry_run").value), enable_live_execution=bool(self.get_parameter("enable_live_execution").value), operator_confirmed=bool(self.get_parameter("operator_confirmed").value), validated_calibration=bool(self.get_parameter("validated_calibration").value), action_server_available=self._trajectory.server_is_ready(), joint_state_stamp_ns=self._joint_stamp_ns, now_stamp_ns=time.time_ns(), max_joint_state_age_sec=float(self.get_parameter("max_joint_state_age_sec").value))

    def _on_confirmation(self, msg: String) -> None:
        try: command = json.loads(msg.data); step = command.get("step"); confirmed = command.get("confirmed") is True
        except (json.JSONDecodeError, AttributeError): self._publish(state="rejected", reason="step_confirmation_invalid"); return
        allowed, reason = validate_execution_admission(self._coordinate, self._plan, self._gates())
        if not allowed: self._publish(state="rejected", step=step, reason=reason); return
        allowed, reason = validate_single_step(self._plan, step, operator_step_confirmed=confirmed)
        if not allowed: self._publish(state="rejected", step=step, reason=reason); return
        # Dispatch is deliberately still inhibited: a live adapter must add an
        # independently reviewed collision/tracking monitor before this line is
        # replaced.  The clients above are only interface/availability probes.
        self._publish(state="rejected", step=step, reason="live_dispatch_requires_collision_tracking_adapter")

    def cancel_active_goal(self) -> None:
        """Cancellation hook for the future dispatcher; harmless with no goal."""
        if self._active_goal is not None: self._active_goal.cancel_goal_async()


def main(args: Any = None) -> None:
    rclpy.init(args=args); node = PickRos2ExecutionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.cancel_active_goal()
    finally: node.destroy_node(); rclpy.shutdown()
