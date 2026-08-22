"""Simulation-only JointState stage executor; never opens a robot serial port."""
from __future__ import annotations
import json
import time
from typing import Any
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from .isaac_single_pen_contract import PHASES, SCHEMA, evaluate_pen_lift, stage_feedback_converged
from .isaac_right_arm_ik_contract import (
    RIGHT_ARM_JOINT_NAMES,
    RIGHT_GRIPPER_ROOT_JOINT_NAMES,
    RIGHT_GRIPPER_ROOT_SIGNS,
    RIGHT_GRIPPER_SAFE_TRAVEL_RAD,
    build_sparse_right_arm_command,
    build_sparse_right_gripper_command,
)


class IsaacRightArmStageExecutor(Node):
    def __init__(self) -> None:
        super().__init__("isaac_right_arm_stage_executor")
        defaults = {"plan_topic": "/x1_sim/grasp/single_pen_plan", "ros_domain_id": 46, "joint_command_topic": "/x1_sim/joint_command", "gripper_command_topic": "/x1_sim/gripper_command", "joint_state_topic": "/x1_sim/joint_states", "gripper_state_topic": "/x1_sim/gripper_joint_states", "pen_pose_topic": "/x1_sim/pen_pose", "status_topic": "/x1_sim/grasp/motion_status", "trace_topic": "/x1_sim/grasp/motion_trace", "right_joint_names": [], "right_gripper_feedback_names": [], "right_gripper_command_names": [], "enable_execution": False, "simulation_only": False, "motion_enabled": False, "feedback_timeout_sec": .5, "stage_timeout_sec": 8., "joint_tolerance_rad": .03, "gripper_tolerance": .02, "pen_lift_threshold_m": .03}
        for key, value in defaults.items(): self.declare_parameter(key, value)
        self._joint_pub = self.create_publisher(JointState, str(self.get_parameter("joint_command_topic").value), 10)
        self._gripper_pub = self.create_publisher(JointState, str(self.get_parameter("gripper_command_topic").value), 10)
        self._status = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)
        self._trace = self.create_publisher(String, str(self.get_parameter("trace_topic").value), 10)
        self.create_subscription(String, str(self.get_parameter("plan_topic").value), self._on_plan, 10)
        self.create_subscription(JointState, str(self.get_parameter("joint_state_topic").value), self._on_joint, 10)
        self.create_subscription(JointState, str(self.get_parameter("gripper_state_topic").value), self._on_gripper, 10)
        self.create_subscription(String, str(self.get_parameter("pen_pose_topic").value), self._on_pen_pose, 10)
        self._plan: dict[str, Any] | None = None; self._joint_stamp = 0.; self._joint_values: dict[str, float] = {}; self._gripper_stamp = 0.; self._gripper_value: float | None = None; self._before_z: float | None = None; self._after_z: float | None = None; self._done = False; self._index = 0; self._waiting = False; self._stage_started = 0.; self._timer = self.create_timer(.05, self._tick)

    def _emit(self, state: str, reason: str, **extra: Any) -> None:
        payload = {"schema": SCHEMA, "state": state, "reason": reason, "commands_emitted": False, **extra}
        msg = String(); msg.data = json.dumps(payload, separators=(",", ":")); self._status.publish(msg); self._trace.publish(msg)

    def _on_joint(self, message: JointState) -> None:
        self._joint_stamp = time.monotonic()
        self._joint_values = {str(name): float(value) for name, value in zip(message.name, message.position)}

    def _on_gripper(self, message: JointState) -> None:
        self._gripper_stamp = time.monotonic()
        names = list(self.get_parameter("right_gripper_feedback_names").value)
        values = {str(name): float(value) for name, value in zip(message.name, message.position)}
        if tuple(str(name) for name in names) == RIGHT_GRIPPER_ROOT_JOINT_NAMES and all(name in values for name in names):
            # Convert the four signed root-joint observations back to aperture.
            self._gripper_value = sum(values[name] * sign for name, sign in zip(names, RIGHT_GRIPPER_ROOT_SIGNS)) / (len(names) * RIGHT_GRIPPER_SAFE_TRAVEL_RAD)

    def _on_pen_pose(self, message: String) -> None:
        try:
            value = json.loads(message.data); z = float(value["z_m"])
            if self._before_z is None: self._before_z = z
            self._after_z = z
        except (ValueError, TypeError, KeyError, json.JSONDecodeError): pass

    def _on_plan(self, message: String) -> None:
        if self._done or self._plan is not None: return
        try: plan = json.loads(message.data)
        except json.JSONDecodeError: self._emit("rejected", "plan_json_invalid"); return
        if plan.get("schema") != SCHEMA or plan.get("state") != "ready" or not plan.get("simulation_only") or not plan.get("scene_sha256"):
            self._emit("rejected", "plan_simulation_binding_invalid"); return
        self._plan = plan
        flags = all(bool(self.get_parameter(name).value) for name in ("enable_execution", "simulation_only", "motion_enabled"))
        if not flags:
            self._emit("dry_run_complete", "execution_gates_closed", target_id=plan.get("target_id"), stamp_ns=plan.get("stamp_ns"), scene_sha256=plan.get("scene_sha256"), phases=list(PHASES)); return
        names = list(self.get_parameter("right_joint_names").value)
        gripper_names = list(self.get_parameter("right_gripper_command_names").value)
        feedback_gripper_names = list(self.get_parameter("right_gripper_feedback_names").value)
        if (tuple(str(name) for name in names) != RIGHT_ARM_JOINT_NAMES
                or tuple(str(name) for name in gripper_names) != RIGHT_GRIPPER_ROOT_JOINT_NAMES
                or tuple(str(name) for name in feedback_gripper_names) != RIGHT_GRIPPER_ROOT_JOINT_NAMES):
            self._emit("rejected", "right_joint_names_must_be_configured"); return
        if time.monotonic() - self._joint_stamp > float(self.get_parameter("feedback_timeout_sec").value):
            self._emit("rejected", "joint_feedback_stale"); return
        self._index = 0; self._waiting = False; self._stage_started = 0.

    def _tick(self) -> None:
        if not self._plan or self._done or not bool(self.get_parameter("enable_execution").value) or not bool(self.get_parameter("simulation_only").value) or not bool(self.get_parameter("motion_enabled").value): return
        if self._index >= len(self._plan.get("steps", [])):
            self._done = True; self._emit("succeeded", "complete", target_id=self._plan["target_id"], stamp_ns=self._plan["stamp_ns"], scene_sha256=self._plan["scene_sha256"]); return
        step = self._plan["steps"][self._index]; phase = str(step.get("phase")); now = time.monotonic()
        if not self._waiting:
            if phase == "lift": self._before_z = self._after_z
            self._publish_step(step); self._waiting = True; self._stage_started = now
            self._emit("executing", "stage_command_published", phase=phase, stage_index=self._index, target_id=self._plan["target_id"], stamp_ns=self._plan["stamp_ns"], scene_sha256=self._plan["scene_sha256"]); return
        if now - self._stage_started > float(self.get_parameter("stage_timeout_sec").value):
            self._done = True; self._emit("failed", "stage_feedback_timeout", phase=phase, stage_index=self._index, target_id=self._plan["target_id"], stamp_ns=self._plan["stamp_ns"], scene_sha256=self._plan["scene_sha256"]); return
        age = now - (self._gripper_stamp if phase in {"close", "release"} else self._joint_stamp)
        if phase in {"close", "release"}:
            ok, reason = stage_feedback_converged(phase=phase, target_gripper=float(step["right_gripper"]), observed_gripper=self._gripper_value, feedback_age_sec=age, timeout_sec=float(self.get_parameter("feedback_timeout_sec").value), gripper_tolerance=float(self.get_parameter("gripper_tolerance").value))
        else:
            names = list(self.get_parameter("right_joint_names").value); observed = [self._joint_values.get(str(name), float("nan")) for name in names]
            ok, reason = stage_feedback_converged(phase=phase, target_arm_rad=step.get("right_arm_rad"), observed_arm_rad=observed, feedback_age_sec=age, timeout_sec=float(self.get_parameter("feedback_timeout_sec").value), arm_tolerance_rad=float(self.get_parameter("joint_tolerance_rad").value))
        if not ok: return
        if phase == "lift":
            ok, reason = evaluate_pen_lift(before_z_m=self._before_z, after_z_m=self._after_z, threshold_m=float(self.get_parameter("pen_lift_threshold_m").value))
            if not ok: self._done = True; self._emit("failed", reason, phase=phase, target_id=self._plan["target_id"], stamp_ns=self._plan["stamp_ns"], scene_sha256=self._plan["scene_sha256"]); return
        self._waiting = False; self._index += 1

    def _publish_step(self, step: dict[str, Any]) -> None:
        phase = step["phase"]
        if phase in {"close", "release"}:
            ok, reason, payload = build_sparse_right_gripper_command(step["right_gripper"])
            if not ok or payload is None:
                self._done = True; self._emit("failed", reason); return
            msg = JointState(); msg.header.stamp = self.get_clock().now().to_msg(); msg.name = payload["name"]; msg.position = payload["position"]; self._gripper_pub.publish(msg)
        else:
            ok, reason, payload = build_sparse_right_arm_command(step["right_arm_rad"])
            if not ok or payload is None:
                self._done = True; self._emit("failed", reason); return
            msg = JointState(); msg.header.stamp = self.get_clock().now().to_msg(); msg.name = payload["name"]; msg.position = payload["position"]; self._joint_pub.publish(msg)


def main(args: Any = None) -> None:
    rclpy.init(args=args); node = IsaacRightArmStageExecutor()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
