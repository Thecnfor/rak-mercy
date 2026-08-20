"""Publish dry-run pen-pick plans; this node has no hardware command path."""

from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .pen_pick_dry_run_contract import PickDryRunStateMachine, PickPlanLimits


class PenPickDryRunNode(Node):
    def __init__(self) -> None:
        super().__init__("pen_pick_dry_run_node")
        defaults = {
            "candidate_topic": "/x1/grasp/pen_candidates", "plan_topic": "/x1/pick/dry_run_plan",
            "status_topic": "/x1/pick/dry_run_status", "enable_execution": False,
            "operator_approved": False, "site_profile_validated": False,
            "max_candidate_age_sec": .35, "min_detection_confidence": .50,
            "max_depth_mad_m": .012, "min_mask_depth_valid_ratio": .25,
            "pregrasp_clearance_m": .12, "approach_distance_m": .08,
            "lift_distance_m": .15, "retreat_distance_m": .18,
            # Use numeric triples (rather than empty ROS parameters) so a
            # site profile can override them with its verified bounds. Equal
            # bounds are rejected by the contract.
            "workspace_min_base_m": [0.0, 0.0, 0.0], "workspace_max_base_m": [0.0, 0.0, 0.0],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._plan_pub = self.create_publisher(String, str(self.get_parameter("plan_topic").value), qos_profile_sensor_data)
        self._status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), qos_profile_sensor_data)
        self._machine = PickDryRunStateMachine()
        self.create_subscription(String, str(self.get_parameter("candidate_topic").value), self._on_candidate, qos_profile_sensor_data)
        self.get_logger().warn("dry-run only: no Nav2, arm, base, or gripper client is created")

    def _limits(self) -> PickPlanLimits:
        def vector(name: str) -> tuple[float, float, float] | None:
            value = list(self.get_parameter(name).value)
            return tuple(float(v) for v in value) if len(value) == 3 else None
        return PickPlanLimits(
            max_candidate_age_sec=float(self.get_parameter("max_candidate_age_sec").value),
            min_detection_confidence=float(self.get_parameter("min_detection_confidence").value),
            max_depth_mad_m=float(self.get_parameter("max_depth_mad_m").value),
            min_mask_depth_valid_ratio=float(self.get_parameter("min_mask_depth_valid_ratio").value),
            pregrasp_clearance_m=float(self.get_parameter("pregrasp_clearance_m").value),
            approach_distance_m=float(self.get_parameter("approach_distance_m").value),
            lift_distance_m=float(self.get_parameter("lift_distance_m").value),
            retreat_distance_m=float(self.get_parameter("retreat_distance_m").value),
            workspace_min_base_m=vector("workspace_min_base_m"), workspace_max_base_m=vector("workspace_max_base_m"),
        )

    def _on_candidate(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            result = self._machine.transition(
                payload, now_stamp_ns=time.time_ns(), limits=self._limits(),
                site_profile_validated=bool(self.get_parameter("site_profile_validated").value),
                enable_execution=bool(self.get_parameter("enable_execution").value),
                operator_approved=bool(self.get_parameter("operator_approved").value),
            )
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            result = {"state": "rejected", "reason": f"invalid_candidate_json:{exc}", "commands_emitted": False}
        output = String()
        output.data = json.dumps(result, ensure_ascii=False, separators=(",", ":"))
        self._plan_pub.publish(output)
        status = String()
        status.data = json.dumps({"level": "ok" if result["state"] == "dry_run_ready" else "warn", **result}, ensure_ascii=False, separators=(",", ":"))
        self._status_pub.publish(status)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = PenPickDryRunNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
