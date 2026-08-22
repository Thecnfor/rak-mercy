"""Create one identity-bound right-arm dry-run plan from trusted TF geometry."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .pen_pick_dry_run_contract import PickPlanLimits, build_plan_from_coordinate_result
from .right_arm_execution_contract import profile_from_mapping, validate_profile


class SingleShotPickPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("single_shot_pick_planner_node")
        defaults = {
            "coordinate_result_topic":"/x1/coordinate_chain/result", "plan_topic":"/x1/pick/dry_run_plan",
            "status_topic":"/x1/pick/planner_status", "site_profile_validated":False,
            "site_profile_path":"",
            "max_candidate_age_sec":2.0, "min_detection_confidence":.50,
            "max_depth_mad_m":.012, "min_mask_depth_valid_ratio":.25,
            "pregrasp_clearance_m":.12, "approach_distance_m":.08,
            "lift_distance_m":.10, "retreat_distance_m":.12,
            "workspace_min_base_m":[0.,0.,0.], "workspace_max_base_m":[0.,0.,0.],
        }
        for name,value in defaults.items(): self.declare_parameter(name,value)
        self._done=False
        self._plan=self.create_publisher(String,str(self.get_parameter("plan_topic").value),qos_profile_sensor_data)
        self._status=self.create_publisher(String,str(self.get_parameter("status_topic").value),qos_profile_sensor_data)
        self.create_subscription(String,str(self.get_parameter("coordinate_result_topic").value),self._on_coordinate,qos_profile_sensor_data)

    def _limits(self) -> PickPlanLimits:
        path=str(self.get_parameter("site_profile_path").value).strip()
        if path:
            with Path(path).expanduser().open("r",encoding="utf-8") as handle:profile=profile_from_mapping(yaml.safe_load(handle) or {})
            valid,reason=validate_profile(profile)
            if not valid:raise ValueError(reason)
            workspace_min,workspace_max=profile.workspace_min_base_m,profile.workspace_max_base_m
        else:
            workspace_min,workspace_max=tuple(float(v) for v in self.get_parameter("workspace_min_base_m").value),tuple(float(v) for v in self.get_parameter("workspace_max_base_m").value)
        return PickPlanLimits(
            max_candidate_age_sec=float(self.get_parameter("max_candidate_age_sec").value),
            min_detection_confidence=float(self.get_parameter("min_detection_confidence").value),
            max_depth_mad_m=float(self.get_parameter("max_depth_mad_m").value),
            min_mask_depth_valid_ratio=float(self.get_parameter("min_mask_depth_valid_ratio").value),
            pregrasp_clearance_m=float(self.get_parameter("pregrasp_clearance_m").value),
            approach_distance_m=float(self.get_parameter("approach_distance_m").value),
            lift_distance_m=float(self.get_parameter("lift_distance_m").value),
            retreat_distance_m=float(self.get_parameter("retreat_distance_m").value),
            workspace_min_base_m=workspace_min, workspace_max_base_m=workspace_max,
        )

    def _on_coordinate(self,message:String)->None:
        if self._done: return
        try:
            coordinate=json.loads(message.data)
            site_path=str(self.get_parameter("site_profile_path").value).strip()
            result=build_plan_from_coordinate_result(coordinate,now_stamp_ns=self.get_clock().now().nanoseconds,limits=self._limits(),site_profile_validated=bool(site_path) or bool(self.get_parameter("site_profile_validated").value))
        except (json.JSONDecodeError,TypeError,ValueError) as exc:
            result={"state":"rejected","reason":f"coordinate_json_invalid:{exc}","commands_emitted":False}
        self._done=True
        output=String();output.data=json.dumps(result,separators=(",",":"));self._plan.publish(output)
        status=String();status.data=json.dumps({"level":"ok" if result.get("state")=="dry_run_ready" else "invalid",**result},separators=(",",":"));self._status.publish(status)


def main(args:Any=None)->None:
    rclpy.init(args=args);node=SingleShotPickPlannerNode()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
