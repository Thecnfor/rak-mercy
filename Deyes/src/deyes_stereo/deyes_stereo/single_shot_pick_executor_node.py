"""Execute one identity-bound plan through guarded right-arm Action servers."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import rclpy
from deyes_interfaces.action import ExecuteCartesianStage, ExecuteGripper
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .right_arm_execution_contract import build_action_steps


class SingleShotPickExecutorNode(Node):
    def __init__(self)->None:
        super().__init__("single_shot_pick_executor_node")
        defaults={"plan_topic":"/x1/pick/dry_run_plan","coordinate_result_topic":"/x1/coordinate_chain/result","status_topic":"/x1/pick/execution_status","cartesian_action":"/x1/right_arm/execute_cartesian_stage","gripper_action":"/x1/right_arm/execute_gripper","arm_side":"right","autonomous_once":True,"dry_run":True,"enable_live_execution":False,"operator_confirmed":False,"max_cartesian_speed_m_s":.02,"cartesian_timeout_sec":8.,"gripper_timeout_sec":3.,"gripper_effort_percent":20.,"log_root":""}
        for name,value in defaults.items():self.declare_parameter(name,value)
        self._status=self.create_publisher(String,str(self.get_parameter("status_topic").value),qos_profile_sensor_data)
        self._cartesian=ActionClient(self,ExecuteCartesianStage,str(self.get_parameter("cartesian_action").value))
        self._gripper=ActionClient(self,ExecuteGripper,str(self.get_parameter("gripper_action").value))
        self._steps:list[dict[str,Any]]=[];self._index=0;self._locked=False;self._hold_until:float|None=None;self._coordinate:dict[str,Any]|None=None;self._pending_plan:String|None=None;self._pending_deadline:float|None=None;self._transaction_id=""
        self.create_subscription(String,str(self.get_parameter("coordinate_result_topic").value),self._on_coordinate,qos_profile_sensor_data)
        self.create_subscription(String,str(self.get_parameter("plan_topic").value),self._on_plan,qos_profile_sensor_data)
        self.create_timer(.05,self._timer)

    def _publish(self,state:str,reason:str,**extra:Any)->None:
        payload={"schema":"single_shot_pick_execution/v1","state":state,"reason":reason,"selected_arm":"right","transaction_id":self._transaction_id,"hardware_commands_emitted":bool(extra.pop("hardware_commands_emitted",False)),**extra}
        message=String();message.data=json.dumps(payload,separators=(",",":"));self._status.publish(message)
        root=str(self.get_parameter("log_root").value).strip()
        if root and self._transaction_id:
            try:
                directory=Path(root).expanduser()/self._transaction_id;directory.mkdir(parents=True,exist_ok=True)
                with (directory/"execution_trace.jsonl").open("a",encoding="utf-8") as handle:handle.write(json.dumps({"wall_time_ns":time.time_ns(),**payload},separators=(",",":"))+"\n")
            except OSError:pass

    def _on_plan(self,message:String)->None:
        if self._locked:return
        self._locked=True
        try:plan=json.loads(message.data);steps,reason=build_action_steps(plan)
        except json.JSONDecodeError as exc:self._publish("failed",f"plan_json_invalid:{exc}");return
        if reason!="ok":self._publish("failed",reason);return
        self._transaction_id=str(plan.get("transaction_id") or "")
        coordinate=self._coordinate
        if not isinstance(coordinate,dict) or coordinate.get("trusted_for_execution") is not True:
            self._locked=False;self._pending_plan=message;self._pending_deadline=time.monotonic()+.5;self._publish("waiting","waiting_for_trusted_coordinate_result");return
        if str(coordinate.get("transaction_id"))!=str(plan.get("transaction_id")) or str(coordinate.get("candidate_id"))!=str(plan.get("target_id")) or str(coordinate.get("calibration_id"))!=str(plan.get("calibration_id")) or int(coordinate.get("stamp_ns",-1))!=int(plan.get("candidate_stamp_ns",-2)):
            self._publish("failed","coordinate_plan_identity_mismatch");return
        if str(self.get_parameter("arm_side").value)!="right":self._publish("failed","arm_side_must_be_right");return
        live=not bool(self.get_parameter("dry_run").value) and bool(self.get_parameter("enable_live_execution").value) and bool(self.get_parameter("operator_confirmed").value) and bool(self.get_parameter("autonomous_once").value)
        if not live:
            self._publish("dry_run_complete","live_execution_gates_closed",transaction_id=plan.get("transaction_id"),step_count=len(steps),hardware_commands_emitted=False)
            return
        self._steps=steps;self._index=0;self._publish("executing","starting",transaction_id=plan.get("transaction_id"));self._dispatch()

    def _on_coordinate(self,message:String)->None:
        if self._locked:return
        try:value=json.loads(message.data);self._coordinate=value if isinstance(value,dict) else None
        except json.JSONDecodeError:self._coordinate=None
        if self._pending_plan is not None:
            pending=self._pending_plan;self._pending_plan=None;self._pending_deadline=None;self._on_plan(pending)

    def _timer(self)->None:
        if self._pending_deadline is not None and time.monotonic()>self._pending_deadline:
            self._pending_deadline=None;self._pending_plan=None;self._locked=True;self._publish("failed","trusted_coordinate_result_missing");return
        if self._hold_until is not None and time.monotonic()>=self._hold_until:
            self._hold_until=None;self._index+=1;self._dispatch()

    def _dispatch(self)->None:
        if self._index>=len(self._steps):self._publish("succeeded","ok",hardware_commands_emitted=True);return
        step=self._steps[self._index]
        if step["kind"]=="hold":
            self._publish("executing","holding_lift",step_index=self._index);self._hold_until=time.monotonic()+float(step["duration_sec"]);return
        if step["kind"]=="cartesian":
            if not self._cartesian.wait_for_server(timeout_sec=.25):self._publish("failed","cartesian_action_server_unavailable",failed_step=step["stage"]);return
            goal=ExecuteCartesianStage.Goal();goal.transaction_id=step["transaction_id"];goal.calibration_id=step["calibration_id"];goal.arm_side="right";goal.stage=step["stage"];goal.pose_base=step["pose_base"];goal.max_speed_m_s=float(self.get_parameter("max_cartesian_speed_m_s").value);goal.timeout_sec=float(self.get_parameter("cartesian_timeout_sec").value)
            future=self._cartesian.send_goal_async(goal);future.add_done_callback(lambda value:self._goal_response(value,step))
        else:
            if not self._gripper.wait_for_server(timeout_sec=.25):self._publish("failed","gripper_action_server_unavailable",failed_step=step["action"]);return
            goal=ExecuteGripper.Goal();goal.transaction_id=step["transaction_id"];goal.calibration_id=step["calibration_id"];goal.arm_side="right";goal.action=step["action"];goal.effort_percent=float(self.get_parameter("gripper_effort_percent").value);goal.timeout_sec=float(self.get_parameter("gripper_timeout_sec").value)
            future=self._gripper.send_goal_async(goal);future.add_done_callback(lambda value:self._goal_response(value,step))

    def _goal_response(self,future:Any,step:dict[str,Any])->None:
        try:handle=future.result()
        except Exception as exc:self._publish("failed","action_goal_exception:"+type(exc).__name__);return
        if not handle.accepted:self._publish("failed","action_goal_rejected",failed_step=step.get("stage",step.get("action")));return
        result=handle.get_result_async();result.add_done_callback(lambda value:self._result(value,step))

    def _result(self,future:Any,step:dict[str,Any])->None:
        try:result=future.result().result
        except Exception as exc:self._publish("failed","action_result_exception:"+type(exc).__name__);return
        if not result.success:self._publish("failed",str(result.failure_code or "action_failed"),failed_step=step.get("stage",step.get("action")),hardware_commands_emitted=True);return
        self._index+=1;self._publish("executing","stage_succeeded",completed_step=step.get("stage",step.get("action")),step_index=self._index,hardware_commands_emitted=True);self._dispatch()


def main(args:Any=None)->None:
    rclpy.init(args=args);node=SingleShotPickExecutorNode()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
