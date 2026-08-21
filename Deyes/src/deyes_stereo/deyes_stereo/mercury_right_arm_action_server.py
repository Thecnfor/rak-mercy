"""Guarded ROS 2 Action server and sole owner of ``/dev/right_arm``."""

from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
import numpy as np
import yaml
from deyes_interfaces.action import ExecuteCartesianStage, ExecuteGripper
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .extrinsics_contract import load_yaml_document, validate_extrinsics
from .mercury_single_joint_executor import acquire_serial_port_lock, find_serial_port_owners, release_serial_port_lock
from .right_arm_execution_contract import RightArmExecutionProfile, profile_from_mapping, validate_cartesian_goal, validate_profile


def _failure(result: Any, code: str) -> Any:
    result.success=False; result.failure_code=code; return result


class MercuryRightArmActionServer(Node):
    def __init__(self)->None:
        super().__init__("mercury_right_arm_action_server")
        defaults={"dry_run":True,"enable_live_execution":False,"operator_confirmed":False,"autonomous_once":True,"site_profile_path":"","stereo_calibration_path":"","extrinsics_path":"","joint_state_topic":"/x1/right_arm/joint_states","status_topic":"/x1/right_arm/execution_status","cartesian_action":"/x1/right_arm/execute_cartesian_stage","gripper_action":"/x1/right_arm/execute_gripper"}
        for name,value in defaults.items():self.declare_parameter(name,value)
        self._status=self.create_publisher(String,str(self.get_parameter("status_topic").value),10)
        self._joint_pub=self.create_publisher(JointState,str(self.get_parameter("joint_state_topic").value),10)
        self._profile=RightArmExecutionProfile();self._calibration_id="";self._gate_reason="not_initialized"
        self._robot:Any=None;self._lock_fd:int|None=None;self._io_lock=threading.RLock();self._active=False
        self._configure()
        self._cartesian=ActionServer(self,ExecuteCartesianStage,str(self.get_parameter("cartesian_action").value),execute_callback=self._execute_cartesian,goal_callback=self._goal_callback,cancel_callback=lambda _:CancelResponse.ACCEPT)
        self._gripper=ActionServer(self,ExecuteGripper,str(self.get_parameter("gripper_action").value),execute_callback=self._execute_gripper,goal_callback=self._goal_callback,cancel_callback=lambda _:CancelResponse.ACCEPT)
        self.create_timer(.10,self._publish_feedback)
        self._publish("ready" if self._robot is not None else "inhibited",self._gate_reason)

    def _publish(self,state:str,reason:str,**extra:Any)->None:
        message=String();message.data=json.dumps({"state":state,"reason":reason,"selected_arm":"right","calibration_id":self._calibration_id,"hardware_connected":self._robot is not None,**extra},separators=(",",":"));self._status.publish(message)

    def _configure(self)->None:
        if bool(self.get_parameter("dry_run").value):self._gate_reason="dry_run_enabled";return
        if not bool(self.get_parameter("enable_live_execution").value):self._gate_reason="enable_live_execution_false";return
        if not bool(self.get_parameter("operator_confirmed").value):self._gate_reason="operator_confirmation_missing";return
        if not bool(self.get_parameter("autonomous_once").value):self._gate_reason="autonomous_once_required";return
        try:
            path=Path(str(self.get_parameter("site_profile_path").value)).expanduser()
            with path.open("r",encoding="utf-8") as handle:self._profile=profile_from_mapping(yaml.safe_load(handle) or {})
            valid,reason=validate_profile(self._profile)
            if not valid:self._gate_reason=reason;return
            stereo=load_yaml_document(str(self.get_parameter("stereo_calibration_path").value))
            extrinsics=load_yaml_document(str(self.get_parameter("extrinsics_path").value))
            checked=validate_extrinsics(extrinsics,stereo_document=stereo)
            if not checked.valid:self._gate_reason="calibration_gate:"+",".join(checked.reasons);return
            self._calibration_id=checked.calibration_id
            owners=find_serial_port_owners(self._profile.serial_port)
            if owners:self._gate_reason="serial_port_owned_by_other_process:"+",".join(map(str,owners));return
            self._lock_fd=acquire_serial_port_lock(self._profile.serial_port)
            from pymycobot import Mercury
            self._robot=Mercury(self._profile.serial_port)
            ok,reason=self._health()
            if not ok:raise RuntimeError(reason)
            self._gate_reason="ok"
        except Exception as exc:
            self._gate_reason=f"live_initialization_failed:{type(exc).__name__}:{exc}"
            self._robot=None
            if self._lock_fd is not None:release_serial_port_lock(self._lock_fd);self._lock_fd=None

    def _health(self)->tuple[bool,str]:
        if self._robot is None:return False,self._gate_reason
        try:
            with self._io_lock:
                powered=self._robot.is_power_on()
                status=self._robot.get_robot_status()
                errors=self._robot.get_error_information()
                angles=self._robot.get_angles()
            if powered not in (True,1):return False,"arm_power_not_confirmed"
            if status is None:return False,"robot_status_missing"
            if errors not in (None,0,[],()):return False,"robot_error_present"
            if not isinstance(angles,(list,tuple)) or len(angles)!=6:return False,"joint_feedback_invalid"
            if any(float(value)<low or float(value)>high for value,low,high in zip(angles,self._profile.joint_min_deg,self._profile.joint_max_deg)):return False,"joint_feedback_outside_site_limits"
        except Exception as exc:return False,"robot_health_read_failed:"+type(exc).__name__
        return True,"ok"

    def _goal_callback(self,_request:Any)->GoalResponse:
        with self._io_lock:
            if self._active:return GoalResponse.REJECT
            self._active=True;return GoalResponse.ACCEPT

    def _publish_feedback(self)->None:
        if self._robot is None:return
        with self._io_lock:
            try:angles=[float(v) for v in self._robot.get_angles()]
            except Exception:return
        if len(angles)!=6:return
        message=JointState();message.header.stamp=self.get_clock().now().to_msg();message.name=[f"right_arm_joint_{i+1}" for i in range(6)];message.position=[math.radians(v) for v in angles];self._joint_pub.publish(message)

    def _read_pose(self)->list[float]:
        with self._io_lock:values=[float(v) for v in self._robot.get_base_coords()]
        if len(values)!=6:raise RuntimeError("cartesian_feedback_invalid")
        return [values[0]/1000.,values[1]/1000.,values[2]/1000.,*values[3:]]

    def _stop(self)->None:
        try:
            with self._io_lock:self._robot.stop()
        except Exception:pass

    def _execute_cartesian(self,goal_handle:Any)->ExecuteCartesianStage.Result:
        result=ExecuteCartesianStage.Result();request=goal_handle.request
        valid,reason,target=validate_cartesian_goal(request,self._profile,calibration_id=self._calibration_id)
        if self._robot is None or not valid:
            self._active=False
            goal_handle.abort();return _failure(result,self._gate_reason if self._robot is None else reason)
        started=time.monotonic();last_time=started
        try:
            ok,reason=self._health()
            if not ok:goal_handle.abort();return _failure(result,reason)
            current=self._read_pose();last_position=current[:3];best_error=float(np.linalg.norm(np.asarray(current[:3])-np.asarray(target[:3])))
            vendor=[target[0]*1000.,target[1]*1000.,target[2]*1000.,*target[3:]]
            with self._io_lock:self._robot.send_base_coords(vendor,self._profile.vendor_speed,_async=True)
            while True:
                if goal_handle.is_cancel_requested:self._stop();goal_handle.canceled();return _failure(result,"cancelled")
                now=time.monotonic()
                if now-started>float(request.timeout_sec):self._stop();goal_handle.abort();return _failure(result,"stage_timeout_stopped")
                current=self._read_pose();after=time.monotonic()
                gap=after-last_time
                if gap>self._profile.max_feedback_gap_sec:self._stop();goal_handle.abort();return _failure(result,"feedback_gap_exceeds_limit")
                dt=max(1e-6,after-last_time);speed=float(np.linalg.norm(np.asarray(current[:3])-np.asarray(last_position)))/dt
                if speed>self._profile.max_cartesian_speed_m_s*1.20:self._stop();goal_handle.abort();return _failure(result,"measured_cartesian_speed_exceeds_limit")
                error=float(np.linalg.norm(np.asarray(current[:3])-np.asarray(target[:3])))
                if after-started>.5 and error>best_error+self._profile.max_tracking_error_m:self._stop();goal_handle.abort();return _failure(result,"tracking_divergence_exceeds_limit")
                best_error=min(best_error,error)
                feedback=ExecuteCartesianStage.Feedback();feedback.stage=request.stage;feedback.elapsed_sec=after-started;feedback.tracking_error_m=error;feedback.current_pose_base=current;goal_handle.publish_feedback(feedback)
                if error<=self._profile.max_tracking_error_m:
                    with self._io_lock:joints=[float(v) for v in self._robot.get_angles()]
                    if len(joints)!=6:raise RuntimeError("joint_feedback_invalid")
                    result.success=True;result.failure_code="";result.final_pose_base=current;result.final_joint_deg=joints;goal_handle.succeed();return result
                last_time,last_position=after,current[:3];time.sleep(.10)
        except Exception as exc:
            self._stop();goal_handle.abort();return _failure(result,"execution_exception:"+type(exc).__name__)
        finally:self._active=False

    def _execute_gripper(self,goal_handle:Any)->ExecuteGripper.Result:
        result=ExecuteGripper.Result();request=goal_handle.request
        valid,reason=validate_profile(self._profile)
        if self._robot is None or not valid:self._active=False;goal_handle.abort();return _failure(result,self._gate_reason if self._robot is None else reason)
        if request.arm_side!="right" or request.calibration_id!=self._calibration_id or not request.transaction_id.startswith("pick-"):self._active=False;goal_handle.abort();return _failure(result,"gripper_goal_identity_invalid")
        if request.action not in {"open","close"} or not 0<=request.effort_percent<=20 or not 0<request.timeout_sec<=5:self._active=False;goal_handle.abort();return _failure(result,"gripper_goal_invalid")
        started=time.monotonic();target=self._profile.gripper_open_value if request.action=="open" else self._profile.gripper_closed_value
        try:
            if not callable(getattr(self._robot,"get_gripper_value",None)):goal_handle.abort();return _failure(result,"gripper_feedback_unavailable")
            with self._io_lock:self._robot.set_gripper_value(target,self._profile.gripper_speed)
            while time.monotonic()-started<=request.timeout_sec:
                if goal_handle.is_cancel_requested:self._stop();goal_handle.canceled();return _failure(result,"cancelled")
                with self._io_lock:current=float(self._robot.get_gripper_value())
                feedback=ExecuteGripper.Feedback();feedback.action=request.action;feedback.elapsed_sec=time.monotonic()-started;feedback.current_value=current;goal_handle.publish_feedback(feedback)
                if abs(current-target)<=5:result.success=True;result.failure_code="";result.final_value=current;goal_handle.succeed();return result
                time.sleep(.10)
            self._stop();goal_handle.abort();return _failure(result,"gripper_timeout_stopped")
        except Exception as exc:self._stop();goal_handle.abort();return _failure(result,"gripper_exception:"+type(exc).__name__)
        finally:self._active=False

    def close(self)->None:
        self._stop()
        if self._lock_fd is not None:release_serial_port_lock(self._lock_fd);self._lock_fd=None


def main(args:Any=None)->None:
    rclpy.init(args=args);node=MercuryRightArmActionServer();executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    try:executor.spin()
    finally:node.close();node.destroy_node();rclpy.shutdown()
