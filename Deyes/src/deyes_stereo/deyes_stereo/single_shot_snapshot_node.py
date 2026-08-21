"""Freeze one exact-stamp stereo bundle after a conservative stability gate."""

from __future__ import annotations

import json
import math
import time
from collections import OrderedDict
from pathlib import Path
from typing import Any

import rclpy
import cv2
import numpy as np
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .single_shot_snapshot_contract import NavGate, SnapshotLimits, StabilitySample, StabilityTracker, diagnostic_values, new_transaction_id, validate_nav_gate


def _stamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


class SingleShotSnapshotNode(Node):
    def __init__(self) -> None:
        super().__init__("single_shot_snapshot_node")
        defaults = {
            "image_topic": "/x1/stereo/debug/left_rect", "depth_topic": "/x1/stereo/depth",
            "camera_info_topic": "/x1/stereo/left/camera_info_rect", "plane_topic": "/x1/ground/plane",
            "pair_diagnostics_topic": "/x1/stereo/pair_diagnostics", "odom_topic": "/odom",
            "right_arm_joint_state_topic": "/x1/right_arm/joint_states",
            "snapshot_image_topic": "/x1/snapshot/left_rect", "snapshot_depth_topic": "/x1/snapshot/depth",
            "snapshot_camera_info_topic": "/x1/snapshot/camera_info_rect", "snapshot_plane_topic": "/x1/snapshot/plane",
            "status_topic": "/x1/pick/transaction_status", "reset_service": "/x1/pick/reset",
            "nav_gate_topic": "/x1/pick/nav_gate", "require_nav_gate": False,
            "autonomous_once": True, "dry_run": True, "enable_live_execution": False,
            "required_samples": 5, "stable_hold_sec": .5, "max_pair_skew_ms": 10.0,
            "max_plane_center_delta_m": .005, "max_plane_normal_delta_deg": 2.0,
            "max_base_linear_m_s": .01, "max_base_angular_rad_s": .02,
            "max_joint_delta_deg": .3, "state_timeout_sec": .5,
            "cache_capacity": 12, "cache_max_age_sec": 1.0, "log_root": "",
        }
        for name, value in defaults.items(): self.declare_parameter(name, value)
        limits = SnapshotLimits(**{name: self.get_parameter(name).value for name in SnapshotLimits.__dataclass_fields__})
        self._live_mode = not bool(self.get_parameter("dry_run").value) and bool(self.get_parameter("enable_live_execution").value)
        # A real arm is never implicitly authorized without navigation arrival
        # evidence. Debug/replay users can leave the parameter at false.
        self._require_nav_gate = self._live_mode or bool(self.get_parameter("require_nav_gate").value)
        self._tracker = StabilityTracker(limits, live_mode=self._live_mode)
        self._armed = bool(self.get_parameter("autonomous_once").value)
        self._capacity = int(self.get_parameter("cache_capacity").value)
        self._max_age_ns = int(float(self.get_parameter("cache_max_age_sec").value) * 1e9)
        self._bundles: OrderedDict[int, dict[str, Any]] = OrderedDict()
        self._pair_diagnostics: tuple[float,int,float] | None = None
        self._odom: tuple[float, float, float] | None = None
        self._joints: tuple[tuple[float, ...], float] | None = None
        self._transaction_id = ""
        self._nav_gate: tuple[NavGate, float] | None = None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._image_pub = self.create_publisher(Image, str(self.get_parameter("snapshot_image_topic").value), latched)
        self._depth_pub = self.create_publisher(Image, str(self.get_parameter("snapshot_depth_topic").value), latched)
        self._camera_pub = self.create_publisher(CameraInfo, str(self.get_parameter("snapshot_camera_info_topic").value), latched)
        self._plane_pub = self.create_publisher(String, str(self.get_parameter("snapshot_plane_topic").value), latched)
        self._status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), latched)
        self.create_subscription(Image, str(self.get_parameter("image_topic").value), lambda m: self._put("image", _stamp_ns(m), m), qos_profile_sensor_data)
        self.create_subscription(Image, str(self.get_parameter("depth_topic").value), lambda m: self._put("depth", _stamp_ns(m), m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, str(self.get_parameter("camera_info_topic").value), lambda m: self._put("camera", _stamp_ns(m), m), qos_profile_sensor_data)
        self.create_subscription(String, str(self.get_parameter("plane_topic").value), self._on_plane, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, str(self.get_parameter("pair_diagnostics_topic").value), self._on_diagnostics, 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, qos_profile_sensor_data)
        self.create_subscription(JointState, str(self.get_parameter("right_arm_joint_state_topic").value), self._on_joints, qos_profile_sensor_data)
        nav_gate_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, str(self.get_parameter("nav_gate_topic").value), self._on_nav_gate, nav_gate_qos)
        self.create_service(Trigger, str(self.get_parameter("reset_service").value), self._on_reset)
        self.create_timer(.05, self._evaluate)
        self._publish("waiting", "waiting_for_stability", commands_emitted=False)

    def _publish(self, state: str, reason: str, **extra: Any) -> None:
        gate = self._nav_gate[0] if self._nav_gate is not None else None
        message = String(); message.data = json.dumps({"schema":"single_shot_pick_transaction/v1","state":state,"reason":reason,"transaction_id":self._transaction_id,"live_mode":self._live_mode,"require_nav_gate":self._require_nav_gate,"mission_id":"" if gate is None else gate.mission_id,"nav_epoch":0 if gate is None else gate.nav_epoch,**extra}, separators=(",",":")); self._status_pub.publish(message)

    def _put(self, kind: str, stamp: int, message: Any) -> None:
        if stamp <= 0 or self._tracker.locked: return
        self._bundles.setdefault(stamp, {})[kind] = message
        self._bundles[stamp]["_receipt_ns"] = time.monotonic_ns()
        self._bundles.move_to_end(stamp)
        while len(self._bundles) > self._capacity: self._bundles.popitem(last=False)

    def _on_plane(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            stamp = int(payload.get("stamp_sec", 0))*1_000_000_000 + int(payload.get("stamp_nanosec", 0))
            if stamp <= 0: raise ValueError("plane_stamp_missing")
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self._publish("waiting", f"invalid_plane:{exc}"); return
        self._put("plane", stamp, (message, payload))

    def _on_diagnostics(self, message: DiagnosticArray) -> None:
        values = diagnostic_values(message)
        try:self._pair_diagnostics=(float(values["current_skew_ms"]),int(values.get("drop_skew","0"))+int(values.get("drop_stale","0")),time.monotonic())
        except (KeyError, TypeError, ValueError):self._pair_diagnostics=None

    def _on_odom(self, message: Odometry) -> None:
        linear = math.sqrt(message.twist.twist.linear.x**2 + message.twist.twist.linear.y**2 + message.twist.twist.linear.z**2)
        angular = math.sqrt(message.twist.twist.angular.x**2 + message.twist.twist.angular.y**2 + message.twist.twist.angular.z**2)
        self._odom = (linear, angular, time.monotonic())

    def _on_joints(self, message: JointState) -> None:
        if len(message.position) != 6: return
        self._joints = (tuple(math.degrees(float(value)) for value in message.position), time.monotonic())

    def _on_nav_gate(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            self._nav_gate = None
            self._publish("waiting", "nav_gate_json_invalid")
            return
        gate, reason = validate_nav_gate(payload, receipt_age_sec=0.0, limits=self._tracker.limits)
        self._nav_gate = None if gate is None else (gate, time.monotonic())
        if gate is None:
            self._publish("waiting", reason)

    def _fresh_nav_gate(self, now_s: float) -> tuple[NavGate | None, str]:
        if self._nav_gate is None:
            return None, "nav_gate_missing"
        gate, receipt_s = self._nav_gate
        payload = {
            "schema": "pick_nav_gate/v1", "state": "PICK_ARMED", "pick_authorized": True,
            "mission_id": gate.mission_id, "nav_epoch": gate.nav_epoch,
            "arrival_evidence": {"stamp_ns": gate.arrival_stamp_ns, "odom_stationary_sec": gate.odom_stationary_sec,
                                 "linear_speed_m_s": gate.linear_speed_m_s, "angular_speed_rad_s": gate.angular_speed_rad_s},
        }
        return validate_nav_gate(payload, receipt_age_sec=now_s - receipt_s, limits=self._tracker.limits)

    def _evaluate(self) -> None:
        if not self._armed or self._tracker.locked: return
        now_ns, now_s = time.monotonic_ns(), time.monotonic()
        gate: NavGate | None = None
        if self._require_nav_gate:
            gate, gate_reason = self._fresh_nav_gate(now_s)
            if gate is None:
                # Never allow samples accumulated before navigation arrival to
                # satisfy the post-arrival stability window.
                if self._tracker.samples:
                    self._tracker.reset()
                self._publish("waiting", gate_reason, stable_samples=0)
                return
        for stamp in list(self._bundles):
            if now_ns-int(self._bundles[stamp].get("_receipt_ns",now_ns)) > self._max_age_ns and len(self._bundles[stamp]) < 5:self._bundles.pop(stamp,None)
        complete = next(((stamp, value) for stamp, value in reversed(self._bundles.items()) if {"image","depth","camera","plane"} <= value.keys()), None)
        if complete is None or self._pair_diagnostics is None:return
        stamp, bundle = complete
        plane = bundle["plane"][1]
        odom = self._odom; joints = self._joints
        try:
            sample = StabilitySample(stamp,now_s,self._pair_diagnostics[0],tuple(float(v) for v in plane["plane_center_camera_m"]),tuple(float(v) for v in plane["plane_normal"]),bool(plane.get("valid_for_table_removal")),None if odom is None else odom[0],None if odom is None else odom[1],None if joints is None else joints[0],None if odom is None else now_s-odom[2],None if joints is None else now_s-joints[1],now_s-self._pair_diagnostics[2],self._pair_diagnostics[1])
        except (KeyError, TypeError, ValueError): return
        ready, reason = self._tracker.update(sample)
        self._bundles.pop(stamp, None)
        if not ready:
            self._publish("waiting", reason, stable_samples=len(self._tracker.samples)); return
        self._transaction_id = new_transaction_id(stamp)
        self._image_pub.publish(bundle["image"]); self._depth_pub.publish(bundle["depth"]); self._camera_pub.publish(bundle["camera"])
        snapshot_plane = dict(plane)
        if gate is not None:
            snapshot_plane.update({"transaction_id": self._transaction_id, "mission_id": gate.mission_id, "nav_epoch": gate.nav_epoch})
        plane_message = String(); plane_message.data = json.dumps(snapshot_plane, separators=(",", ":")); self._plane_pub.publish(plane_message)
        self._write_snapshot(stamp,bundle,snapshot_plane)
        self._publish("snapshot_frozen", "ok", stamp_ns=stamp, commands_emitted=False)

    def _write_snapshot(self,stamp:int,bundle:dict[str,Any],plane:dict[str,Any])->None:
        root = str(self.get_parameter("log_root").value).strip()
        if not root: return
        try:
            directory = Path(root).expanduser() / self._transaction_id; directory.mkdir(parents=True, exist_ok=False)
            image=bundle["image"];raw=np.frombuffer(image.data,dtype=np.uint8)
            if image.encoding in {"bgr8","rgb8"}:
                pixels=raw.reshape(image.height,image.step//3,3)[:,:image.width]
                if image.encoding=="rgb8":pixels=cv2.cvtColor(pixels,cv2.COLOR_RGB2BGR)
            elif image.encoding=="mono8":pixels=raw.reshape(image.height,image.step)[:,:image.width]
            else:raise ValueError("snapshot_image_encoding_unsupported")
            if not cv2.imwrite(str(directory/"left_rect.png"),pixels):raise OSError("snapshot_image_write_failed")
            depth=bundle["depth"]
            if depth.encoding!="32FC1":raise ValueError("snapshot_depth_must_be_32FC1")
            depth_array=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.step//4)[:,:depth.width].copy()
            np.save(directory/"depth_m.npy",depth_array,allow_pickle=False)
            camera=bundle["camera"]
            camera_payload={"frame_id":camera.header.frame_id,"width":camera.width,"height":camera.height,"d":list(camera.d),"k":list(camera.k),"r":list(camera.r),"p":list(camera.p)}
            (directory/"camera_info_rect.json").write_text(json.dumps(camera_payload,indent=2),encoding="utf-8")
            (directory/"snapshot_manifest.json").write_text(json.dumps({"transaction_id":self._transaction_id,"stamp_ns":stamp,"plane":plane,"files":["left_rect.png","depth_m.npy","camera_info_rect.json"]},indent=2),encoding="utf-8")
        except (OSError,ValueError,cv2.error) as exc:self._publish("snapshot_frozen",f"log_write_failed:{type(exc).__name__}",stamp_ns=stamp)

    def _on_reset(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._bundles.clear(); self._tracker.reset(); self._transaction_id=""; self._nav_gate=None; self._armed=True
        response.success=True; response.message="single_shot_transaction_reset"; self._publish("waiting","manual_reset")
        return response


def main(args: Any = None) -> None:
    rclpy.init(args=args); node=SingleShotSnapshotNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
