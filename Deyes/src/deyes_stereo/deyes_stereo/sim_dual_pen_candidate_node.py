"""ROS 2 wrapper for the simulation-only dual-pen candidate contract."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .depth_coordinate_node import depth_msg_to_array
from .sim_dual_pen_candidate_contract import (
    ExpectedSimulation,
    SimCameraInfo,
    SimImageFrame,
    SimTransform,
    bind_generic_pen_features_to_simulation,
    build_simulation_dual_pen_candidates,
)
from .stamp_pairing import BoundedStampCache, ExactStampPairCache


def _stamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def _rotation_from_quaternion(x: float, y: float, z: float, w: float) -> np.ndarray:
    values = np.asarray((x, y, z, w), dtype=np.float64)
    norm = float(np.linalg.norm(values))
    if norm < 1e-9 or not np.all(np.isfinite(values)):
        raise ValueError("tf_quaternion_invalid")
    x, y, z, w = values / norm
    return np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


@dataclass(frozen=True)
class _Depth:
    frame: SimImageFrame


class SimDualPenCandidateNode(Node):
    """Join only exact simulation frames and annotate their limited trust scope."""

    def __init__(self) -> None:
        super().__init__("sim_dual_pen_candidate_node")
        defaults = {
            "pen_features_topic": "/left_camera/pen_features", "depth_topic": "/left_camera/depth",
            "camera_info_topic": "/left_camera/camera_info_rect", "plane_topic": "/left_camera/ground_plane",
            "output_topic": "/sim/grasp/dual_pen_candidates", "status_topic": "/sim/grasp/dual_pen_candidates_status",
            # Empty defaults intentionally do not bind generic RGB features to
            # a scene.  Operators must copy these values from the manifest of
            # the USD currently launched in Isaac.
            "expected_world_id": "", "expected_scene_sha256": "", "expected_seed": -1,
            "expected_camera_frame": "Left_Camera", "pickup_table_id": "table_1",
            "initial_scene_phase": "", "assign_visible_pens_to_pickup_table": False,
            "min_depth_m": .20, "max_depth_m": 2.50, "min_plane_clearance_m": .004, "edge_margin_px": 12,
            "sync_cache_capacity": 8, "sync_cache_max_age_sec": .50, "publish_period_sec": .08,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        capacity = int(self.get_parameter("sync_cache_capacity").value)
        age_ns = int(float(self.get_parameter("sync_cache_max_age_sec").value) * 1e9)
        self._depth_info = ExactStampPairCache(capacity, age_ns)
        self._pair_waiting_plane = BoundedStampCache(capacity, age_ns)
        self._planes = BoundedStampCache(capacity, age_ns)
        self._ready = BoundedStampCache(capacity, age_ns)
        self._features = BoundedStampCache(capacity, age_ns)
        self._waiting_ready = BoundedStampCache(capacity, age_ns)
        self._expected = ExpectedSimulation(
            world_id=str(self.get_parameter("expected_world_id").value),
            scene_sha256=str(self.get_parameter("expected_scene_sha256").value),
            seed=int(self.get_parameter("expected_seed").value),
            camera_frame=str(self.get_parameter("expected_camera_frame").value),
            pickup_table_id=str(self.get_parameter("pickup_table_id").value),
            initial_scene_phase=str(self.get_parameter("initial_scene_phase").value),
        )
        self._output = self.create_publisher(String, str(self.get_parameter("output_topic").value), qos_profile_sensor_data)
        self._status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), qos_profile_sensor_data)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self.create_subscription(String, str(self.get_parameter("pen_features_topic").value), self._on_features, qos_profile_sensor_data)
        self.create_subscription(Image, str(self.get_parameter("depth_topic").value), self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, str(self.get_parameter("camera_info_topic").value), self._on_camera_info, qos_profile_sensor_data)
        self.create_subscription(String, str(self.get_parameter("plane_topic").value), self._on_plane, qos_profile_sensor_data)
        self.create_timer(float(self.get_parameter("publish_period_sec").value), self._on_timer)
        self._status("ok", "simulation_only_bridge_started", physical_execution_eligible=False)

    def _status(self, level: str, reason: str, **extra: Any) -> None:
        message = String()
        message.data = json.dumps({"level": level, "reason": reason, "source": "isaac_sim", "simulation_only": True, "physical_execution_eligible": False, **extra}, ensure_ascii=False, separators=(",", ":"))
        self._status_pub.publish(message)

    def _on_features(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            stamp = int(payload.get("stamp_sec", 0) or 0) * 1_000_000_000 + int(payload.get("stamp_nanosec", 0) or 0)
            if stamp <= 0:
                raise ValueError("pen_features_stamp_missing")
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self._status("warn", "invalid_pen_features:" + str(exc))
            return
        payload, binding_reason = bind_generic_pen_features_to_simulation(
            payload, self._expected,
            assign_visible_pens_to_pickup_table=bool(self.get_parameter("assign_visible_pens_to_pickup_table").value),
        )
        if binding_reason is not None:
            self._status("warn", binding_reason)
            return
        now = time.monotonic_ns()
        ready = self._waiting_ready.pop(stamp, now)
        if ready is None:
            self._features.put(stamp, payload, now)
        else:
            self._publish_from_join(stamp, *ready, payload)

    def _on_depth(self, msg: Image) -> None:
        try:
            frame = SimImageFrame(_stamp_ns(msg), str(msg.header.frame_id), int(msg.width), int(msg.height), str(msg.encoding), depth_msg_to_array(msg))
        except RuntimeError as exc:
            self._status("warn", str(exc))
            return
        pair = self._depth_info.add_left(frame.stamp_ns, _Depth(frame), time.monotonic_ns())
        if pair is not None:
            self._accept_depth_info(pair[0], pair[1].frame, pair[2])

    def _on_camera_info(self, msg: CameraInfo) -> None:
        pair = self._depth_info.add_right(_stamp_ns(msg), msg, time.monotonic_ns())
        if pair is not None:
            self._accept_depth_info(pair[0], pair[1].frame, pair[2])

    def _accept_depth_info(self, stamp: int, depth: SimImageFrame, info: CameraInfo) -> None:
        camera = SimCameraInfo(_stamp_ns(info), str(info.header.frame_id), int(info.width), int(info.height), tuple(float(value) for value in info.p))
        now = time.monotonic_ns()
        plane = self._planes.pop(stamp, now)
        if plane is None:
            self._pair_waiting_plane.put(stamp, (depth, camera), now)
        else:
            self._ready.put(stamp, (depth, camera, plane), now)

    def _on_plane(self, msg: String) -> None:
        try:
            plane = json.loads(msg.data)
            stamp = int(plane.get("stamp_sec", 0) or 0) * 1_000_000_000 + int(plane.get("stamp_nanosec", 0) or 0)
            if stamp <= 0:
                raise ValueError("plane_stamp_missing")
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self._status("warn", "invalid_plane:" + str(exc))
            return
        now = time.monotonic_ns()
        pair = self._pair_waiting_plane.pop(stamp, now)
        if pair is None:
            self._planes.put(stamp, plane, now)
        else:
            self._ready.put(stamp, (pair[0], pair[1], plane), now)

    def _lookup_transform(self, depth: SimImageFrame) -> SimTransform | None:
        try:
            # Query at exactly the depth stamp.  A static TF response may have
            # stamp=0; the requested stamp is retained in SimTransform.
            transform = self._tf_buffer.lookup_transform("base_link", self._expected.camera_frame, Time(nanoseconds=depth.stamp_ns), timeout=Duration(seconds=0.05))
            value = transform.transform
            return SimTransform(
                stamp_ns=depth.stamp_ns, parent_frame="base_link", child_frame=self._expected.camera_frame,
                rotation=_rotation_from_quaternion(value.rotation.x, value.rotation.y, value.rotation.z, value.rotation.w),
                translation=np.asarray((value.translation.x, value.translation.y, value.translation.z), dtype=np.float64),
            )
        except (TransformException, ValueError) as exc:
            self._status("warn", "base_link_T_Left_Camera_missing:" + str(exc))
            return None

    def _on_timer(self) -> None:
        now = time.monotonic_ns()
        self._depth_info.expire(now)
        self._pair_waiting_plane.expire(now)
        self._planes.expire(now)
        self._features.expire(now)
        self._waiting_ready.expire(now)
        ready = self._ready.pop_oldest(now)
        if ready is None:
            return
        stamp, (depth, camera, plane) = ready
        features = self._features.pop(stamp, now)
        if features is None:
            self._waiting_ready.put(stamp, (depth, camera, plane), now)
            self._status("waiting", "waiting_for_exact_stamp_pen_features")
            return
        self._publish_from_join(stamp, depth, camera, plane, features)

    def _publish_from_join(self, stamp: int, depth: SimImageFrame, camera: SimCameraInfo, plane: dict[str, Any], features: dict[str, Any]) -> None:
        result = build_simulation_dual_pen_candidates(
            features, depth, camera, plane, self._lookup_transform(depth), self._expected,
            min_depth_m=float(self.get_parameter("min_depth_m").value), max_depth_m=float(self.get_parameter("max_depth_m").value),
            min_plane_clearance_m=float(self.get_parameter("min_plane_clearance_m").value), edge_margin_px=int(self.get_parameter("edge_margin_px").value),
        )
        output = String()
        output.data = json.dumps(result, ensure_ascii=False, separators=(",", ":"))
        self._output.publish(output)
        self._status("ok" if result["valid"] else "warn", str(result["reason"]), candidate_count=int(result["candidate_count"]), stamp_ns=stamp)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = SimDualPenCandidateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
