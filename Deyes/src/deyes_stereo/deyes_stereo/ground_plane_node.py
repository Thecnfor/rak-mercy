"""Camera-relative dynamic table-plane evidence for depth masking only.

This node is deliberately not a grasp/world-frame provider. A table plane can
move as RANSAC inliers move; fixed ``base_link`` coordinates require separately
validated physical extrinsics.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .ground_plane_contract import (
    PlaneFit,
    evaluate_plane,
    fit_plane_ransac,
    normal_delta_deg,
    project_rectified_depth_pixels,
    validate_rectified_depth_pair,
)
from .stamp_pairing import BoundedStampCache, ExactStampPairCache


@dataclass
class DepthFrame:
    stamp_ns: int
    frame_id: str
    width: int
    height: int
    encoding: str
    depth: np.ndarray


def _stamp_ns(msg: Any) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def depth_msg_to_array(msg: Image) -> np.ndarray:
    if msg.encoding != "32FC1":
        raise RuntimeError(f"depth_encoding_must_be_32FC1:{msg.encoding}")
    row = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)
    return row[:, : msg.width].copy()


def compact_float(value: float) -> float:
    return round(float(value), 5)


def rotation_matrix_to_quaternion(matrix: np.ndarray) -> tuple[float, float, float, float]:
    r = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(r))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        x, y, z, w = (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, .25 * s
    elif r[0, 0] >= r[1, 1] and r[0, 0] >= r[2, 2]:
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        x, y, z, w = .25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s
    elif r[1, 1] >= r[2, 2]:
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        x, y, z, w = (r[0, 1] + r[1, 0]) / s, .25 * s, (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s
    else:
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        x, y, z, w = (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, .25 * s, (r[1, 0] - r[0, 1]) / s
    return (float(x), float(y), float(z), float(w))


class GroundPlaneNode(Node):
    def __init__(self) -> None:
        super().__init__("ground_plane_node")
        defaults = {
            "depth_topic": "/x1/stereo/depth", "camera_info_topic": "/x1/stereo/left/camera_info_rect",
            "plane_topic": "/x1/ground/plane", "status_topic": "/x1/ground/plane_status",
            "dynamic_plane_frame": "table_plane_dynamic_debug", "publish_period_sec": .30,
            "min_depth_m": .20, "max_depth_m": 1.50, "sample_step": 2, "max_points": 6000,
            "ransac_distance_threshold": .02, "ransac_iterations": 120, "min_inlier_ratio": .20,
            "max_plane_residual_rms_m": .010, "max_plane_residual_p95_m": .020,
            "max_normal_delta_deg": 15.0, "publish_debug_tf": False,
            "sync_cache_capacity": 8, "sync_cache_max_age_sec": .50, "pending_pair_capacity": 2,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        cache_age_ns = int(float(self.get_parameter("sync_cache_max_age_sec").value) * 1e9)
        cache_capacity = int(self.get_parameter("sync_cache_capacity").value)
        self._depth_info_pairs = ExactStampPairCache(cache_capacity, cache_age_ns)
        self._pending_pairs = BoundedStampCache(
            int(self.get_parameter("pending_pair_capacity").value), cache_age_ns
        )
        self._last_normal: Optional[np.ndarray] = None
        self._last_center: Optional[np.ndarray] = None
        self._depth_topic = str(self.get_parameter("depth_topic").value)
        self._camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self._plane_topic = str(self.get_parameter("plane_topic").value)
        self._status_topic = str(self.get_parameter("status_topic").value)
        self._dynamic_frame = str(self.get_parameter("dynamic_plane_frame").value)
        self._min_depth_m, self._max_depth_m = float(self.get_parameter("min_depth_m").value), float(self.get_parameter("max_depth_m").value)
        self._sample_step, self._max_points = max(1, int(self.get_parameter("sample_step").value)), max(32, int(self.get_parameter("max_points").value))
        self._ransac_threshold = float(self.get_parameter("ransac_distance_threshold").value)
        self._ransac_iterations = max(1, int(self.get_parameter("ransac_iterations").value))
        self._min_inlier_ratio = float(self.get_parameter("min_inlier_ratio").value)
        self._max_rms = float(self.get_parameter("max_plane_residual_rms_m").value)
        self._max_p95 = float(self.get_parameter("max_plane_residual_p95_m").value)
        self._max_normal_delta_deg = float(self.get_parameter("max_normal_delta_deg").value)
        self._publish_debug_tf = bool(self.get_parameter("publish_debug_tf").value)
        self._plane_pub = self.create_publisher(String, self._plane_topic, qos_profile_sensor_data)
        self._status_pub = self.create_publisher(String, self._status_topic, qos_profile_sensor_data)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_debug_tf else None
        self.create_subscription(Image, self._depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self._camera_info_topic, self._on_camera_info, qos_profile_sensor_data)
        self.create_timer(float(self.get_parameter("publish_period_sec").value), self._on_timer)
        self.get_logger().info(f"ground plane is camera-relative table evidence only: depth={self._depth_topic} info={self._camera_info_topic} publish_debug_tf={self._publish_debug_tf}")

    def _publish_status(self, level: str, message: str, **extra: Any) -> None:
        msg = String()
        msg.data = json.dumps({"level": level, "message": message, "coordinate_contract": "dynamic_table_plane_camera_relative_only", "trusted_for_grasp": False, "dynamic_tf_debug_only": self._publish_debug_tf, **extra}, ensure_ascii=False, separators=(",", ":"))
        self._status_pub.publish(msg)

    def _on_depth(self, msg: Image) -> None:
        try:
            frame = DepthFrame(_stamp_ns(msg), str(msg.header.frame_id), int(msg.width), int(msg.height), str(msg.encoding), depth_msg_to_array(msg))
            pair = self._depth_info_pairs.add_left(frame.stamp_ns, frame, time.monotonic_ns())
            if pair is not None:
                self._pending_pairs.put(pair[0], (pair[1], pair[2]), time.monotonic_ns())
        except (RuntimeError, ValueError) as exc:
            self._publish_status("invalid", str(exc))

    def _on_camera_info(self, msg: CameraInfo) -> None:
        pair = self._depth_info_pairs.add_right(_stamp_ns(msg), msg, time.monotonic_ns())
        if pair is not None:
            self._pending_pairs.put(pair[0], (pair[1], pair[2]), time.monotonic_ns())

    def _on_timer(self) -> None:
        now_ns = time.monotonic_ns()
        self._depth_info_pairs.expire(now_ns)
        pending = self._pending_pairs.pop_newest(now_ns)
        if pending is None:
            self._publish_status("waiting", "waiting_for_exact_rectified_depth_camera_info_pair")
            return
        _, (frame, info) = pending
        try:
            payload = self._build_payload(frame, info)
        except RuntimeError as exc:
            self._publish_status("invalid", str(exc))
            return
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self._plane_pub.publish(message)
        level = "degraded" if payload["degraded"] else "ok"
        self._publish_status(level, payload["reason"], inlier_ratio=payload["inlier_ratio"], residual_rms_m=payload["residual_rms_m"], residual_p95_m=payload["residual_p95_m"], normal_delta_deg=payload["normal_delta_deg"])

    def _build_payload(self, frame: DepthFrame, info: CameraInfo) -> dict[str, Any]:
        contract = validate_rectified_depth_pair(depth_stamp_ns=frame.stamp_ns, depth_frame_id=frame.frame_id, depth_width=frame.width, depth_height=frame.height, depth_encoding=frame.encoding, info_stamp_ns=_stamp_ns(info), info_frame_id=str(info.header.frame_id), info_width=int(info.width), info_height=int(info.height), projection=info.p)
        if not contract.valid:
            raise RuntimeError("rectified_depth_camera_info_contract:" + ",".join(contract.reasons))
        sampled = frame.depth[::self._sample_step, ::self._sample_step]
        valid = np.isfinite(sampled) & (sampled >= self._min_depth_m) & (sampled <= self._max_depth_m)
        if int(np.count_nonzero(valid)) < 3:
            raise RuntimeError("no_valid_depth_samples")
        rows, cols = np.nonzero(valid)
        z = sampled[valid].astype(np.float64)
        u, v = cols.astype(np.float64) * self._sample_step, rows.astype(np.float64) * self._sample_step
        points = project_rectified_depth_pixels(u, v, z, info.p)
        if len(points) > self._max_points:
            points = points[np.linspace(0, len(points) - 1, self._max_points, dtype=np.int32)]
        fresh = fit_plane_ransac(points, self._ransac_threshold, self._ransac_iterations, seed=frame.stamp_ns % (2**32))
        if fresh is None:
            raise RuntimeError("ransac_failed")
        if fresh.inlier_ratio < self._min_inlier_ratio or fresh.residual_rms_m > self._max_rms or fresh.residual_p95_m > self._max_p95:
            raise RuntimeError("plane_quality_rejected:ratio=%.4f,rms=%.5f,p95=%.5f" % (fresh.inlier_ratio, fresh.residual_rms_m, fresh.residual_p95_m))
        if self._last_normal is not None and float(fresh.normal @ self._last_normal) < 0.0:
            fresh = PlaneFit(-fresh.normal, fresh.center, fresh.inlier_mask, fresh.residuals_m)
        delta = normal_delta_deg(fresh.normal, self._last_normal)
        degraded = delta is not None and delta > self._max_normal_delta_deg
        selected, reason = fresh, "fresh_plane"
        if degraded:
            assert self._last_normal is not None and self._last_center is not None
            selected, reason = evaluate_plane(points, self._last_normal, self._last_center, self._ransac_threshold), "normal_discontinuity_fallback"
        else:
            self._last_normal, self._last_center = fresh.normal.copy(), fresh.center.copy()
        if self._publish_debug_tf and not degraded:
            self._broadcast_debug_tf(selected, frame)
        return {
            "stamp_sec": frame.stamp_ns // 1_000_000_000, "stamp_nanosec": frame.stamp_ns % 1_000_000_000,
            "camera_frame": frame.frame_id, "dynamic_plane_frame": self._dynamic_frame,
            "coordinate_contract": "dynamic_table_plane_camera_relative_only", "trusted_for_grasp": False,
            "valid_for_table_removal": not degraded, "degraded": degraded, "reason": reason, "dynamic_tf_debug_only": self._publish_debug_tf,
            "plane_normal": [compact_float(v) for v in selected.normal], "plane_center_camera_m": [compact_float(v) for v in selected.center],
            "plane_distance_camera_m": compact_float(float(selected.normal @ selected.center)),
            "inlier_count": selected.inlier_count, "inlier_ratio": compact_float(selected.inlier_ratio),
            "residual_rms_m": compact_float(selected.residual_rms_m), "residual_p95_m": compact_float(selected.residual_p95_m),
            "normal_delta_deg": None if delta is None else compact_float(delta),
        }

    def _broadcast_debug_tf(self, fit: PlaneFit, frame: DepthFrame) -> None:
        if self._tf_broadcaster is None:
            return
        z_axis = fit.normal / np.linalg.norm(fit.normal)
        x_axis = np.array([1.0, 0.0, 0.0])
        x_axis -= float(x_axis @ z_axis) * z_axis
        if np.linalg.norm(x_axis) < 1e-6:
            x_axis = np.array([0.0, 1.0, 0.0])
            x_axis -= float(x_axis @ z_axis) * z_axis
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        q = rotation_matrix_to_quaternion(np.stack([x_axis, y_axis, z_axis], axis=1))
        transform = TransformStamped()
        transform.header.stamp.sec, transform.header.stamp.nanosec = frame.stamp_ns // 1_000_000_000, frame.stamp_ns % 1_000_000_000
        transform.header.frame_id, transform.child_frame_id = frame.frame_id, self._dynamic_frame
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = [float(v) for v in fit.center]
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = q
        self._tf_broadcaster.sendTransform(transform)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = GroundPlaneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
