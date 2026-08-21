"""ROS 2 exact-join wrapper for camera-optical grasp candidates.

This node is perception only: it creates no TF, robot, gripper, or motion
clients and publishes no command topic.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .depth_coordinate_node import depth_msg_to_array
from .stamp_pairing import BoundedStampCache, ExactStampPairCache
from .vision_grasp_candidate_contract import build_camera_optical_pen_candidates, coordinate_chain_templates


def _stamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


@dataclass(frozen=True)
class DepthFrame:
    stamp_ns: int
    frame_id: str
    width: int
    height: int
    encoding: str
    depth_m: Any


class VisionGraspCandidateNode(Node):
    """Build camera-only candidates after four exact-stamp inputs arrive."""

    def __init__(self) -> None:
        super().__init__("vision_grasp_candidate_node")
        defaults = {
            "pen_features_topic": "/x1/detection/pen_features", "depth_topic": "/x1/stereo/depth",
            "camera_info_topic": "/x1/stereo/left/camera_info_rect", "plane_topic": "/x1/ground/plane",
            "output_topic": "/x1/grasp/candidates_camera",
            "coordinate_chain_templates_topic": "/x1/grasp/candidates_camera/coordinate_chain_templates",
            "status_topic": "/x1/grasp/candidates_camera/status", "source": "physical_topic",
            "min_depth_m": .20, "max_depth_m": 2.50, "min_plane_clearance_m": .004,
            "edge_margin_px": 12, "max_target_age_sec": .50,
            "sync_cache_capacity": 8, "sync_cache_max_age_sec": .50, "publish_period_sec": .08,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        capacity = int(self.get_parameter("sync_cache_capacity").value)
        cache_age_ns = int(float(self.get_parameter("sync_cache_max_age_sec").value) * 1e9)
        self._depth_camera = ExactStampPairCache(capacity, cache_age_ns)
        self._pairs_waiting_plane = BoundedStampCache(capacity, cache_age_ns)
        self._planes = BoundedStampCache(capacity, cache_age_ns)
        self._ready = BoundedStampCache(capacity, cache_age_ns)
        self._features = BoundedStampCache(capacity, cache_age_ns)
        self._ready_waiting_feature = BoundedStampCache(capacity, cache_age_ns)
        self._output = self.create_publisher(String, str(self.get_parameter("output_topic").value), qos_profile_sensor_data)
        self._templates = self.create_publisher(String, str(self.get_parameter("coordinate_chain_templates_topic").value), qos_profile_sensor_data)
        self._status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), qos_profile_sensor_data)
        self.create_subscription(String, str(self.get_parameter("pen_features_topic").value), self._on_features, qos_profile_sensor_data)
        self.create_subscription(Image, str(self.get_parameter("depth_topic").value), self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, str(self.get_parameter("camera_info_topic").value), self._on_camera, qos_profile_sensor_data)
        self.create_subscription(String, str(self.get_parameter("plane_topic").value), self._on_plane, qos_profile_sensor_data)
        self.create_timer(float(self.get_parameter("publish_period_sec").value), self._on_timer)
        self._status("ok", "camera_candidate_node_started", physical_execution_eligible=False)

    def _status(self, level: str, reason: str, **values: Any) -> None:
        message = String()
        message.data = json.dumps({"level": level, "reason": reason, "physical_execution_eligible": False, **values}, ensure_ascii=False, separators=(",", ":"))
        self._status_pub.publish(message)

    @staticmethod
    def _json_stamp(payload: dict[str, Any], label: str) -> int:
        stamp = int(payload.get("stamp_sec", 0) or 0) * 1_000_000_000 + int(payload.get("stamp_nanosec", 0) or 0)
        if stamp <= 0:
            raise ValueError(label + "_stamp_missing")
        return stamp

    def _on_features(self, message: String) -> None:
        try:
            feature = json.loads(message.data)
            if not isinstance(feature, dict):
                raise ValueError("pen_features_must_be_object")
            stamp = self._json_stamp(feature, "pen_features")
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self._status("warn", "invalid_pen_features:" + str(exc))
            return
        now = time.monotonic_ns()
        ready = self._ready_waiting_feature.pop(stamp, now)
        if ready is None:
            self._features.put(stamp, feature, now)
        else:
            self._publish(stamp, *ready, feature)

    def _on_depth(self, message: Image) -> None:
        try:
            frame = DepthFrame(_stamp_ns(message), str(message.header.frame_id), int(message.width), int(message.height), str(message.encoding), depth_msg_to_array(message))
            if frame.stamp_ns <= 0:
                raise ValueError("depth_stamp_missing")
        except (RuntimeError, ValueError) as exc:
            self._status("warn", "invalid_depth:" + str(exc))
            return
        pair = self._depth_camera.add_left(frame.stamp_ns, frame, time.monotonic_ns())
        if pair is not None:
            self._accept_depth_camera(*pair)

    def _on_camera(self, message: CameraInfo) -> None:
        stamp = _stamp_ns(message)
        if stamp <= 0:
            self._status("warn", "camera_info_stamp_missing")
            return
        pair = self._depth_camera.add_right(stamp, message, time.monotonic_ns())
        if pair is not None:
            self._accept_depth_camera(*pair)

    def _accept_depth_camera(self, stamp: int, depth: DepthFrame, camera: CameraInfo) -> None:
        now = time.monotonic_ns()
        plane = self._planes.pop(stamp, now)
        if plane is None:
            self._pairs_waiting_plane.put(stamp, (depth, camera), now)
        else:
            self._ready.put(stamp, (depth, camera, plane), now)

    def _on_plane(self, message: String) -> None:
        try:
            plane = json.loads(message.data)
            if not isinstance(plane, dict):
                raise ValueError("plane_must_be_object")
            stamp = self._json_stamp(plane, "plane")
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self._status("warn", "invalid_plane:" + str(exc))
            return
        now = time.monotonic_ns()
        pair = self._pairs_waiting_plane.pop(stamp, now)
        if pair is None:
            self._planes.put(stamp, plane, now)
        else:
            self._ready.put(stamp, (pair[0], pair[1], plane), now)

    def _on_timer(self) -> None:
        now = time.monotonic_ns()
        for cache in (self._depth_camera, self._pairs_waiting_plane, self._planes, self._ready, self._features, self._ready_waiting_feature):
            cache.expire(now)
        ready = self._ready.pop_oldest(now)
        if ready is None:
            return
        stamp, (depth, camera, plane) = ready
        feature = self._features.pop(stamp, now)
        if feature is None:
            self._ready_waiting_feature.put(stamp, (depth, camera, plane), now)
            self._status("waiting", "waiting_for_exact_stamp_pen_features", stamp_ns=stamp)
            return
        self._publish(stamp, depth, camera, plane, feature)

    def _publish(self, stamp: int, depth: DepthFrame, camera: CameraInfo, plane: dict[str, Any], feature: dict[str, Any]) -> None:
        result = build_camera_optical_pen_candidates(
            feature, depth.depth_m, depth_stamp_ns=stamp, depth_frame_id=depth.frame_id,
            depth_width=depth.width, depth_height=depth.height, depth_encoding=depth.encoding,
            camera_stamp_ns=_stamp_ns(camera), camera_frame_id=str(camera.header.frame_id),
            camera_width=int(camera.width), camera_height=int(camera.height), projection=tuple(float(value) for value in camera.p),
            plane_payload=plane, source=str(self.get_parameter("source").value), now_stamp_ns=self.get_clock().now().nanoseconds,
            max_candidate_age_ns=int(float(self.get_parameter("max_target_age_sec").value) * 1e9),
            min_depth_m=float(self.get_parameter("min_depth_m").value), max_depth_m=float(self.get_parameter("max_depth_m").value),
            min_plane_clearance_m=float(self.get_parameter("min_plane_clearance_m").value), edge_margin_px=int(self.get_parameter("edge_margin_px").value),
        )
        output = String(); output.data = json.dumps(result, ensure_ascii=False, separators=(",", ":")); self._output.publish(output)
        template = coordinate_chain_templates(result)
        request_output = String(); request_output.data = json.dumps(template, ensure_ascii=False, separators=(",", ":")); self._templates.publish(request_output)
        self._status("ok" if result["valid"] else "warn", str(result["reason"]), candidate_count=result["candidate_count"], stamp_ns=stamp)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = VisionGraspCandidateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
