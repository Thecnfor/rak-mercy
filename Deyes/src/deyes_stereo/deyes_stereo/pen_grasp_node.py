"""Publish fail-closed pen candidates from exact-stamp depth/plane joins."""

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
from .extrinsics_contract import ExtrinsicsValidation, load_yaml_document, validate_extrinsics
from .ground_plane_contract import (
    rectified_intrinsics,
    validate_dynamic_plane_for_depth,
    validate_rectified_depth_pair,
)
from .pen_grasp_contract import build_pen_candidates, feature_matches_depth_stamp, pen_feature_stamp_ns
from .stamp_pairing import BoundedStampCache, ExactStampPairCache


def _stamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


@dataclass(frozen=True)
class DepthSample:
    stamp_ns: int
    frame_id: str
    width: int
    height: int
    encoding: str
    depth: Any


class PenGraspNode(Node):
    def __init__(self) -> None:
        super().__init__("pen_grasp_node")
        defaults = {
            "pen_features_topic": "/x1/detection/pen_features", "depth_topic": "/x1/stereo/depth",
            "camera_info_topic": "/x1/stereo/left/camera_info_rect", "plane_topic": "/x1/ground/plane",
            "output_topic": "/x1/grasp/pen_candidates", "status_topic": "/x1/grasp/pen_candidates_status",
            "extrinsics_path": "", "stereo_calibration_path": "",
            "min_depth_m": .20, "max_depth_m": 1.00, "min_plane_clearance_m": .004,
            "edge_margin_px": 12, "publish_period_sec": .08, "sync_cache_capacity": 8,
            "sync_cache_max_age_sec": .50,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        cache_age_ns = int(float(self.get_parameter("sync_cache_max_age_sec").value) * 1e9)
        cache_capacity = int(self.get_parameter("sync_cache_capacity").value)
        self._depth_info_pairs = ExactStampPairCache(cache_capacity, cache_age_ns)
        self._unmatched_pairs = BoundedStampCache(cache_capacity, cache_age_ns)
        self._unmatched_planes = BoundedStampCache(cache_capacity, cache_age_ns)
        self._ready = BoundedStampCache(cache_capacity, cache_age_ns)
        self._unmatched_ready = BoundedStampCache(cache_capacity, cache_age_ns)
        self._features = BoundedStampCache(cache_capacity, cache_age_ns)
        self._trust = self._load_trust()
        self._output = self.create_publisher(String, str(self.get_parameter("output_topic").value), qos_profile_sensor_data)
        self._status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), qos_profile_sensor_data)
        self.create_subscription(String, str(self.get_parameter("pen_features_topic").value), self._on_feature, qos_profile_sensor_data)
        self.create_subscription(String, str(self.get_parameter("plane_topic").value), self._on_plane, qos_profile_sensor_data)
        self.create_subscription(Image, str(self.get_parameter("depth_topic").value), self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, str(self.get_parameter("camera_info_topic").value), self._on_camera, qos_profile_sensor_data)
        self.create_timer(float(self.get_parameter("publish_period_sec").value), self._on_timer)
        self._status("warn" if not self._trust.valid else "ok", trusted_for_grasp=self._trust.valid, reasons=list(self._trust.reasons))

    def _load_trust(self) -> ExtrinsicsValidation:
        try:
            extrinsics = load_yaml_document(str(self.get_parameter("extrinsics_path").value))
            stereo = load_yaml_document(str(self.get_parameter("stereo_calibration_path").value))
            return validate_extrinsics(extrinsics, stereo_document=stereo)
        except (OSError, ValueError) as exc:
            return ExtrinsicsValidation(False, (str(exc),))

    def _status(self, level: str, **values: Any) -> None:
        message = String()
        message.data = json.dumps({"level": level, **values}, ensure_ascii=False, separators=(",", ":"))
        self._status_pub.publish(message)

    def _on_feature(self, msg: String) -> None:
        try:
            feature = json.loads(msg.data)
            stamp_ns = pen_feature_stamp_ns(feature)
            if stamp_ns <= 0:
                raise ValueError("pen_feature_stamp_missing_or_invalid")
        except (json.JSONDecodeError, AttributeError, TypeError) as exc:
            self._status("warn", trusted_for_grasp=False, reason=f"invalid_pen_feature_json:{exc}")
            return
        except ValueError as exc:
            self._status("warn", trusted_for_grasp=False, reason=str(exc))
            return
        now_ns = time.monotonic_ns()
        ready = self._unmatched_ready.pop(stamp_ns, now_ns)
        if ready is None:
            self._features.put(stamp_ns, feature, now_ns)
        else:
            self._build_and_publish(stamp_ns, *ready, feature)

    def _on_depth(self, msg: Image) -> None:
        try:
            frame = DepthSample(_stamp_ns(msg), str(msg.header.frame_id), int(msg.width), int(msg.height), str(msg.encoding), depth_msg_to_array(msg))
        except RuntimeError as exc:
            self._status("warn", trusted_for_grasp=False, reason=str(exc))
            return
        pair = self._depth_info_pairs.add_left(frame.stamp_ns, frame, time.monotonic_ns())
        if pair is not None:
            self._accept_depth_info_pair(*pair)

    def _on_camera(self, msg: CameraInfo) -> None:
        pair = self._depth_info_pairs.add_right(_stamp_ns(msg), msg, time.monotonic_ns())
        if pair is not None:
            self._accept_depth_info_pair(*pair)

    def _accept_depth_info_pair(self, stamp_ns: int, frame: DepthSample, info: CameraInfo) -> None:
        now_ns = time.monotonic_ns()
        contract = validate_rectified_depth_pair(
            depth_stamp_ns=frame.stamp_ns, depth_frame_id=frame.frame_id, depth_width=frame.width,
            depth_height=frame.height, depth_encoding=frame.encoding, info_stamp_ns=_stamp_ns(info),
            info_frame_id=str(info.header.frame_id), info_width=int(info.width), info_height=int(info.height), projection=info.p,
        )
        if not contract.valid:
            self._status("warn", trusted_for_grasp=False, reason="rectified_depth_camera_info_contract:" + ",".join(contract.reasons))
            return
        plane = self._unmatched_planes.pop(stamp_ns, now_ns)
        if plane is None:
            self._unmatched_pairs.put(stamp_ns, (frame, info), now_ns)
        else:
            self._ready.put(stamp_ns, (frame, info, plane), now_ns)

    def _on_plane(self, msg: String) -> None:
        try:
            plane = json.loads(msg.data)
            stamp_ns = int(plane.get("stamp_sec", 0) or 0) * 1_000_000_000 + int(plane.get("stamp_nanosec", 0) or 0)
        except (json.JSONDecodeError, AttributeError, TypeError, ValueError) as exc:
            self._status("warn", trusted_for_grasp=False, reason=f"invalid_plane_json:{exc}")
            return
        now_ns = time.monotonic_ns()
        pair = self._unmatched_pairs.pop(stamp_ns, now_ns)
        if pair is None:
            self._unmatched_planes.put(stamp_ns, plane, now_ns)
        else:
            self._ready.put(stamp_ns, (pair[0], pair[1], plane), now_ns)

    def _on_timer(self) -> None:
        now_ns = time.monotonic_ns()
        self._depth_info_pairs.expire(now_ns)
        self._unmatched_pairs.expire(now_ns)
        self._unmatched_planes.expire(now_ns)
        self._unmatched_ready.expire(now_ns)
        self._features.expire(now_ns)
        ready = self._ready.pop_oldest(now_ns)
        if ready is None:
            return
        stamp_ns, (frame, info, plane) = ready
        feature = self._features.pop(stamp_ns, now_ns)
        if feature is None:
            self._unmatched_ready.put(stamp_ns, (frame, info, plane), now_ns)
            self._status("waiting", trusted_for_grasp=False, reason="waiting_for_exact_stamp_pen_feature")
            return
        self._build_and_publish(stamp_ns, frame, info, plane, feature)

    def _build_and_publish(self, stamp_ns: int, frame: DepthSample, info: CameraInfo, plane: dict[str, Any], feature: dict[str, Any]) -> None:
        if not feature_matches_depth_stamp(feature, stamp_ns):
            self._status("warn", trusted_for_grasp=False, reason="pen_feature_depth_timestamp_mismatch")
            return
        plane_contract = validate_dynamic_plane_for_depth(plane, depth_stamp_ns=stamp_ns, depth_frame_id=frame.frame_id)
        if not plane_contract.valid:
            self._status("warn", trusted_for_grasp=False, reason="table_plane_depth_contract:" + ",".join(plane_contract.reasons))
            return
        try:
            result = build_pen_candidates(
                feature, frame.depth, rectified_intrinsics(info.p), plane_payload=plane,
                rotation=self._trust.rotation, translation=self._trust.translation, trusted_for_grasp=self._trust.valid,
                min_depth_m=float(self.get_parameter("min_depth_m").value), max_depth_m=float(self.get_parameter("max_depth_m").value),
                min_plane_clearance_m=float(self.get_parameter("min_plane_clearance_m").value), edge_margin_px=int(self.get_parameter("edge_margin_px").value),
            )
        except ValueError as exc:
            result = {"valid": False, "trusted_for_grasp": False, "reason": str(exc), "candidate_count": 0, "candidates": []}
        result.update({"stamp_sec": stamp_ns // 1_000_000_000, "stamp_nanosec": stamp_ns % 1_000_000_000, "source_frame": frame.frame_id, "extrinsics_calibration_id": self._trust.calibration_id})
        message = String()
        message.data = json.dumps(result, ensure_ascii=False, separators=(",", ":"))
        self._output.publish(message)
        self._status("ok" if result["valid"] else "warn", trusted_for_grasp=bool(result["trusted_for_grasp"]), reason=result["reason"], confidence=result.get("confidence", 0.0))


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = PenGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
