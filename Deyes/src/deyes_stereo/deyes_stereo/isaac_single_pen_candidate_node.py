"""ROS2 single-pen Isaac candidate bridge; perception only, never motion."""
from __future__ import annotations
import json
import time
from typing import Any
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from .depth_coordinate_node import depth_msg_to_array
from .vision_grasp_candidate_contract import build_camera_optical_pen_candidates


def _stamp(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def discard_stale_cache(cache, latest_stamp_ns, max_age_ns):
    """Drop unpaired messages older than the newest observed ROS stamp.

    This deliberately compares message stamps instead of wall clock time: Isaac
    replay/simulation time and the host clock need not share an epoch.  A cache
    entry at exactly the age boundary remains eligible for an equality join.
    """
    cutoff = int(latest_stamp_ns) - max(0, int(max_age_ns))
    stale = [stamp for stamp in cache if int(stamp) < cutoff]
    for stamp in stale:
        cache.pop(stamp, None)
    return len(stale)


class IsaacSinglePenCandidateNode(Node):
    def __init__(self) -> None:
        super().__init__("isaac_single_pen_candidate_node")
        defaults = {"pen_features_topic": "/x1_sim/detection/pen_features", "depth_topic": "/x1_sim/left_camera/depth", "camera_info_topic": "/x1_sim/left_camera/camera_info", "plane_topic": "/x1_sim/ground/plane", "output_topic": "/x1_sim/grasp/single_pen_candidate", "status_topic": "/x1_sim/grasp/single_pen_candidate_status", "camera_frame": "left_camera_optical_frame", "expected_scene_sha256": "", "max_age_sec": .5, "min_depth_m": .2, "max_depth_m": 2.5, "min_plane_clearance_m": .004, "edge_margin_px": 12}
        for name, value in defaults.items(): self.declare_parameter(name, value)
        self._features: dict[int, dict[str, Any]] = {}; self._depth: dict[int, tuple[Image, np.ndarray]] = {}; self._info: dict[int, CameraInfo] = {}; self._plane: dict[int, dict[str, Any]] = {}; self._latest_stamp_ns = 0
        self._out = self.create_publisher(String, str(self.get_parameter("output_topic").value), qos_profile_sensor_data); self._status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), qos_profile_sensor_data)
        self.create_subscription(String, str(self.get_parameter("pen_features_topic").value), self._on_features, qos_profile_sensor_data); self.create_subscription(Image, str(self.get_parameter("depth_topic").value), self._on_depth, qos_profile_sensor_data); self.create_subscription(CameraInfo, str(self.get_parameter("camera_info_topic").value), self._on_info, qos_profile_sensor_data); self.create_subscription(String, str(self.get_parameter("plane_topic").value), self._on_plane, qos_profile_sensor_data)
        self._tf = Buffer(); self._listener = TransformListener(self._tf, self, spin_thread=True); self.create_timer(.05, self._join)

    def _status(self, level: str, reason: str, **extra: Any) -> None:
        msg = String(); msg.data = json.dumps({"level": level, "reason": reason, "source": "isaac_sim", "simulation_only": True, "physical_execution_eligible": False, **extra}, separators=(",", ":")); self._status_pub.publish(msg)

    def _on_features(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data); stamp = int(payload.get("stamp_sec", 0)) * 1_000_000_000 + int(payload.get("stamp_nanosec", 0)); self._features[stamp] = payload; self._note_stamp(stamp)
        except (json.JSONDecodeError, TypeError, ValueError): self._status("warn", "features_invalid")

    def _on_depth(self, msg: Image) -> None:
        try:
            stamp = _stamp(msg); self._depth[stamp] = (msg, depth_msg_to_array(msg)); self._note_stamp(stamp)
        except RuntimeError as exc: self._status("warn", str(exc))

    def _on_info(self, msg: CameraInfo) -> None:
        stamp = _stamp(msg); self._info[stamp] = msg; self._note_stamp(stamp)

    def _on_plane(self, msg: String) -> None:
        try:
            value = json.loads(msg.data); stamp = int(value.get("stamp_sec", 0)) * 1_000_000_000 + int(value.get("stamp_nanosec", 0)); self._plane[stamp] = value; self._note_stamp(stamp)
        except (json.JSONDecodeError, TypeError, ValueError): self._status("warn", "plane_invalid")

    def _join(self) -> None:
        self._discard_stale()
        if not self._depth: return
        for stamp in list(self._depth):
            if stamp <= 0: self._depth.pop(stamp, None); continue
            if stamp not in self._features or stamp not in self._info or stamp not in self._plane: continue
            image, depth = self._depth.pop(stamp); info = self._info.pop(stamp); feature = self._features.pop(stamp); plane = self._plane.pop(stamp)
            self._process(stamp, image, depth, info, feature, plane)

    def _note_stamp(self, stamp: int) -> None:
        if stamp > self._latest_stamp_ns:
            self._latest_stamp_ns = stamp
        self._discard_stale()

    def _discard_stale(self) -> None:
        if self._latest_stamp_ns <= 0:
            return
        max_age_ns = int(float(self.get_parameter("max_age_sec").value) * 1e9)
        expired = {
            "features": discard_stale_cache(self._features, self._latest_stamp_ns, max_age_ns),
            "depth": discard_stale_cache(self._depth, self._latest_stamp_ns, max_age_ns),
            "camera_info": discard_stale_cache(self._info, self._latest_stamp_ns, max_age_ns),
            "plane": discard_stale_cache(self._plane, self._latest_stamp_ns, max_age_ns),
        }
        if any(expired.values()):
            self._status("warn", "unpaired_input_expired", latest_stamp_ns=self._latest_stamp_ns, expired=expired)

    def _process(self, stamp: int, image: Image, depth: np.ndarray, info: CameraInfo, feature: dict[str, Any], plane: dict[str, Any]) -> None:
        frame = str(image.header.frame_id); expected_frame = str(self.get_parameter("camera_frame").value)
        if frame != expected_frame or str(info.header.frame_id) != frame:
            self._status("warn", "frame_mismatch", stamp_ns=stamp); return
        scene = str(self.get_parameter("expected_scene_sha256").value)
        simulation = feature.get("simulation") if isinstance(feature.get("simulation"), dict) else {}
        if not scene or simulation.get("scene_sha256") != scene:
            self._status("warn", "scene_sha256_mismatch", stamp_ns=stamp); return
        result = build_camera_optical_pen_candidates(feature, depth, depth_stamp_ns=stamp, depth_frame_id=frame, depth_width=int(image.width), depth_height=int(image.height), depth_encoding=str(image.encoding), camera_stamp_ns=_stamp(info), camera_frame_id=str(info.header.frame_id), camera_width=int(info.width), camera_height=int(info.height), projection=tuple(float(v) for v in info.p), plane_payload=plane, source="isaac_sim", now_stamp_ns=stamp, max_candidate_age_ns=int(float(self.get_parameter("max_age_sec").value) * 1e9), min_depth_m=float(self.get_parameter("min_depth_m").value), max_depth_m=float(self.get_parameter("max_depth_m").value), min_plane_clearance_m=float(self.get_parameter("min_plane_clearance_m").value), edge_margin_px=int(self.get_parameter("edge_margin_px").value))
        if result.get("valid") is not True:
            self._publish_rejected(str(result.get("reason") or "candidate_result_invalid"), stamp); return
        if result.get("candidate_count") != 1:
            self._publish_rejected("exactly_one_feature_required", stamp); return
        item = result["candidates"][0]; tf = self._lookup(frame, stamp)
        if tf is None:
            self._publish_rejected("base_link_T_camera_missing", stamp); return
        rotation, translation = tf; point = np.asarray(item["grasp_point_camera_optical_m"], dtype=float); axis = np.asarray(item["axis_camera_optical_unit"], dtype=float); normal = np.asarray(item["approach_normal_camera_optical_unit"], dtype=float)
        item.update({"target_frame": "base_link", "grasp_point_base_m": (rotation @ point + translation).tolist(), "axis_base_unit": (rotation @ axis).tolist(), "approach_normal_base_unit": (rotation @ normal).tolist()})
        result.update({"schema": "isaac_single_pen_candidate/v1", "simulation_only": True, "physical_validated": False, "physical_execution_eligible": False, "scene_sha256": scene, "candidates": [item], "candidate_count": 1})
        self._publish(result); self._status("ok", "single_pen_candidate_ready", stamp_ns=stamp)

    def _lookup(self, frame: str, stamp: int) -> tuple[np.ndarray, np.ndarray] | None:
        try:
            value = self._tf.lookup_transform("base_link", frame, Time(nanoseconds=stamp), timeout=Duration(seconds=.05)).transform; q = np.asarray([value.rotation.x, value.rotation.y, value.rotation.z, value.rotation.w], dtype=float); q /= np.linalg.norm(q); x, y, z, w = q
            rotation = np.asarray([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
            return rotation, np.asarray([value.translation.x, value.translation.y, value.translation.z], dtype=float)
        except (TransformException, ValueError, np.linalg.LinAlgError): return None

    def _publish_rejected(self, reason: str, stamp: int) -> None: self._publish({"schema": "isaac_single_pen_candidate/v1", "valid": False, "reason": reason, "candidate_count": 0, "candidates": [], "stamp_ns": stamp, "source": "isaac_sim", "simulation_only": True, "physical_validated": False, "physical_execution_eligible": False})
    def _publish(self, value: dict[str, Any]) -> None: msg = String(); msg.data = json.dumps(value, separators=(",", ":")); self._out.publish(msg)


def main(args: Any = None) -> None:
    rclpy.init(args=args); node = IsaacSinglePenCandidateNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
