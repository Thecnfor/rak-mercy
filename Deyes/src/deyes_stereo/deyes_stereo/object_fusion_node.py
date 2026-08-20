from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Optional

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .depth_coordinate_node import (
    DepthFrame,
    compact_float,
    depth_msg_to_array,
    euler_deg_to_rotation_matrix,
    normalize_vec3,
    quaternion_to_rotation_matrix,
)


@dataclass
class DetectionFrame:
    stamp_ns: int
    frame_id: str
    detections: list[dict[str, Any]]


def stamp_ns_from_msg(msg: Any) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def clamp_int(value: Any, minimum: int, maximum: int) -> int:
    return max(minimum, min(int(value), maximum))


class ObjectFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("object_fusion_node")

        defaults = {
            "detection_topic": "/x1/detection/boxes",
            "depth_topic": "/x1/stereo/depth",
            "camera_info_topic": "/x1/stereo/left/camera_info_rect",
            "target_frame": "base_link",
            "output_topic": "/x1/detection/objects_3d",
            "status_topic": "/x1/detection/objects_3d_status",
            "publish_period_sec": 0.12,
            "transform_timeout_sec": 0.05,
            "max_detection_age_sec": 0.25,
            "min_depth_m": 0.20,
            "max_depth_m": 1.00,
            "roi_shrink_ratio": 0.20,
            "min_valid_ratio": 0.08,
            "use_tf_transform": True,
            "use_manual_transform_fallback": True,
            "manual_translation_m": [0.0, 0.0, 0.0],
            "manual_rpy_deg": [0.0, 0.0, 0.0],
            "source_frame_override": "",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._depth_frame: Optional[DepthFrame] = None
        self._camera_info: Optional[CameraInfo] = None
        self._detection_frame: Optional[DetectionFrame] = None
        self._last_published_key = ""

        self._target_frame = str(self.get_parameter("target_frame").value)
        self._transform_timeout = Duration(
            seconds=float(self.get_parameter("transform_timeout_sec").value)
        )
        self._max_detection_age_ns = int(
            float(self.get_parameter("max_detection_age_sec").value) * 1_000_000_000
        )
        self._min_depth_m = float(self.get_parameter("min_depth_m").value)
        self._max_depth_m = float(self.get_parameter("max_depth_m").value)
        self._roi_shrink_ratio = float(self.get_parameter("roi_shrink_ratio").value)
        self._min_valid_ratio = float(self.get_parameter("min_valid_ratio").value)
        self._use_tf_transform = bool(self.get_parameter("use_tf_transform").value)
        self._use_manual_transform_fallback = bool(
            self.get_parameter("use_manual_transform_fallback").value
        )
        self._manual_translation = normalize_vec3(
            self.get_parameter("manual_translation_m").value, "manual_translation_m"
        )
        self._manual_rpy_deg = normalize_vec3(
            self.get_parameter("manual_rpy_deg").value, "manual_rpy_deg"
        )
        self._source_frame_override = str(self.get_parameter("source_frame_override").value).strip()

        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._objects_pub = self.create_publisher(
            String, str(self.get_parameter("output_topic").value), qos_profile_sensor_data
        )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), qos_profile_sensor_data
        )

        self.create_subscription(
            String,
            str(self.get_parameter("detection_topic").value),
            self._on_detections,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_timer(float(self.get_parameter("publish_period_sec").value), self._on_timer)

        self.get_logger().info(
            "object_fusion_node started: "
            f"detection_topic={str(self.get_parameter('detection_topic').value)} "
            f"depth_topic={str(self.get_parameter('depth_topic').value)} "
            f"target_frame={self._target_frame} "
            f"output_topic={str(self.get_parameter('output_topic').value)}"
        )

    def _publish_status(self, level: str, message: str) -> None:
        payload = {"level": level, "message": message}
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self._status_pub.publish(msg)

    def _on_detections(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self._publish_status("error", f"检测载荷不是合法 JSON: {exc}")
            return

        detections = list(payload.get("detections") or [])
        stamp_sec = int(payload.get("stamp_sec", 0) or 0)
        stamp_nanosec = int(payload.get("stamp_nanosec", 0) or 0)
        frame_id = str(payload.get("frame_id") or payload.get("source_frame") or "").strip()
        if not frame_id:
            frame_id = "left_camera_optical_frame"

        self._detection_frame = DetectionFrame(
            stamp_ns=stamp_sec * 1_000_000_000 + stamp_nanosec,
            frame_id=frame_id,
            detections=detections,
        )

    def _on_depth(self, msg: Image) -> None:
        self._depth_frame = DepthFrame(
            stamp_ns=stamp_ns_from_msg(msg),
            frame_id=msg.header.frame_id or "left_camera_optical_frame",
            width=msg.width,
            height=msg.height,
            depth=depth_msg_to_array(msg),
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _resolve_transform(self, frame_id: str) -> tuple[np.ndarray, np.ndarray, str]:
        source_frame = self._source_frame_override or frame_id
        if self._use_tf_transform:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._target_frame,
                    source_frame,
                    Time(),
                    timeout=self._transform_timeout,
                )
                q = transform.transform.rotation
                t = transform.transform.translation
                return (
                    quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w),
                    np.asarray([t.x, t.y, t.z], dtype=np.float32),
                    "tf",
                )
            except TransformException:
                if not self._use_manual_transform_fallback:
                    raise

        rotation_matrix = euler_deg_to_rotation_matrix(
            float(self._manual_rpy_deg[0]),
            float(self._manual_rpy_deg[1]),
            float(self._manual_rpy_deg[2]),
        )
        return rotation_matrix, self._manual_translation.astype(np.float32), "manual"

    def _fuse_one_detection(
        self,
        detection: dict[str, Any],
        depth_frame: DepthFrame,
        camera_info: CameraInfo,
        rotation: np.ndarray,
        translation: np.ndarray,
    ) -> dict[str, Any]:
        bbox = detection.get("bbox_xyxy") or detection.get("bbox")
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            return {
                "status": "invalid_bbox",
                "label": detection.get("class_name") or detection.get("label") or "unknown",
                "confidence": float(detection.get("confidence", 0.0) or 0.0),
            }

        x0, y0, x1, y1 = [float(v) for v in bbox]
        width = max(1.0, x1 - x0)
        height = max(1.0, y1 - y0)
        inset_x = width * self._roi_shrink_ratio * 0.5
        inset_y = height * self._roi_shrink_ratio * 0.5
        roi_x0 = clamp_int(round(x0 + inset_x), 0, depth_frame.width - 1)
        roi_y0 = clamp_int(round(y0 + inset_y), 0, depth_frame.height - 1)
        roi_x1 = clamp_int(round(x1 - inset_x), roi_x0 + 1, depth_frame.width)
        roi_y1 = clamp_int(round(y1 - inset_y), roi_y0 + 1, depth_frame.height)

        roi = depth_frame.depth[roi_y0:roi_y1, roi_x0:roi_x1]
        sampled = np.where(np.isfinite(roi), roi, -1.0)
        valid_mask = (sampled >= self._min_depth_m) & (sampled <= self._max_depth_m)
        total_pixels = max(1, roi.shape[0] * roi.shape[1])
        valid_ratio = float(np.count_nonzero(valid_mask)) / float(total_pixels)
        if valid_ratio < self._min_valid_ratio or not np.any(valid_mask):
            return {
                "status": "low_depth_confidence",
                "label": detection.get("class_name") or detection.get("label") or "unknown",
                "confidence": float(detection.get("confidence", 0.0) or 0.0),
                "bbox_xyxy": [compact_float(v) for v in [x0, y0, x1, y1]],
                "valid_ratio": compact_float(valid_ratio),
            }

        valid_depths = sampled[valid_mask]
        median_depth = float(np.median(valid_depths))
        center_u = float((roi_x0 + roi_x1 - 1) * 0.5)
        center_v = float((roi_y0 + roi_y1 - 1) * 0.5)

        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])
        x_cam = (center_u - cx) * median_depth / fx
        y_cam = (center_v - cy) * median_depth / fy
        camera_point = np.asarray([x_cam, y_cam, median_depth], dtype=np.float32)
        base_point = rotation @ camera_point + translation

        return {
            "status": "ok",
            "label": detection.get("class_name") or detection.get("label") or "unknown",
            "class_id": detection.get("class_id"),
            "confidence": compact_float(float(detection.get("confidence", 0.0) or 0.0)),
            "bbox_xyxy": [compact_float(v) for v in [x0, y0, x1, y1]],
            "roi_xyxy": [int(roi_x0), int(roi_y0), int(roi_x1), int(roi_y1)],
            "center_px": [compact_float(center_u), compact_float(center_v)],
            "depth_median_m": compact_float(median_depth),
            "valid_ratio": compact_float(valid_ratio),
            "center_camera_m": [compact_float(v) for v in camera_point.tolist()],
            "center_base_m": [compact_float(v) for v in base_point.tolist()],
        }

    def _build_payload(self) -> dict[str, Any]:
        if self._depth_frame is None:
            raise RuntimeError("尚未收到深度图")
        if self._camera_info is None:
            raise RuntimeError("尚未收到 CameraInfo")
        if self._detection_frame is None:
            raise RuntimeError("尚未收到检测载荷")

        depth_frame = self._depth_frame
        detection_frame = self._detection_frame
        if detection_frame.stamp_ns > 0 and abs(depth_frame.stamp_ns - detection_frame.stamp_ns) > self._max_detection_age_ns:
            raise RuntimeError("检测结果与深度帧时间差过大")

        rotation, translation, transform_source = self._resolve_transform(depth_frame.frame_id)
        fused = [
            self._fuse_one_detection(det, depth_frame, self._camera_info, rotation, translation)
            for det in detection_frame.detections
        ]

        return {
            "stamp_sec": depth_frame.stamp_ns // 1_000_000_000,
            "stamp_nanosec": depth_frame.stamp_ns % 1_000_000_000,
            "source_frame": depth_frame.frame_id,
            "target_frame": self._target_frame,
            "transform_source": transform_source,
            "detection_count": len(detection_frame.detections),
            "object_count": len(fused),
            "objects": fused,
            "stats": {
                "manual_translation_m": [compact_float(v) for v in self._manual_translation.tolist()],
                "manual_rpy_deg": [compact_float(v) for v in self._manual_rpy_deg.tolist()],
                "min_valid_ratio": compact_float(self._min_valid_ratio),
            },
        }

    def _on_timer(self) -> None:
        try:
            payload = self._build_payload()
        except Exception as exc:
            self._publish_status("warn", str(exc))
            return

        dedupe_key = f"{payload['stamp_sec']}:{payload['stamp_nanosec']}:{payload['object_count']}"
        if dedupe_key == self._last_published_key:
            return
        self._last_published_key = dedupe_key

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self._objects_pub.publish(msg)
        self._publish_status(
            "ok",
            f"objects={payload['object_count']} target_frame={self._target_frame} transform={payload['transform_source']}",
        )


def main() -> None:
    rclpy.init()
    node = ObjectFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
