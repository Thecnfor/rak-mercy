from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Optional

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class DepthFrame:
    stamp_ns: int
    frame_id: str
    width: int
    height: int
    depth: np.ndarray


def _stamp_ns(msg: Any) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def depth_msg_to_array(msg: Image) -> np.ndarray:
    if msg.encoding == "32FC1":
        row = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)
        return row[:, : msg.width].copy()
    if msg.encoding == "16UC1":
        row = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)
        return (row[:, : msg.width].astype(np.float32) / 1000.0).copy()
    raise RuntimeError(f"unsupported depth encoding: {msg.encoding}")


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.asarray(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float32,
    )


def euler_deg_to_rotation_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    roll = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_deg)
    yaw = np.deg2rad(yaw_deg)

    cr = np.cos(roll)
    sr = np.sin(roll)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)

    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float32,
    )


def compact_float(value: float) -> float:
    return round(float(value), 4)


def normalize_vec3(values: Any, param_name: str) -> np.ndarray:
    try:
        vector = np.asarray(list(values), dtype=np.float32).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{param_name} 必须是长度为 3 的数字数组") from exc
    if vector.size != 3:
        raise ValueError(f"{param_name} 必须是长度为 3 的数字数组")
    return vector


def normalize_positive_int(value: Any, param_name: str, minimum: int) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{param_name} 必须是整数") from exc
    if parsed < minimum:
        raise ValueError(f"{param_name} 必须大于等于 {minimum}")
    return parsed


class DepthCoordinateNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_coordinate_node")

        defaults = {
            "depth_topic": "/x1/stereo/depth",
            "camera_info_topic": "/x1/left_camera/camera_info",
            "target_frame": "base_link",
            "heatmap_topic": "/x1/stereo/base_heatmap",
            "status_topic": "/x1/stereo/base_heatmap_status",
            "sample_step": 2,
            "max_points": 4000,
            "publish_period_sec": 0.20,
            "transform_timeout_sec": 0.05,
            "min_depth_m": 0.20,
            "max_depth_m": 1.00,
            "use_tf_transform": True,
            "use_manual_transform_fallback": False,
            "manual_translation_m": [0.0, 0.0, 0.0],
            "manual_rpy_deg": [0.0, 0.0, 0.0],
            "source_frame_override": "",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._depth_frame: Optional[DepthFrame] = None
        self._camera_info: Optional[CameraInfo] = None
        self._last_published_stamp_ns = -1

        self._target_frame = str(self.get_parameter("target_frame").value)
        self._sample_step = max(1, int(self.get_parameter("sample_step").value))
        self._max_points = max(32, int(self.get_parameter("max_points").value))
        self._transform_timeout = Duration(
            seconds=float(self.get_parameter("transform_timeout_sec").value)
        )
        self._min_depth_m = float(self.get_parameter("min_depth_m").value)
        self._max_depth_m = float(self.get_parameter("max_depth_m").value)
        self._apply_tuning_values(
            use_tf_transform=bool(self.get_parameter("use_tf_transform").value),
            use_manual_transform_fallback=bool(
                self.get_parameter("use_manual_transform_fallback").value
            ),
            manual_translation_m=self.get_parameter("manual_translation_m").value,
            manual_rpy_deg=self.get_parameter("manual_rpy_deg").value,
            source_frame_override=str(self.get_parameter("source_frame_override").value).strip(),
            sample_step=self._sample_step,
            max_points=self._max_points,
        )
        self.add_on_set_parameters_callback(self._on_parameters_set)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._heatmap_pub = self.create_publisher(
            String, str(self.get_parameter("heatmap_topic").value), qos_profile_sensor_data
        )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), qos_profile_sensor_data
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
            "depth_coordinate_node started: "
            f"depth={str(self.get_parameter('depth_topic').value)} "
            f"camera_info={str(self.get_parameter('camera_info_topic').value)} "
            f"target_frame={self._target_frame} "
            f"heatmap_topic={str(self.get_parameter('heatmap_topic').value)} "
            f"use_tf={self._use_tf_transform} "
            f"manual_fallback={self._use_manual_transform_fallback} "
            f"sample_step={self._sample_step} "
            f"max_points={self._max_points}"
        )

    def _apply_tuning_values(
        self,
        *,
        use_tf_transform: bool,
        use_manual_transform_fallback: bool,
        manual_translation_m: Any,
        manual_rpy_deg: Any,
        source_frame_override: str,
        sample_step: Any,
        max_points: Any,
    ) -> None:
        self._use_tf_transform = bool(use_tf_transform)
        self._use_manual_transform_fallback = bool(use_manual_transform_fallback)
        self._manual_translation = normalize_vec3(manual_translation_m, "manual_translation_m")
        self._manual_rpy_deg = normalize_vec3(manual_rpy_deg, "manual_rpy_deg")
        self._source_frame_override = str(source_frame_override).strip()
        self._sample_step = normalize_positive_int(sample_step, "sample_step", 1)
        self._max_points = normalize_positive_int(max_points, "max_points", 32)

    def _on_parameters_set(self, parameters: list[Parameter]) -> SetParametersResult:
        pending = {
            "use_tf_transform": self._use_tf_transform,
            "use_manual_transform_fallback": self._use_manual_transform_fallback,
            "manual_translation_m": self._manual_translation,
            "manual_rpy_deg": self._manual_rpy_deg,
            "source_frame_override": self._source_frame_override,
            "sample_step": self._sample_step,
            "max_points": self._max_points,
        }
        touched = False
        try:
            for parameter in parameters:
                if parameter.name not in pending:
                    continue
                touched = True
                if parameter.name in {"manual_translation_m", "manual_rpy_deg"}:
                    pending[parameter.name] = parameter.value
                elif parameter.name == "source_frame_override":
                    pending[parameter.name] = str(parameter.value or "").strip()
                elif parameter.name == "sample_step":
                    pending[parameter.name] = normalize_positive_int(parameter.value, "sample_step", 1)
                elif parameter.name == "max_points":
                    pending[parameter.name] = normalize_positive_int(parameter.value, "max_points", 32)
                else:
                    pending[parameter.name] = bool(parameter.value)
            if touched:
                self._apply_tuning_values(**pending)
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        if touched:
            self.get_logger().info(
                "runtime tuning updated: "
                f"use_tf={self._use_tf_transform} "
                f"manual_fallback={self._use_manual_transform_fallback} "
                f"translation={[compact_float(v) for v in self._manual_translation]} "
                f"rpy_deg={[compact_float(v) for v in self._manual_rpy_deg]} "
                f"source_override={self._source_frame_override or '<depth_frame>'} "
                f"sample_step={self._sample_step} "
                f"max_points={self._max_points}"
            )
        return SetParametersResult(successful=True)

    def _publish_status(self, level: str, message: str) -> None:
        payload = {
            "level": level,
            "message": message,
            "target_frame": self._target_frame,
            "stamp": self.get_clock().now().to_msg().sec,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self._status_pub.publish(msg)

    def _on_depth(self, msg: Image) -> None:
        self._depth_frame = DepthFrame(
            stamp_ns=_stamp_ns(msg),
            frame_id=msg.header.frame_id,
            width=int(msg.width),
            height=int(msg.height),
            depth=depth_msg_to_array(msg),
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _on_timer(self) -> None:
        frame = self._depth_frame
        info = self._camera_info
        if frame is None or info is None:
            self._publish_status("waiting", "等待 depth 与 camera_info")
            return
        if frame.stamp_ns == self._last_published_stamp_ns:
            return

        try:
            payload = self._build_payload(frame, info)
        except RuntimeError as exc:
            self._publish_status("invalid", str(exc))
            return

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self._heatmap_pub.publish(msg)
        self._publish_status(
            "ok",
            "base heatmap ready: "
            f"points={payload['point_count']} "
            f"source={payload['source_frame']} "
            f"target={payload['target_frame']} "
            f"transform={payload['transform_source']}",
        )
        self._last_published_stamp_ns = frame.stamp_ns

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
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                rotation_matrix = quaternion_to_rotation_matrix(
                    rotation.x, rotation.y, rotation.z, rotation.w
                )
                translation_vector = np.asarray(
                    [translation.x, translation.y, translation.z], dtype=np.float32
                )
                return rotation_matrix, translation_vector, "tf"
            except TransformException as exc:
                if not self._use_manual_transform_fallback:
                    raise RuntimeError(f"TF 查询失败且未启用手动外参回退: {exc}") from exc

        rotation_matrix = euler_deg_to_rotation_matrix(
            float(self._manual_rpy_deg[0]),
            float(self._manual_rpy_deg[1]),
            float(self._manual_rpy_deg[2]),
        )
        return rotation_matrix, self._manual_translation.astype(np.float32), "manual"

    def _build_payload(self, frame: DepthFrame, info: CameraInfo) -> dict[str, Any]:
        depth = frame.depth
        if depth.size == 0:
            raise RuntimeError("深度图为空")
        if len(info.k) < 9:
            raise RuntimeError("camera_info 缺少有效内参")

        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            raise RuntimeError("camera_info 内参非法")

        step = self._sample_step
        sampled = depth[::step, ::step]
        valid_mask = np.isfinite(sampled)
        sampled_min_checked = np.where(valid_mask, sampled, self._min_depth_m - 1.0)
        valid_mask &= sampled_min_checked >= self._min_depth_m
        sampled_max_checked = np.where(valid_mask, sampled, self._max_depth_m + 1.0)
        valid_mask &= sampled_max_checked <= self._max_depth_m
        if not np.any(valid_mask):
            raise RuntimeError("当前深度图没有可用坐标样本")

        rows, cols = np.nonzero(valid_mask)
        rows = rows.astype(np.float32) * step
        cols = cols.astype(np.float32) * step
        z = sampled[valid_mask].astype(np.float32)

        x = ((cols - cx) * z) / fx
        y = ((rows - cy) * z) / fy
        points_camera = np.stack([x, y, z], axis=1)

        if points_camera.shape[0] > self._max_points:
            indices = np.linspace(0, points_camera.shape[0] - 1, num=self._max_points, dtype=np.int32)
            points_camera = points_camera[indices]
            z = z[indices]

        rotation_matrix, translation_vector, transform_source = self._resolve_transform(frame.frame_id)
        points_base = (rotation_matrix @ points_camera.T).T + translation_vector

        min_z = float(np.min(points_base[:, 2]))
        max_z = float(np.max(points_base[:, 2]))
        if max_z > min_z + 1e-6:
            heat = (points_base[:, 2] - min_z) / (max_z - min_z)
        else:
            heat = np.full(points_base.shape[0], 0.5, dtype=np.float32)

        distances = np.linalg.norm(points_base, axis=1)
        centroid = np.mean(points_base, axis=0)
        bounds_min = np.min(points_base, axis=0)
        bounds_max = np.max(points_base, axis=0)

        points_payload = [
            [
                compact_float(px),
                compact_float(py),
                compact_float(pz),
                compact_float(ph),
                compact_float(pd),
            ]
            for (px, py, pz), ph, pd in zip(points_base, heat, distances)
        ]

        stamp_sec = int(frame.stamp_ns // 1_000_000_000)
        stamp_nanosec = int(frame.stamp_ns % 1_000_000_000)
        return {
            "stamp_sec": stamp_sec,
            "stamp_nanosec": stamp_nanosec,
            "source_frame": frame.frame_id,
            "target_frame": self._target_frame,
            "transform_source": transform_source,
            "image_size": [frame.width, frame.height],
            "sample_step": step,
            "point_count": len(points_payload),
            "encoding": "xyzhd",
            "points": points_payload,
            "stats": {
                "depth_range_m": [compact_float(float(np.min(z))), compact_float(float(np.max(z)))],
                "distance_range_m": [
                    compact_float(float(np.min(distances))),
                    compact_float(float(np.max(distances))),
                ],
                "height_range_m": [compact_float(min_z), compact_float(max_z)],
                "centroid_m": [compact_float(v) for v in centroid],
                "bounds_m": {
                    "min": [compact_float(v) for v in bounds_min],
                    "max": [compact_float(v) for v in bounds_max],
                },
                "manual_translation_m": [compact_float(v) for v in self._manual_translation],
                "manual_rpy_deg": [compact_float(v) for v in self._manual_rpy_deg],
            },
        }


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = DepthCoordinateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
