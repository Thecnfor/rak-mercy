"""地面/桌面坐标系节点。

从深度图反投影出相机系点云，用 RANSAC 拟合最大水平平面（桌面/地面），
建立 ground 坐标系，并通过 TF 广播 ground <- camera 的变换。

设计目标：
- 相机俯仰角可动时，每帧都从深度重新估计桌面平面，camera -> ground 变换自动更新，
  不依赖电机角度反馈，也不依赖机械臂正解。
- 输出桌面平面参数（法向量、距离、内点中心），供目标定位使用。
"""

from __future__ import annotations

import json
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


def compact_float(value: float) -> float:
    return round(float(value), 4)


def rotation_matrix_to_quaternion(matrix: np.ndarray) -> tuple[float, float, float, float]:
    r = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(r))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = np.sqrt(max(1.0 + r[0, 0] - r[1, 1] - r[2, 2], 0.0)) * 2.0
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = np.sqrt(max(1.0 + r[1, 1] - r[0, 0] - r[2, 2], 0.0)) * 2.0
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = np.sqrt(max(1.0 + r[2, 2] - r[0, 0] - r[1, 1], 0.0)) * 2.0
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    norm = float(np.sqrt(w * w + x * x + y * y + z * z))
    return (x / norm, y / norm, z / norm, w / norm)


def fit_plane_ransac(
    points: np.ndarray,
    distance_threshold: float,
    iterations: int,
    seed: Optional[int] = None,
) -> tuple[Optional[np.ndarray], Optional[np.ndarray], int]:
    """RANSAC 拟合平面，返回 (法向量, 内点中心, 内点数量)。

    法向量为单位向量，满足 normal·p + d = 0，其中 d = -normal·center。
    """
    count = int(points.shape[0])
    if count < 3:
        return None, None, 0

    rng = np.random.default_rng(seed)
    best_inliers: Optional[np.ndarray] = None
    best_normal: Optional[np.ndarray] = None
    best_center: Optional[np.ndarray] = None
    best_count = 0

    for _ in range(max(1, int(iterations))):
        sample_idx = rng.choice(count, 3, replace=False)
        p0, p1, p2 = points[sample_idx]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = float(np.linalg.norm(normal))
        if norm < 1e-9:
            continue
        normal = normal / norm
        d = float(-np.dot(normal, p0))
        distances = np.abs(points @ normal + d)
        inlier_mask = distances < distance_threshold
        inlier_count = int(inlier_mask.sum())
        if inlier_count > best_count:
            best_count = inlier_count
            best_inliers = inlier_mask
            best_normal = normal
            best_center = p0

    if best_inliers is None or best_count < 3:
        return None, None, 0

    inlier_points = points[best_inliers]
    center = inlier_points.mean(axis=0)
    centered = inlier_points - center
    _, _, vt = np.linalg.svd(centered)
    normal = vt[-1].astype(np.float64)
    if best_normal is not None and float(np.dot(normal, best_normal)) < 0.0:
        normal = -normal
    normal = normal / (float(np.linalg.norm(normal)) + 1e-12)
    return normal, center, best_count


def build_ground_frame(normal: np.ndarray, center: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """由桌面平面法向量和内点中心构建 camera -> ground 的变换。

    返回 (rotation_c2g, translation_c2g)，满足：
    p_ground = rotation_c2g @ (p_camera - translation_c2g)
    """
    z_axis = normal.astype(np.float64)
    if float(z_axis[2]) < 0.0:
        z_axis = -z_axis
    z_axis = z_axis / (float(np.linalg.norm(z_axis)) + 1e-12)

    x_axis = np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
    x_axis = x_axis - float(np.dot(x_axis, z_axis)) * z_axis
    x_norm = float(np.linalg.norm(x_axis))
    if x_norm < 1e-6:
        x_axis = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
        x_axis = x_axis - float(np.dot(x_axis, z_axis)) * z_axis
        x_axis = x_axis / (float(np.linalg.norm(x_axis)) + 1e-12)
    else:
        x_axis = x_axis / x_norm

    y_axis = np.cross(z_axis, x_axis)
    y_axis = y_axis / (float(np.linalg.norm(y_axis)) + 1e-12)

    rotation_c2g = np.stack([x_axis, y_axis, z_axis], axis=0)
    translation_c2g = center.astype(np.float64)
    return rotation_c2g, translation_c2g


class GroundPlaneNode(Node):
    def __init__(self) -> None:
        super().__init__("ground_plane_node")

        defaults = {
            "depth_topic": "/x1/stereo/depth",
            "camera_info_topic": "/x1/left_camera/camera_info",
            "plane_topic": "/x1/ground/plane",
            "status_topic": "/x1/ground/plane_status",
            "camera_frame": "left_camera_optical_frame",
            "ground_frame": "ground",
            "publish_period_sec": 0.30,
            "min_depth_m": 0.20,
            "max_depth_m": 1.50,
            "sample_step": 2,
            "max_points": 6000,
            "ransac_distance_threshold": 0.02,
            "ransac_iterations": 120,
            "min_inlier_ratio": 0.20,
            "publish_tf": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._depth_frame: Optional[DepthFrame] = None
        self._camera_info: Optional[CameraInfo] = None
        self._last_published_stamp_ns = -1
        self._last_normal: Optional[np.ndarray] = None
        self._last_center: Optional[np.ndarray] = None

        self._depth_topic = str(self.get_parameter("depth_topic").value)
        self._camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self._plane_topic = str(self.get_parameter("plane_topic").value)
        self._status_topic = str(self.get_parameter("status_topic").value)
        self._camera_frame = str(self.get_parameter("camera_frame").value)
        self._ground_frame = str(self.get_parameter("ground_frame").value)
        self._min_depth_m = float(self.get_parameter("min_depth_m").value)
        self._max_depth_m = float(self.get_parameter("max_depth_m").value)
        self._sample_step = max(1, int(self.get_parameter("sample_step").value))
        self._max_points = max(32, int(self.get_parameter("max_points").value))
        self._ransac_threshold = float(self.get_parameter("ransac_distance_threshold").value)
        self._ransac_iterations = max(1, int(self.get_parameter("ransac_iterations").value))
        self._min_inlier_ratio = float(self.get_parameter("min_inlier_ratio").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)

        self._plane_pub = self.create_publisher(
            String, self._plane_topic, qos_profile_sensor_data
        )
        self._status_pub = self.create_publisher(
            String, self._status_topic, qos_profile_sensor_data
        )
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Image, self._depth_topic, self._on_depth, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, self._camera_info_topic, self._on_camera_info, qos_profile_sensor_data
        )
        self.create_timer(float(self.get_parameter("publish_period_sec").value), self._on_timer)

        self.get_logger().info(
            "ground_plane_node started: "
            f"depth={self._depth_topic} "
            f"camera_frame={self._camera_frame} "
            f"ground_frame={self._ground_frame} "
            f"ransac_threshold={self._ransac_threshold} "
            f"min_inlier_ratio={self._min_inlier_ratio}"
        )

    def _publish_status(self, level: str, message: str) -> None:
        payload = {
            "level": level,
            "message": message,
            "ground_frame": self._ground_frame,
            "stamp": self.get_clock().now().to_msg().sec,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self._status_pub.publish(msg)

    def _on_depth(self, msg: Image) -> None:
        self._depth_frame = DepthFrame(
            stamp_ns=_stamp_ns(msg),
            frame_id=msg.header.frame_id or self._camera_frame,
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
        self._plane_pub.publish(msg)
        self._publish_status(
            "ok",
            "ground plane ready: "
            f"inliers={payload['inlier_count']} "
            f"ratio={payload['inlier_ratio']} "
            f"height={payload['ground_height_m']}",
        )
        self._last_published_stamp_ns = frame.stamp_ns

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
        valid_mask &= np.where(valid_mask, sampled, self._min_depth_m - 1.0) >= self._min_depth_m
        valid_mask &= np.where(valid_mask, sampled, self._max_depth_m + 1.0) <= self._max_depth_m
        if int(valid_mask.sum()) < 3:
            raise RuntimeError("当前深度图没有可用点云样本")

        rows, cols = np.nonzero(valid_mask)
        rows = rows.astype(np.float32) * step
        cols = cols.astype(np.float32) * step
        z = sampled[valid_mask].astype(np.float32)

        x = ((cols - cx) * z) / fx
        y = ((rows - cy) * z) / fy
        points_camera = np.stack([x, y, z], axis=1).astype(np.float64)

        if points_camera.shape[0] > self._max_points:
            indices = np.linspace(0, points_camera.shape[0] - 1, num=self._max_points, dtype=np.int32)
            points_camera = points_camera[indices]

        normal, center, inlier_count = fit_plane_ransac(
            points_camera,
            distance_threshold=self._ransac_threshold,
            iterations=self._ransac_iterations,
        )
        if normal is None or center is None or inlier_count < 3:
            raise RuntimeError("RANSAC 未能拟合出有效平面")

        inlier_ratio = float(inlier_count) / float(points_camera.shape[0])
        if inlier_ratio < self._min_inlier_ratio:
            raise RuntimeError(f"平面内点比例过低: {inlier_ratio:.3f} < {self._min_inlier_ratio}")

        rotation_c2g, translation_c2g = build_ground_frame(normal, center)

        # 时间平滑：内点比例过低时回退到上一帧的平面，避免抖动。
        if self._last_normal is not None and self._last_center is not None:
            angle = float(np.arccos(np.clip(float(np.dot(normal, self._last_normal)), -1.0, 1.0)))
            if angle > np.deg2rad(15.0):
                normal = self._last_normal
                center = self._last_center
                rotation_c2g, translation_c2g = build_ground_frame(normal, center)

        self._last_normal = normal
        self._last_center = center

        if self._publish_tf:
            self._broadcast_tf(rotation_c2g, translation_c2g, frame)

        quat = rotation_matrix_to_quaternion(rotation_c2g.T)
        stamp_sec = int(frame.stamp_ns // 1_000_000_000)
        stamp_nanosec = int(frame.stamp_ns % 1_000_000_000)
        return {
            "stamp_sec": stamp_sec,
            "stamp_nanosec": stamp_nanosec,
            "camera_frame": frame.frame_id,
            "ground_frame": self._ground_frame,
            "plane_normal": [compact_float(v) for v in normal],
            "plane_center_camera_m": [compact_float(v) for v in center],
            "inlier_count": int(inlier_count),
            "inlier_ratio": compact_float(inlier_ratio),
            "ground_height_m": compact_float(float(np.dot(normal, center))),
            "rotation_c2g": [[compact_float(v) for v in row] for row in rotation_c2g],
            "translation_c2g_m": [compact_float(v) for v in translation_c2g],
            "quaternion_xyzw": [compact_float(v) for v in quat],
        }

    def _broadcast_tf(
        self,
        rotation_c2g: np.ndarray,
        translation_c2g: np.ndarray,
        frame: DepthFrame,
    ) -> None:
        # ground 相对 camera 的旋转 = rotation_c2g 的转置（把 ground 轴转到 camera 轴）。
        quat = rotation_matrix_to_quaternion(rotation_c2g.T)
        transform = TransformStamped()
        transform.header.stamp.sec = int(frame.stamp_ns // 1_000_000_000)
        transform.header.stamp.nanosec = int(frame.stamp_ns % 1_000_000_000)
        transform.header.frame_id = frame.frame_id
        transform.child_frame_id = self._ground_frame
        transform.transform.translation.x = float(translation_c2g[0])
        transform.transform.translation.y = float(translation_c2g[1])
        transform.transform.translation.z = float(translation_c2g[2])
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
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


if __name__ == "__main__":
    main()
