#!/usr/bin/env python3
"""IMX219 双目相机 ROS 2 发布节点。

把 Jetson 上的两路 IMX219 CSI 相机发布为 ROS 2 图像话题，供后续
同步诊断、标定和深度链路使用。

采集 pipeline 复用实机 `mercury_grasp/grab_stereo.py` 已验证可用的
`nvarguscamerasrc -> nvvidconv -> videoconvert -> appsink` 方案，不依赖
官方 `mercury_camera.launch.py`（那套依赖 `astra_camera`，与实机硬件不符）。

话题默认对齐实机 `x1_vision` 的约定：
  - /x1/left_camera/image_raw
  - /x1/right_camera/image_raw
  - /x1/left_camera/camera_info
  - /x1/right_camera/camera_info

注意：
  - 左右图像时间戳当前使用 ROS 接收时刻近似（软件时间戳），仅用于初步
    同步诊断。真实硬件同步能力需后续用 group_hold 或硬件 PTS 单独验证。
  - camera_info 初始来自 spec 标定（未做物理标定），只保证尺寸/内参占位，
    不用于宣称可信深度。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import threading
import time
from typing import Any, Deque, Optional

import cv2
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def gst_pipeline(sensor_id: int, width: int, height: int, fps: int) -> str:
    """构造 nvarguscamerasrc 到 OpenCV 可读 BGR 的 GStreamer pipeline。"""
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width={width},height={height},"
        f"format=NV12,framerate={fps}/1 ! "
        f"nvvidconv ! video/x-raw,format=BGRx ! "
        f"videoconvert ! video/x-raw,format=BGR ! "
        f"appsink drop=1 max-buffers=1 sync=false"
    )


def _matrix_to_list(mat: np.ndarray) -> list:
    return [float(v) for v in mat.ravel().tolist()]


def load_camera_info(
    calib_path: str,
    side: str,
    frame_id: str,
    output_width: int,
    output_height: int,
) -> CameraInfo:
    """从 stereo_calib.yaml 读取左/右相机参数，生成 CameraInfo 消息。"""
    with open(calib_path, "r", encoding="utf-8") as f:
        c = yaml.safe_load(f)

    k_key = "K1" if side == "left" else "K2"
    d_key = "D1" if side == "left" else "D2"
    p_key = "P1" if side == "left" else "P2"

    K = np.asarray(c[k_key], dtype=np.float64)
    D = np.asarray(c[d_key], dtype=np.float64)

    calib_width = int(c["img_size"][0])
    calib_height = int(c["img_size"][1])
    scale_x = float(output_width) / float(calib_width)
    scale_y = float(output_height) / float(calib_height)

    K = K.copy()
    K[0, 0] *= scale_x
    K[0, 2] *= scale_x
    K[1, 1] *= scale_y
    K[1, 2] *= scale_y

    ci = CameraInfo()
    ci.header.frame_id = frame_id
    ci.width = int(output_width)
    ci.height = int(output_height)
    ci.distortion_model = "plumb_bob"
    ci.k = _matrix_to_list(K)
    ci.d = _matrix_to_list(D)

    # 校正旋转矩阵：spec 平行双目为 I，物理标定产物当前未存 R1/R2，先占位 I。
    ci.r = _matrix_to_list(np.eye(3, dtype=np.float64))

    # 投影矩阵：优先用 yaml 里的 P，否则用 K 拼接零平移列。
    if p_key in c:
        P = np.asarray(c[p_key], dtype=np.float64).copy()
        P[0, 0] *= scale_x
        P[0, 2] *= scale_x
        P[0, 3] *= scale_x
        P[1, 1] *= scale_y
        P[1, 2] *= scale_y
        P[1, 3] *= scale_y
        ci.p = _matrix_to_list(P)
    else:
        P = np.hstack([K, np.zeros((3, 1), dtype=np.float64)])
        ci.p = _matrix_to_list(P)

    return ci


@dataclass
class CaptureSnapshot:
    frame: np.ndarray
    stamp_sec: float
    seq: int


@dataclass
class CaptureState:
    latest: Optional[CaptureSnapshot] = None
    receipt_history: Deque[float] = field(default_factory=lambda: deque(maxlen=120))
    frame_count: int = 0
    total_failures: int = 0
    consecutive_failures: int = 0
    last_failure_sec: float = 0.0
    lock: threading.Lock = field(default_factory=threading.Lock)

    def update_success(self, frame: np.ndarray, stamp_sec: float) -> None:
        with self.lock:
            self.frame_count += 1
            self.latest = CaptureSnapshot(frame=frame, stamp_sec=stamp_sec, seq=self.frame_count)
            self.receipt_history.append(stamp_sec)
            self.consecutive_failures = 0

    def update_failure(self, stamp_sec: float) -> int:
        with self.lock:
            self.total_failures += 1
            self.consecutive_failures += 1
            self.last_failure_sec = stamp_sec
            return self.consecutive_failures

    def snapshot(self) -> Optional[CaptureSnapshot]:
        with self.lock:
            return self.latest

    def rate_hz(self) -> float:
        with self.lock:
            if len(self.receipt_history) < 2:
                return 0.0
            elapsed = self.receipt_history[-1] - self.receipt_history[0]
            if elapsed <= 0.0:
                return 0.0
            return (len(self.receipt_history) - 1) / elapsed


class Imx219StereoPublisher(Node):
    def __init__(self) -> None:
        super().__init__("imx219_stereo_publisher")

        self.declare_parameter("left_sensor_id", 0)
        self.declare_parameter("right_sensor_id", 1)
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("fps", 30)
        self.declare_parameter("calib_path", "")
        self.declare_parameter("left_image_topic", "/x1/left_camera/image_raw")
        self.declare_parameter("right_image_topic", "/x1/right_camera/image_raw")
        self.declare_parameter("left_info_topic", "/x1/left_camera/camera_info")
        self.declare_parameter("right_info_topic", "/x1/right_camera/camera_info")
        self.declare_parameter("left_frame_id", "left_camera_optical_frame")
        self.declare_parameter("right_frame_id", "right_camera_optical_frame")
        self.declare_parameter("publish_info", True)
        self.declare_parameter("publish_period_sec", 0.033333333)
        self.declare_parameter("target_publish_hz", 30.0)
        self.declare_parameter("pair_max_skew_ms", 20.0)
        self.declare_parameter("frame_stale_sec", 0.2)
        self.declare_parameter("capture_retry_sleep_sec", 0.002)
        self.declare_parameter("log_stats_period_sec", 2.0)
        self.declare_parameter("reuse_latest_frame", False)
        self.declare_parameter("output_encoding", "mono8")

        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        fps = int(self.get_parameter("fps").value)

        self._publish_info = bool(self.get_parameter("publish_info").value)
        self._output_encoding = str(self.get_parameter("output_encoding").value).lower()
        self._pair_max_skew_sec = float(self.get_parameter("pair_max_skew_ms").value) / 1000.0
        self._frame_stale_sec = float(self.get_parameter("frame_stale_sec").value)
        self._capture_retry_sleep_sec = float(self.get_parameter("capture_retry_sleep_sec").value)
        self._log_stats_period_sec = float(self.get_parameter("log_stats_period_sec").value)
        self._reuse_latest_frame = bool(self.get_parameter("reuse_latest_frame").value)
        self._target_publish_hz = float(self.get_parameter("target_publish_hz").value)

        # 图像/Info 统一使用 sensor 数据 QoS，与 sync_monitor 订阅端一致。
        image_qos = qos_profile_sensor_data
        info_qos = qos_profile_sensor_data

        self._left_image_pub = self.create_publisher(
            Image, self.get_parameter("left_image_topic").value, image_qos
        )
        self._right_image_pub = self.create_publisher(
            Image, self.get_parameter("right_image_topic").value, image_qos
        )
        self._left_info_pub = self.create_publisher(
            CameraInfo, self.get_parameter("left_info_topic").value, info_qos
        )
        self._right_info_pub = self.create_publisher(
            CameraInfo, self.get_parameter("right_info_topic").value, info_qos
        )

        self._left_cap = cv2.VideoCapture(
            gst_pipeline(
                int(self.get_parameter("left_sensor_id").value),
                width,
                height,
                fps,
            ),
            cv2.CAP_GSTREAMER,
        )
        self._right_cap = cv2.VideoCapture(
            gst_pipeline(
                int(self.get_parameter("right_sensor_id").value),
                width,
                height,
                fps,
            ),
            cv2.CAP_GSTREAMER,
        )

        if not self._left_cap.isOpened() or not self._right_cap.isOpened():
            raise RuntimeError("IMX219 双目相机打开失败，检查 nvarguscamerasrc 是否可用")

        # 尽量压低 OpenCV/GStreamer 内部缓冲，避免旧帧堆积拖低有效帧率。
        self._left_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._right_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # CameraInfo 从标定文件加载；路径为空时跳过发布。
        calib_path = str(self.get_parameter("calib_path").value)
        self._left_info: Optional[CameraInfo] = None
        self._right_info: Optional[CameraInfo] = None
        if self._publish_info and calib_path:
            self._left_info = load_camera_info(
                calib_path,
                "left",
                str(self.get_parameter("left_frame_id").value),
                width,
                height,
            )
            self._right_info = load_camera_info(
                calib_path,
                "right",
                str(self.get_parameter("right_frame_id").value),
                width,
                height,
            )

        configured_period = float(self.get_parameter("publish_period_sec").value)
        if self._target_publish_hz > 0.0:
            configured_period = 1.0 / self._target_publish_hz
        self._publish_period_sec = configured_period

        self._left_state = CaptureState()
        self._right_state = CaptureState()
        self._publish_history: Deque[float] = deque(maxlen=120)
        self._publish_count = 0
        self._dropped_skew_count = 0
        self._dropped_stale_count = 0
        self._waiting_for_pair_count = 0
        self._last_published_left_seq = 0
        self._last_published_right_seq = 0
        self._last_stats_log_sec = time.time()
        self._last_skew_ms = 0.0
        self._last_publish_duration_ms = 0.0
        self._stop = False
        self._left_thread = threading.Thread(
            target=self._capture_loop,
            args=("left", self._left_cap, self._left_state),
            name="imx219-left-capture",
            daemon=True,
        )
        self._right_thread = threading.Thread(
            target=self._capture_loop,
            args=("right", self._right_cap, self._right_state),
            name="imx219-right-capture",
            daemon=True,
        )
        self._left_thread.start()
        self._right_thread.start()

        self.create_timer(self._publish_period_sec, self._on_timer)
        self.get_logger().info(
            f"imx219_stereo_publisher started: {width}x{height}@{fps}, "
            f"target_publish_hz={self._target_publish_hz:.1f}, "
            f"sensor-id left={self.get_parameter('left_sensor_id').value}, "
            f"right={self.get_parameter('right_sensor_id').value}"
        )

    def _capture_loop(self, side: str, cap: cv2.VideoCapture, state: CaptureState) -> None:
        """每侧相机独立采集，避免顺序 read() 把另一侧拖慢。"""
        while not self._stop:
            ok, frame = cap.read()
            stamp_sec = time.time()
            if ok and frame is not None and frame.size > 0:
                if self._output_encoding == "mono8":
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                state.update_success(frame, stamp_sec)
                continue

            failure_count = state.update_failure(stamp_sec)
            if failure_count == 1 or failure_count % 30 == 0:
                self.get_logger().warning(
                    f"{side} capture read failed (consecutive_failures={failure_count})",
                    throttle_duration_sec=2.0,
                )
            time.sleep(self._capture_retry_sleep_sec)

    def _publish_rate_hz(self) -> float:
        if len(self._publish_history) < 2:
            return 0.0
        elapsed = self._publish_history[-1] - self._publish_history[0]
        if elapsed <= 0.0:
            return 0.0
        return (len(self._publish_history) - 1) / elapsed

    def _camera_info_msg(self, template: CameraInfo, stamp_sec: float) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp.sec = int(stamp_sec)
        msg.header.stamp.nanosec = int((stamp_sec - int(stamp_sec)) * 1_000_000_000)
        msg.header.frame_id = template.header.frame_id
        msg.height = template.height
        msg.width = template.width
        msg.distortion_model = template.distortion_model
        msg.d = list(template.d)
        msg.k = list(template.k)
        msg.r = list(template.r)
        msg.p = list(template.p)
        return msg

    def _latest_pair(self, now_sec: float) -> Optional[tuple[CaptureSnapshot, CaptureSnapshot, float]]:
        left = self._left_state.snapshot()
        right = self._right_state.snapshot()
        if left is None or right is None:
            self._waiting_for_pair_count += 1
            return None

        if not self._reuse_latest_frame:
            if left.seq <= self._last_published_left_seq or right.seq <= self._last_published_right_seq:
                self._waiting_for_pair_count += 1
                return None

        if now_sec - left.stamp_sec > self._frame_stale_sec or now_sec - right.stamp_sec > self._frame_stale_sec:
            self._dropped_stale_count += 1
            self.get_logger().warning(
                "latest stereo pair is stale, skip publish",
                throttle_duration_sec=2.0,
            )
            return None

        skew_sec = abs(left.stamp_sec - right.stamp_sec)
        self._last_skew_ms = skew_sec * 1000.0
        if skew_sec > self._pair_max_skew_sec:
            self._dropped_skew_count += 1
            self.get_logger().warning(
                f"stereo pair skew too large: {self._last_skew_ms:.2f} ms",
                throttle_duration_sec=2.0,
            )
            return None

        stamp_sec = (left.stamp_sec + right.stamp_sec) * 0.5
        return left, right, stamp_sec

    def _maybe_log_stats(self, now_sec: float) -> None:
        if now_sec - self._last_stats_log_sec < self._log_stats_period_sec:
            return
        self._last_stats_log_sec = now_sec
        self.get_logger().info(
            "stats "
            f"publish_hz={self._publish_rate_hz():.2f} "
            f"left_capture_hz={self._left_state.rate_hz():.2f} "
            f"right_capture_hz={self._right_state.rate_hz():.2f} "
            f"published={self._publish_count} "
            f"drop_skew={self._dropped_skew_count} "
            f"drop_stale={self._dropped_stale_count} "
            f"wait_pair={self._waiting_for_pair_count} "
            f"last_skew_ms={self._last_skew_ms:.2f} "
            f"publish_ms={self._last_publish_duration_ms:.2f}"
        )

    def _image_msg(self, frame: np.ndarray, stamp_sec: float, frame_id: str) -> Image:
        """把 BGR 图像转成 ROS Image，避免依赖 cv_bridge。"""
        if not frame.flags["C_CONTIGUOUS"]:
            frame = np.ascontiguousarray(frame)
        msg = Image()
        msg.header.stamp.sec = int(stamp_sec)
        msg.header.stamp.nanosec = int((stamp_sec - int(stamp_sec)) * 1_000_000_000)
        msg.header.frame_id = frame_id
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        if frame.ndim == 2:
            msg.encoding = "mono8"
            msg.step = int(frame.shape[1])
        else:
            msg.encoding = "bgr8"
            msg.step = int(frame.shape[1] * frame.shape[2])
        msg.is_bigendian = False
        msg.data = frame.tobytes()
        return msg

    def _on_timer(self) -> None:
        now_sec = time.time()
        result = self._latest_pair(now_sec)
        if result is None:
            self._maybe_log_stats(now_sec)
            return
        left, right, stamp_sec = result
        t_publish_start = time.time()
        self._publish_count += 1
        self._left_image_pub.publish(
            self._image_msg(left.frame, stamp_sec, str(self.get_parameter("left_frame_id").value))
        )
        self._right_image_pub.publish(
            self._image_msg(right.frame, stamp_sec, str(self.get_parameter("right_frame_id").value))
        )

        if self._publish_info and self._left_info is not None:
            self._left_info_pub.publish(self._camera_info_msg(self._left_info, stamp_sec))
            self._right_info_pub.publish(self._camera_info_msg(self._right_info, stamp_sec))

        self._publish_history.append(now_sec)
        self._last_published_left_seq = left.seq
        self._last_published_right_seq = right.seq
        self._last_publish_duration_ms = (time.time() - t_publish_start) * 1000.0
        if self._publish_count == 1 or self._publish_count % 30 == 0:
            self.get_logger().info(
                f"publish #{self._publish_count} stamp={stamp_sec:.3f} "
                f"skew_ms={self._last_skew_ms:.2f} "
                f"publish_ms={self._last_publish_duration_ms:.2f}"
            )
        self._maybe_log_stats(now_sec)

    def close(self) -> None:
        self._stop = True
        if self._left_thread.is_alive():
            self._left_thread.join(timeout=2.0)
        if self._right_thread.is_alive():
            self._right_thread.join(timeout=2.0)
        self._left_cap.release()
        self._right_cap.release()


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = Imx219StereoPublisher()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
