from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Tuple

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


@dataclass
class StereoCalibration:
    image_size: Tuple[int, int]
    k1: np.ndarray
    d1: np.ndarray
    k2: np.ndarray
    d2: np.ndarray
    r: np.ndarray
    t: np.ndarray
    q: Optional[np.ndarray]
    fx: float
    baseline_m: float


@dataclass
class FrameBundle:
    stamp_ns: int
    frame_id: str
    image: np.ndarray
    width: int
    height: int


def _stamp_ns(msg: Any) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def _matrix(data: Any, shape: Tuple[int, int]) -> np.ndarray:
    return np.asarray(data, dtype=np.float64).reshape(shape)


def load_stereo_calibration(calib_path: str) -> StereoCalibration:
    with open(calib_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    image_size = tuple(int(v) for v in data["img_size"])
    baseline_m = float(abs(data.get("baseline_m", 0.0)))
    t = np.asarray(data["T"], dtype=np.float64).reshape(-1)
    if baseline_m <= 0.0 and t.size > 0:
        baseline_m = float(abs(t[0]))

    q = data.get("Q")
    return StereoCalibration(
        image_size=(int(image_size[0]), int(image_size[1])),
        k1=_matrix(data["K1"], (3, 3)),
        d1=np.asarray(data["D1"], dtype=np.float64).reshape(-1),
        k2=_matrix(data["K2"], (3, 3)),
        d2=np.asarray(data["D2"], dtype=np.float64).reshape(-1),
        r=_matrix(data["R"], (3, 3)),
        t=np.asarray(data["T"], dtype=np.float64).reshape(-1, 1),
        q=np.asarray(q, dtype=np.float64).reshape(4, 4) if q is not None else None,
        fx=float(data.get("fx", data["K1"][0][0])),
        baseline_m=baseline_m,
    )


def _bytes_per_pixel(encoding: str) -> int:
    if encoding in ("bgr8", "rgb8"):
        return 3
    if encoding in ("mono8", "8UC1"):
        return 1
    raise ValueError(f"Unsupported encoding: {encoding}")


def image_msg_to_array(msg: Image) -> np.ndarray:
    bpp = _bytes_per_pixel(msg.encoding)
    row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    usable = row[:, : msg.width * bpp]
    if bpp == 1:
        return usable.reshape(msg.height, msg.width).copy()
    return usable.reshape(msg.height, msg.width, bpp).copy()


def _contiguous(frame: np.ndarray) -> np.ndarray:
    return frame if frame.flags["C_CONTIGUOUS"] else np.ascontiguousarray(frame)


def make_image_msg(frame: np.ndarray, stamp_ns: int, frame_id: str, encoding: str) -> Image:
    frame = _contiguous(frame)
    msg = Image()
    msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
    msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
    msg.header.frame_id = frame_id
    msg.height = int(frame.shape[0])
    msg.width = int(frame.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = False
    channels = 1 if frame.ndim == 2 else int(frame.shape[2])
    msg.step = int(frame.shape[1] * frame.dtype.itemsize * channels)
    msg.data = frame.tobytes()
    return msg


class SgbmBaselineNode(Node):
    def __init__(self) -> None:
        super().__init__("sgbm_baseline")

        defaults = {
            "calib_path": "",
            "left_image_topic": "/x1/left_camera/image_raw",
            "right_image_topic": "/x1/right_camera/image_raw",
            "left_camera_info_topic": "/x1/left_camera/camera_info",
            "right_camera_info_topic": "/x1/right_camera/camera_info",
            "max_sync_diff_ms": 10.0,
            "publish_period_sec": 0.10,
            "min_depth_m": 0.20,
            "max_depth_m": 1.50,
            "min_disparity": 0,
            "num_disparities": 64,
            "block_size": 11,
            "uniqueness_ratio": 10,
            "speckle_window_size": 100,
            "speckle_range": 32,
            "disp12_max_diff": 1,
            "pre_filter_cap": 31,
            "debug_left_rect_topic": "~/debug/left_rect",
            "debug_right_rect_topic": "~/debug/right_rect",
            "debug_disparity_topic": "~/debug/disparity",
            "debug_depth_topic": "~/debug/depth",
            "debug_valid_mask_topic": "~/debug/valid_mask",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        calib_path = str(self.get_parameter("calib_path").value)
        if not calib_path:
            raise RuntimeError("sgbm_baseline requires calib_path")
        self._calib = load_stereo_calibration(calib_path)

        self._left_image: Optional[FrameBundle] = None
        self._right_image: Optional[FrameBundle] = None
        self._left_info: Optional[CameraInfo] = None
        self._right_info: Optional[CameraInfo] = None
        self._last_processed_pair: Optional[Tuple[int, int]] = None

        self._map1_left = None
        self._map2_left = None
        self._map1_right = None
        self._map2_right = None
        self._q = self._calib.q

        self._matcher = self._build_matcher()
        self._max_sync_diff_ns = int(
            float(self.get_parameter("max_sync_diff_ms").value) * 1_000_000.0
        )
        self._min_depth_m = float(self.get_parameter("min_depth_m").value)
        self._max_depth_m = float(self.get_parameter("max_depth_m").value)

        self._left_rect_pub = self.create_publisher(
            Image, self.get_parameter("debug_left_rect_topic").value, qos_profile_sensor_data
        )
        self._right_rect_pub = self.create_publisher(
            Image, self.get_parameter("debug_right_rect_topic").value, qos_profile_sensor_data
        )
        self._disparity_pub = self.create_publisher(
            Image, self.get_parameter("debug_disparity_topic").value, qos_profile_sensor_data
        )
        self._depth_pub = self.create_publisher(
            Image, self.get_parameter("debug_depth_topic").value, qos_profile_sensor_data
        )
        self._valid_mask_pub = self.create_publisher(
            Image, self.get_parameter("debug_valid_mask_topic").value, qos_profile_sensor_data
        )

        self.create_subscription(
            Image,
            self.get_parameter("left_image_topic").value,
            self._on_left_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.get_parameter("right_image_topic").value,
            self._on_right_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("left_camera_info_topic").value,
            self._on_left_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("right_camera_info_topic").value,
            self._on_right_info,
            qos_profile_sensor_data,
        )

        self.create_timer(float(self.get_parameter("publish_period_sec").value), self._on_timer)
        self.get_logger().info("sgbm_baseline started")

    def _build_matcher(self) -> cv2.StereoSGBM:
        block_size = int(self.get_parameter("block_size").value)
        num_disparities = int(self.get_parameter("num_disparities").value)
        if num_disparities % 16 != 0:
            raise RuntimeError("num_disparities must be divisible by 16")
        p1 = 8 * block_size * block_size
        p2 = 32 * block_size * block_size
        return cv2.StereoSGBM_create(
            minDisparity=int(self.get_parameter("min_disparity").value),
            numDisparities=num_disparities,
            blockSize=block_size,
            uniquenessRatio=int(self.get_parameter("uniqueness_ratio").value),
            speckleWindowSize=int(self.get_parameter("speckle_window_size").value),
            speckleRange=int(self.get_parameter("speckle_range").value),
            disp12MaxDiff=int(self.get_parameter("disp12_max_diff").value),
            preFilterCap=int(self.get_parameter("pre_filter_cap").value),
            P1=p1,
            P2=p2,
        )

    def _frame_bundle(self, msg: Image) -> FrameBundle:
        return FrameBundle(
            stamp_ns=_stamp_ns(msg),
            frame_id=msg.header.frame_id,
            image=image_msg_to_array(msg),
            width=int(msg.width),
            height=int(msg.height),
        )

    def _on_left_image(self, msg: Image) -> None:
        self._left_image = self._frame_bundle(msg)

    def _on_right_image(self, msg: Image) -> None:
        self._right_image = self._frame_bundle(msg)

    def _on_left_info(self, msg: CameraInfo) -> None:
        self._left_info = msg

    def _on_right_info(self, msg: CameraInfo) -> None:
        self._right_info = msg

    def _ensure_rectify_maps(self, width: int, height: int) -> None:
        if self._map1_left is not None:
            return

        if (width, height) != self._calib.image_size:
            self.get_logger().warning(
                f"stream size {(width, height)} does not match calib image_size {self._calib.image_size}"
            )

        r1, r2, p1, p2, q, _, _ = cv2.stereoRectify(
            self._calib.k1,
            self._calib.d1,
            self._calib.k2,
            self._calib.d2,
            (width, height),
            self._calib.r,
            self._calib.t,
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=0,
        )
        self._map1_left, self._map2_left = cv2.initUndistortRectifyMap(
            self._calib.k1, self._calib.d1, r1, p1, (width, height), cv2.CV_32FC1
        )
        self._map1_right, self._map2_right = cv2.initUndistortRectifyMap(
            self._calib.k2, self._calib.d2, r2, p2, (width, height), cv2.CV_32FC1
        )
        self._q = q

    def _gray(self, image: np.ndarray) -> np.ndarray:
        if image.ndim == 2:
            return image
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    def _pair_ready(self) -> bool:
        return (
            self._left_image is not None
            and self._right_image is not None
            and self._left_info is not None
            and self._right_info is not None
        )

    def _on_timer(self) -> None:
        if not self._pair_ready():
            return
        left = self._left_image
        right = self._right_image
        assert left is not None and right is not None

        pair_key = (left.stamp_ns, right.stamp_ns)
        if self._last_processed_pair == pair_key:
            return

        if abs(left.stamp_ns - right.stamp_ns) > self._max_sync_diff_ns:
            self.get_logger().warning("skip pair: left/right stamps exceed max_sync_diff_ms")
            return

        if left.width != right.width or left.height != right.height:
            self.get_logger().warning("skip pair: left/right image size mismatch")
            return

        self._ensure_rectify_maps(left.width, left.height)

        left_rect = cv2.remap(self._gray(left.image), self._map1_left, self._map2_left, cv2.INTER_LINEAR)
        right_rect = cv2.remap(
            self._gray(right.image), self._map1_right, self._map2_right, cv2.INTER_LINEAR
        )

        disparity_raw = self._matcher.compute(left_rect, right_rect)
        disparity = disparity_raw.astype(np.float32) / 16.0
        valid_mask = disparity > float(self.get_parameter("min_disparity").value)

        if self._q is not None:
            points = cv2.reprojectImageTo3D(disparity, self._q)
            depth = points[:, :, 2].astype(np.float32)
        else:
            depth = np.full(disparity.shape, np.nan, dtype=np.float32)
            valid_mask &= disparity > 0.0
            depth[valid_mask] = (
                (self._calib.fx * self._calib.baseline_m) / disparity[valid_mask]
            ).astype(np.float32)

        valid_mask &= np.isfinite(depth)
        valid_mask &= depth >= self._min_depth_m
        valid_mask &= depth <= self._max_depth_m
        depth = depth.astype(np.float32)
        depth[~valid_mask] = np.nan

        stamp_ns = max(left.stamp_ns, right.stamp_ns)
        self._left_rect_pub.publish(make_image_msg(left_rect, stamp_ns, left.frame_id, "mono8"))
        self._right_rect_pub.publish(make_image_msg(right_rect, stamp_ns, right.frame_id, "mono8"))
        self._disparity_pub.publish(
            make_image_msg(disparity.astype(np.float32), stamp_ns, left.frame_id, "32FC1")
        )
        self._depth_pub.publish(make_image_msg(depth, stamp_ns, left.frame_id, "32FC1"))
        self._valid_mask_pub.publish(
            make_image_msg((valid_mask.astype(np.uint8) * 255), stamp_ns, left.frame_id, "mono8")
        )

        self._last_processed_pair = pair_key


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = SgbmBaselineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
