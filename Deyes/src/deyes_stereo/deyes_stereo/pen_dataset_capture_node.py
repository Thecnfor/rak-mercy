#!/usr/bin/env python3
"""ROS 2 node for collecting reviewable office-pen training images.

It records a JPEG and the original ROS header metadata, but never performs an
automatic quality or duplicate deletion.  Collection cadence is the sole
selection policy and is recorded in the session manifest.
"""

from __future__ import annotations

from pathlib import Path
import time
from typing import Any, Sequence

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .dataset_capture import (
    DatasetCaptureError,
    DatasetSession,
    FrameEvidence,
    require_external_absolute_dir,
    utc_now,
    validate_capture_options,
)


def repository_root() -> Path:
    # .../Deyes/src/deyes_stereo/deyes_stereo/pen_dataset_capture_node.py
    return Path(__file__).resolve().parents[4]


def image_msg_to_bgr(message: Image) -> np.ndarray:
    """Decode common ROS 8-bit image encodings without a cv_bridge dependency."""
    width, height, step = int(message.width), int(message.height), int(message.step)
    if width <= 0 or height <= 0 or step <= 0:
        raise ValueError("invalid_image_dimensions_or_step")
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if raw.size < height * step:
        raise ValueError("image_data_shorter_than_height_times_step")
    encoding = str(message.encoding).lower()
    if encoding == "mono8":
        if step < width:
            raise ValueError("mono8_step_smaller_than_width")
        gray = raw[: height * step].reshape(height, step)[:, :width]
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if encoding in {"bgr8", "rgb8"}:
        if step < width * 3:
            raise ValueError("color_step_smaller_than_width_times_3")
        pixels = raw[: height * step].reshape(height, step)[:, : width * 3].reshape(height, width, 3)
        return pixels.copy() if encoding == "bgr8" else cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    if encoding in {"bgra8", "rgba8"}:
        if step < width * 4:
            raise ValueError("alpha_step_smaller_than_width_times_4")
        pixels = raw[: height * step].reshape(height, step)[:, : width * 4].reshape(height, width, 4)
        code = cv2.COLOR_BGRA2BGR if encoding == "bgra8" else cv2.COLOR_RGBA2BGR
        return cv2.cvtColor(pixels, code)
    raise ValueError(f"unsupported_image_encoding:{message.encoding}")


class PenDatasetCapture(Node):
    def __init__(self) -> None:
        super().__init__("pen_dataset_capture")
        self.declare_parameter("image_topic", "/x1/stereo/debug/left_rect")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("min_interval_sec", 0.5)
        self.declare_parameter("max_images", 0)
        self.declare_parameter("jpeg_quality", 95)

        self._topic = str(self.get_parameter("image_topic").value)
        output_dir = str(self.get_parameter("output_dir").value)
        self._min_interval_sec = float(self.get_parameter("min_interval_sec").value)
        self._max_images = int(self.get_parameter("max_images").value)
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        validate_capture_options(
            min_interval_sec=self._min_interval_sec,
            max_images=self._max_images,
            jpeg_quality=self._jpeg_quality,
        )
        root = require_external_absolute_dir(output_dir, repository_root())
        self._session = DatasetSession(
            root,
            configuration={
                "image_topic": self._topic,
                "min_interval_sec": self._min_interval_sec,
                "max_images": self._max_images,
                "jpeg_quality": self._jpeg_quality,
                "quality_policy": "report_only_no_automatic_filtering_or_deletion",
            },
        )
        self._last_saved_monotonic: float | None = None
        self._previous_saved_gray: np.ndarray | None = None
        self.finished = False
        self._end_reason = "shutdown"
        self.create_subscription(Image, self._topic, self._on_image, qos_profile_sensor_data)
        self.get_logger().info(
            f"collecting {self._topic}; session={self._session.path}; "
            f"minimum interval={self._min_interval_sec:.3f}s, max_images={self._max_images or 'unlimited'}"
        )

    def _on_image(self, message: Image) -> None:
        self._session.received_messages += 1
        now = time.monotonic()
        if self._last_saved_monotonic is not None and now - self._last_saved_monotonic < self._min_interval_sec:
            self._session.skipped_interval += 1
            return
        try:
            bgr = image_msg_to_bgr(message)
        except ValueError as exc:
            self._session.skipped_decode_error += 1
            self.get_logger().warning(f"image not saved: {exc}")
            return

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        difference = None
        if self._previous_saved_gray is not None and self._previous_saved_gray.shape == gray.shape:
            difference = float(cv2.absdiff(gray, self._previous_saved_gray).mean())
        evidence = FrameEvidence(
            stamp_sec=int(message.header.stamp.sec),
            stamp_nanosec=int(message.header.stamp.nanosec),
            frame_id=str(message.header.frame_id),
            width=int(message.width),
            height=int(message.height),
            encoding=str(message.encoding),
            received_at_utc=utc_now(),
            sharpness_laplacian_variance=sharpness,
            mean_abs_difference_from_previous_saved=difference,
        )
        destination = self._session.next_image_path(evidence)
        temporary = destination.with_name(f".{destination.stem}.part.jpg")
        if not cv2.imwrite(str(temporary), bgr, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]):
            self._session.skipped_decode_error += 1
            self.get_logger().error(f"JPEG encoder failed for {destination}")
            return
        temporary.replace(destination)
        self._session.record_saved(destination, evidence)
        self._last_saved_monotonic = now
        self._previous_saved_gray = gray
        self.get_logger().info(
            f"saved {self._session.saved_images}: {destination.name} "
            f"sharpness={sharpness:.1f}" + (f" delta={difference:.2f}" if difference is not None else "")
        )
        if self._max_images and self._session.saved_images >= self._max_images:
            self.finished = True
            self._end_reason = "max_images_reached"
            self.get_logger().info("max_images reached; finalizing session")

    def close_session(self, reason: str | None = None) -> None:
        self._session.close(reason=reason or self._end_reason)
        self.get_logger().info(f"session summary: {self._session.summary_path}")


def main(argv: Sequence[str] | None = None) -> int:
    rclpy.init(args=argv)
    node: PenDatasetCapture | None = None
    reason = "shutdown"
    try:
        node = PenDatasetCapture()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.25)
        if node.finished:
            reason = "max_images_reached"
    except DatasetCaptureError as exc:
        if node is not None:
            node.get_logger().fatal(str(exc))
        else:
            print(f"pen_dataset_capture configuration error: {exc}")
        return 2
    except KeyboardInterrupt:
        reason = "operator_ctrl_c"
    finally:
        if node is not None:
            node.close_session(reason)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
