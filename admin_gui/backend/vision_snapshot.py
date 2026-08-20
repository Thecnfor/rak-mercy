#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image


def image_to_bgr(msg: Image) -> np.ndarray:
    array = np.frombuffer(msg.data, dtype=np.uint8)

    if msg.encoding in ("mono8", "8UC1"):
        image = array.reshape((msg.height, msg.width))
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if msg.encoding in ("rgb8", "8UC3"):
        image = array.reshape((msg.height, msg.width, 3))
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if msg.encoding == "bgr8":
        return array.reshape((msg.height, msg.width, 3))
    if msg.encoding == "rgba8":
        image = array.reshape((msg.height, msg.width, 4))
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if msg.encoding == "bgra8":
        image = array.reshape((msg.height, msg.width, 4))
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

    raise RuntimeError(f"unsupported encoding: {msg.encoding}")


class OneShotImageSaver(Node):
    def __init__(
        self,
        topic: str,
        output: Path,
        max_width: int = 0,
        jpeg_quality: int = 85,
    ) -> None:
        super().__init__("admin_gui_snapshot")
        self.output = output
        self.max_width = max_width
        self.jpeg_quality = jpeg_quality
        self.received = False
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.on_image,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

    def on_image(self, msg: Image) -> None:
        image = image_to_bgr(msg)
        if self.max_width > 0 and image.shape[1] > self.max_width:
            scale = self.max_width / float(image.shape[1])
            resized = (
                self.max_width,
                max(1, int(round(image.shape[0] * scale))),
            )
            image = cv2.resize(image, resized, interpolation=cv2.INTER_AREA)
        self.output.parent.mkdir(parents=True, exist_ok=True)
        params = [cv2.IMWRITE_JPEG_QUALITY, max(25, min(self.jpeg_quality, 95))]
        if not cv2.imwrite(str(self.output), image, params):
            raise RuntimeError(f"failed to write snapshot to {self.output}")
        self.received = True


def main() -> int:
    parser = argparse.ArgumentParser(description="Capture one ROS image message to JPEG.")
    parser.add_argument("--topic", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout", type=float, default=4.0)
    parser.add_argument("--max-width", type=int, default=0)
    parser.add_argument("--jpeg-quality", type=int, default=85)
    args = parser.parse_args()

    output = Path(args.output).resolve()
    rclpy.init()
    node = OneShotImageSaver(
        args.topic,
        output,
        max_width=args.max_width,
        jpeg_quality=args.jpeg_quality,
    )
    deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)

    try:
        while rclpy.ok() and not node.received:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.get_clock().now().nanoseconds >= deadline:
                raise TimeoutError(f"timeout waiting for image on {args.topic}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
