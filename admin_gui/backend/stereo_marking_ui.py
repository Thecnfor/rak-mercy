#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import subprocess
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any

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


def stamp_to_sec(msg: Image) -> float:
    stamp = msg.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def fit_image(image: np.ndarray, target_width: int, target_height: int) -> np.ndarray:
    if image.size == 0:
        return np.zeros((target_height, target_width, 3), dtype=np.uint8)

    scale = min(target_width / image.shape[1], target_height / image.shape[0])
    width = max(1, int(round(image.shape[1] * scale)))
    height = max(1, int(round(image.shape[0] * scale)))
    resized = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)

    canvas = np.zeros((target_height, target_width, 3), dtype=np.uint8)
    top = (target_height - height) // 2
    left = (target_width - width) // 2
    canvas[top : top + height, left : left + width] = resized
    return canvas


def next_pair_index(calib_dir: Path) -> int:
    highest = -1
    for path in calib_dir.glob("pair*_left.png"):
        stem = path.name.split("pair", 1)[-1].split("_left", 1)[0]
        if stem.isdigit():
            highest = max(highest, int(stem))
    return highest + 1


class StereoMarkerUI(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("stereo_marker_ui")
        self.left_topic = args.left_topic
        self.right_topic = args.right_topic
        self.calib_dir = Path(args.calib_dir).resolve()
        self.calib_dir.mkdir(parents=True, exist_ok=True)
        self.compute_command = args.compute_command
        self.display_width = args.display_width
        self.display_hz = max(5.0, float(args.display_hz))
        self.sync_tolerance_ms = max(1.0, float(args.sync_tolerance_ms))
        self.max_frame_age_ms = max(10.0, float(args.max_frame_age_ms))
        self.window_name = args.window_name

        self._lock = threading.Lock()
        self.left_image: np.ndarray | None = None
        self.right_image: np.ndarray | None = None
        self.left_stamp = 0.0
        self.right_stamp = 0.0
        self.left_rx = 0.0
        self.right_rx = 0.0
        self.left_events: deque[float] = deque(maxlen=120)
        self.right_events: deque[float] = deque(maxlen=120)
        self.status_text = "Waiting for stereo frames..."
        self.status_ok = False
        self.last_saved_pair: int | None = None
        self.compute_running = False
        self.compute_log_tail = ""
        self.pending_action: str | None = None
        self.button_rects: dict[str, tuple[int, int, int, int]] = {}

        qos = QoSPresetProfiles.SENSOR_DATA.value
        self.left_sub = self.create_subscription(Image, self.left_topic, self.on_left, qos)
        self.right_sub = self.create_subscription(Image, self.right_topic, self.on_right, qos)

    def on_left(self, msg: Image) -> None:
        now = time.monotonic()
        image = image_to_bgr(msg)
        with self._lock:
            self.left_image = image
            self.left_stamp = stamp_to_sec(msg)
            self.left_rx = now
            self.left_events.append(now)

    def on_right(self, msg: Image) -> None:
        now = time.monotonic()
        image = image_to_bgr(msg)
        with self._lock:
            self.right_image = image
            self.right_stamp = stamp_to_sec(msg)
            self.right_rx = now
            self.right_events.append(now)

    def fps_from_events(self, events: deque[float]) -> float:
        if len(events) < 2:
            return 0.0
        span = events[-1] - events[0]
        if span <= 1e-6:
            return 0.0
        return (len(events) - 1) / span

    def latest_state(self) -> dict[str, Any]:
        with self._lock:
            left = None if self.left_image is None else self.left_image.copy()
            right = None if self.right_image is None else self.right_image.copy()
            left_fps = self.fps_from_events(self.left_events)
            right_fps = self.fps_from_events(self.right_events)
            return {
                "left_image": left,
                "right_image": right,
                "left_stamp": self.left_stamp,
                "right_stamp": self.right_stamp,
                "left_rx": self.left_rx,
                "right_rx": self.right_rx,
                "left_fps": left_fps,
                "right_fps": right_fps,
            }

    def save_pair(self) -> tuple[bool, str]:
        state = self.latest_state()
        left = state["left_image"]
        right = state["right_image"]
        if left is None or right is None:
            return False, "Cannot save pair: left/right frame not ready"

        age_ms = max(
            (time.monotonic() - state["left_rx"]) * 1000.0,
            (time.monotonic() - state["right_rx"]) * 1000.0,
        )
        skew_ms = abs(state["left_stamp"] - state["right_stamp"]) * 1000.0
        if age_ms > self.max_frame_age_ms:
            return False, f"Frame too old: {age_ms:.1f} ms > {self.max_frame_age_ms:.1f} ms"
        if skew_ms > self.sync_tolerance_ms:
            return False, f"Pair skew too high: {skew_ms:.1f} ms > {self.sync_tolerance_ms:.1f} ms"

        pair_index = next_pair_index(self.calib_dir)
        left_path = self.calib_dir / f"pair{pair_index}_left.png"
        right_path = self.calib_dir / f"pair{pair_index}_right.png"
        if not cv2.imwrite(str(left_path), left):
            return False, f"Failed to write {left_path}"
        if not cv2.imwrite(str(right_path), right):
            return False, f"Failed to write {right_path}"

        self.last_saved_pair = pair_index
        return True, f"Saved pair{pair_index} (skew={skew_ms:.1f} ms)"

    def start_compute(self) -> tuple[bool, str]:
        if self.compute_running:
            return False, "Compute already running"

        def worker() -> None:
            self.compute_running = True
            try:
                result = subprocess.run(
                    self.compute_command,
                    shell=True,
                    capture_output=True,
                    text=True,
                    env=os.environ.copy(),
                )
                tail = (result.stdout or "") + ("\n" + result.stderr if result.stderr else "")
                lines = [line for line in tail.splitlines() if line.strip()]
                self.compute_log_tail = "\n".join(lines[-12:])
                if result.returncode == 0:
                    self.status_text = "Compute finished successfully"
                    self.status_ok = True
                else:
                    self.status_text = f"Compute failed (code={result.returncode})"
                    self.status_ok = False
            except Exception as exc:
                self.compute_log_tail = str(exc)
                self.status_text = f"Compute error: {exc}"
                self.status_ok = False
            finally:
                self.compute_running = False

        threading.Thread(target=worker, daemon=True, name="stereo-compute-worker").start()
        return True, "Compute started"

    def set_status(self, ok: bool, message: str) -> None:
        self.status_ok = ok
        self.status_text = message

    def on_mouse(self, event: int, x: int, y: int, _flags: int, _userdata: Any) -> None:
        if event != cv2.EVENT_LBUTTONUP:
            return
        for action, (x0, y0, x1, y1) in self.button_rects.items():
            if x0 <= x <= x1 and y0 <= y <= y1:
                self.pending_action = action
                return

    def pair_count(self) -> int:
        return len(list(self.calib_dir.glob("pair*_left.png")))

    def build_canvas(self) -> np.ndarray:
        state = self.latest_state()
        left = state["left_image"]
        right = state["right_image"]
        left_fps = state["left_fps"]
        right_fps = state["right_fps"]
        skew_ms = abs(state["left_stamp"] - state["right_stamp"]) * 1000.0
        age_ms = max(
            (time.monotonic() - state["left_rx"]) * 1000.0 if state["left_rx"] else 1e9,
            (time.monotonic() - state["right_rx"]) * 1000.0 if state["right_rx"] else 1e9,
        )
        fps_threshold = max(1.0, self.display_hz - 1.0)
        rate_ok = left_fps >= fps_threshold and right_fps >= fps_threshold
        sync_ok = (
            left is not None
            and right is not None
            and skew_ms <= self.sync_tolerance_ms
            and age_ms <= self.max_frame_age_ms
        )

        total_width = self.display_width
        pad = 18
        panel_width = (total_width - pad * 3) // 2
        panel_height = int(round(panel_width * 9 / 16))
        header_height = 136
        footer_height = 168
        total_height = header_height + panel_height + footer_height + pad * 3

        canvas = np.zeros((total_height, total_width, 3), dtype=np.uint8)
        canvas[:] = (11, 16, 27)
        cv2.rectangle(canvas, (0, 0), (total_width, 88), (18, 29, 52), thickness=-1)
        cv2.putText(
            canvas,
            "Stereo Mainline Marker",
            (28, 46),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.05,
            (236, 245, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            "Local ROS/C++ live view. Save current synchronized pair as pairN_left/right.png",
            (28, 76),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (146, 164, 188),
            1,
            cv2.LINE_AA,
        )

        left_panel = fit_image(left if left is not None else np.zeros((480, 640, 3), dtype=np.uint8), panel_width, panel_height)
        right_panel = fit_image(right if right is not None else np.zeros((480, 640, 3), dtype=np.uint8), panel_width, panel_height)
        top = header_height
        left_x = pad
        right_x = pad * 2 + panel_width
        canvas[top : top + panel_height, left_x : left_x + panel_width] = left_panel
        canvas[top : top + panel_height, right_x : right_x + panel_width] = right_panel
        cv2.rectangle(canvas, (left_x, top), (left_x + panel_width, top + panel_height), (58, 82, 116), 1)
        cv2.rectangle(canvas, (right_x, top), (right_x + panel_width, top + panel_height), (58, 82, 116), 1)

        cv2.putText(canvas, "LEFT", (left_x + 14, top + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (111, 229, 255), 2, cv2.LINE_AA)
        cv2.putText(canvas, "RIGHT", (right_x + 14, top + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (137, 255, 181), 2, cv2.LINE_AA)

        cv2.putText(
            canvas,
            f"left_rx={left_fps:5.2f} Hz",
            (left_x + 14, top + panel_height - 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.64,
            (231, 240, 252),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"right_rx={right_fps:5.2f} Hz",
            (right_x + 14, top + panel_height - 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.64,
            (231, 240, 252),
            2,
            cv2.LINE_AA,
        )

        info_y = header_height - 18
        if sync_ok and rate_ok:
            sync_color = (110, 230, 158)
            status = "30HZ READY"
        elif sync_ok:
            sync_color = (255, 211, 111)
            status = "SYNC OK / FPS LOW"
        else:
            sync_color = (87, 169, 255)
            status = "WAIT SYNC"
        cv2.putText(
            canvas,
            f"{status}  skew={skew_ms:5.1f} ms  age={age_ms:5.1f} ms  target={self.display_hz:.0f} Hz",
            (28, info_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            sync_color,
            2,
            cv2.LINE_AA,
        )

        footer_top = header_height + panel_height + pad
        cv2.rectangle(canvas, (pad, footer_top), (total_width - pad, total_height - pad), (18, 29, 52), thickness=-1)
        cv2.rectangle(canvas, (pad, footer_top), (total_width - pad, total_height - pad), (48, 70, 102), thickness=1)

        status_color = (110, 230, 158) if self.status_ok else (128, 147, 177)
        cv2.putText(canvas, f"STATUS: {self.status_text}", (pad + 16, footer_top + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.68, status_color, 2, cv2.LINE_AA)
        cv2.putText(
            canvas,
            f"pairs={self.pair_count()}  last_saved={self.last_saved_pair if self.last_saved_pair is not None else '-'}  save_rule=skew<={self.sync_tolerance_ms:.0f}ms",
            (pad + 16, footer_top + 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (220, 229, 242),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"compute={'RUNNING' if self.compute_running else 'idle'}  calib_dir={self.calib_dir}",
            (pad + 16, footer_top + 92),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.56,
            (161, 175, 198),
            1,
            cv2.LINE_AA,
        )

        button_y0 = footer_top + 112
        button_y1 = button_y0 + 38
        buttons = [
            ("save", "Save Pair [S/SPACE]", (255, 211, 111)),
            ("compute", "Compute [C]", (111, 229, 255)),
            ("quit", "Quit [Q/ESC]", (255, 143, 143)),
        ]
        self.button_rects = {}
        button_x = pad + 16
        for key, label, color in buttons:
            width = 248 if key == "save" else 198
            x0 = button_x
            x1 = x0 + width
            cv2.rectangle(canvas, (x0, button_y0), (x1, button_y1), color, thickness=-1)
            cv2.rectangle(canvas, (x0, button_y0), (x1, button_y1), (20, 27, 41), thickness=1)
            cv2.putText(canvas, label, (x0 + 14, button_y0 + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (21, 24, 30), 2, cv2.LINE_AA)
            self.button_rects[key] = (x0, button_y0, x1, button_y1)
            button_x = x1 + 14

        if self.compute_log_tail:
            log_lines = self.compute_log_tail.splitlines()[-2:]
            line_y = footer_top + 124
            for line in log_lines:
                clipped = line[:96]
                cv2.putText(canvas, clipped, (pad + 690, line_y), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (174, 191, 210), 1, cv2.LINE_AA)
                line_y += 22

        return canvas


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Local stereo UI for 30Hz ROS/C++ live view and paired captures.")
    parser.add_argument("--left-topic", default=os.environ.get("DEYES_LEFT_IMAGE_TOPIC", "/x1/left_camera/image_raw"))
    parser.add_argument("--right-topic", default=os.environ.get("DEYES_RIGHT_IMAGE_TOPIC", "/x1/right_camera/image_raw"))
    parser.add_argument("--calib-dir", default=os.environ.get("DEYES_CALIB_DIR", "/home/elephant/mercury_grasp/data/calib"))
    parser.add_argument("--display-width", type=int, default=1600)
    parser.add_argument("--display-hz", type=float, default=30.0)
    parser.add_argument("--sync-tolerance-ms", type=float, default=30.0)
    parser.add_argument("--max-frame-age-ms", type=float, default=200.0)
    parser.add_argument("--window-name", default="Stereo Mainline Marker")
    parser.add_argument(
        "--compute-command",
        default="python3 /home/elephant/mercury_grasp/calibrate_stereo.py compute",
        help="Command started by the Compute button.",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    rclpy.init()
    node = StereoMarkerUI(args)
    cv2.namedWindow(node.window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(node.window_name, args.display_width, int(round(args.display_width * 0.72)))
    cv2.setMouseCallback(node.window_name, node.on_mouse)

    frame_period = 1.0 / node.display_hz
    next_draw = time.monotonic()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)
            now = time.monotonic()
            if now >= next_draw:
                canvas = node.build_canvas()
                cv2.imshow(node.window_name, canvas)
                next_draw = now + frame_period

            if node.pending_action:
                action = node.pending_action
                node.pending_action = None
                if action == "save":
                    ok, message = node.save_pair()
                    node.set_status(ok, message)
                elif action == "compute":
                    ok, message = node.start_compute()
                    node.set_status(ok, message)
                elif action == "quit":
                    break

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            if key in (32, ord("s"), ord("S")):
                ok, message = node.save_pair()
                node.set_status(ok, message)
            elif key in (ord("c"), ord("C")):
                ok, message = node.start_compute()
                node.set_status(ok, message)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
