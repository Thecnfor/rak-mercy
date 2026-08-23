"""ROS-topic-only physical checkerboard stereo calibration.

This command intentionally writes captures and candidate results outside the source
tree.  It never synthesizes a physical calibration: a candidate can become
``validated`` only after measured images, quality gates, and explicit operator
confirmations are all present.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from collections import Counter, deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Deque, Optional, Sequence

import cv2
import numpy as np
import yaml

from .charuco_stereo import detect as find_charuco_corners, intersect as intersect_charuco_ids

try:  # Supports `ros2 run` and direct operator diagnostics alike.
    from .stereo_calibration_contract import (
        DEFAULT_BOARD_INNER_CORNERS,
        CALIBRATION_SIZE,
        MAX_PAIR_SKEW_MS,
        MAX_SAMPLES,
        MIN_SAMPLES,
        coverage_cells,
        coverage_complete,
        normalize_board_inner_corners,
        validate_capture_arguments,
        validation_gate,
    )
except ImportError:  # pragma: no cover - only used for direct script execution.
    from stereo_calibration_contract import (  # type: ignore
        DEFAULT_BOARD_INNER_CORNERS,
        CALIBRATION_SIZE,
        MAX_PAIR_SKEW_MS,
        MAX_SAMPLES,
        MIN_SAMPLES,
        coverage_cells,
        coverage_complete,
        normalize_board_inner_corners,
        validate_capture_arguments,
        validation_gate,
    )


LEFT_TOPIC = "/x1/left_camera/image_raw"
RIGHT_TOPIC = "/x1/right_camera/image_raw"
MANIFEST_NAME = "capture_manifest.json"


def stamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def image_to_gray(message: Any) -> np.ndarray:
    """Decode supported ROS Image encodings without requiring cv_bridge."""
    encoding = str(message.encoding).lower()
    width, height = int(message.width), int(message.height)
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if encoding == "mono8":
        return raw.reshape(height, int(message.step))[:, :width].copy()
    if encoding in {"bgr8", "rgb8"}:
        image = raw.reshape(height, int(message.step))[:, : width * 3].reshape(height, width, 3)
        code = cv2.COLOR_BGR2GRAY if encoding == "bgr8" else cv2.COLOR_RGB2GRAY
        return cv2.cvtColor(image, code)
    raise ValueError(f"unsupported_image_encoding:{message.encoding}")


def find_corners(gray: np.ndarray, board_inner_corners: tuple[int, int]) -> Optional[np.ndarray]:
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    found, corners = cv2.findChessboardCornersSB(gray, board_inner_corners, flags)
    if found:
        return corners.reshape(-1, 2).astype(np.float32)

    # Printed or slightly curved boards are sometimes rejected by the stricter
    # SB detector even though the classic detector can still provide accurate,
    # sub-pixel corners.  Keeping this fallback in the shared helper makes live
    # capture and the compute-time recheck use exactly the same detector.
    classic_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, board_inner_corners, classic_flags)
    if not found:
        return None
    refined = cv2.cornerSubPix(
        gray,
        corners,
        (5, 5),
        (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001),
    )
    return refined.reshape(-1, 2).astype(np.float32)


def blur_score(gray: np.ndarray) -> float:
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def board_descriptor(corners: np.ndarray) -> np.ndarray:
    centre = corners.mean(axis=0)
    span = np.maximum(corners.max(axis=0) - corners.min(axis=0), 1.0)
    vector = corners[-1] - corners[0]
    angle = math.atan2(float(vector[1]), float(vector[0]))
    return np.array([centre[0] / 640.0, centre[1] / 360.0, span[0] / 640.0, span[1] / 360.0, angle])


def is_duplicate_pose(candidate: np.ndarray, previous: Sequence[np.ndarray]) -> bool:
    # Similar image centre, scale and orientation means no useful new geometry.
    return any(float(np.linalg.norm(candidate - item)) < 0.035 for item in previous)


def object_points(square_size_m: float, count: int, board_inner_corners: tuple[int, int]) -> list[np.ndarray]:
    grid = np.zeros((board_inner_corners[0] * board_inner_corners[1], 3), np.float32)
    grid[:, :2] = np.mgrid[0 : board_inner_corners[0], 0 : board_inner_corners[1]].T.reshape(-1, 2)
    grid *= float(square_size_m)
    return [grid.copy() for _ in range(count)]


def rectified_epipolar_errors(
    left_points: Sequence[np.ndarray], right_points: Sequence[np.ndarray], k1: np.ndarray, d1: np.ndarray,
    k2: np.ndarray, d2: np.ndarray, r1: np.ndarray, p1: np.ndarray, r2: np.ndarray, p2: np.ndarray,
) -> np.ndarray:
    errors: list[np.ndarray] = []
    for left, right in zip(left_points, right_points):
        left_rect = cv2.undistortPoints(left.reshape(-1, 1, 2), k1, d1, R=r1, P=p1).reshape(-1, 2)
        right_rect = cv2.undistortPoints(right.reshape(-1, 1, 2), k2, d2, R=r2, P=p2).reshape(-1, 2)
        errors.append(np.abs(left_rect[:, 1] - right_rect[:, 1]))
    return np.concatenate(errors) if errors else np.empty((0,), dtype=np.float64)


def solve_stereo(
    left_points: Sequence[np.ndarray], right_points: Sequence[np.ndarray], square_size_m: float,
    board_inner_corners: tuple[int, int],
) -> dict[str, Any]:
    size = CALIBRATION_SIZE
    object_pts = object_points(square_size_m, len(left_points), board_inner_corners)
    calibration_flags = cv2.CALIB_RATIONAL_MODEL
    _, k1, d1, _, _ = cv2.calibrateCamera(object_pts, list(left_points), size, None, None, flags=calibration_flags)
    _, k2, d2, _, _ = cv2.calibrateCamera(object_pts, list(right_points), size, None, None, flags=calibration_flags)
    rms, k1, d1, k2, d2, r, t, _, _ = cv2.stereoCalibrate(
        object_pts, list(left_points), list(right_points), k1, d1, k2, d2, size,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-7),
        flags=calibration_flags | cv2.CALIB_FIX_INTRINSIC,
    )
    r1, r2, p1, p2, q, _, _ = cv2.stereoRectify(k1, d1, k2, d2, size, r, t, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    epipolar = rectified_epipolar_errors(left_points, right_points, k1, d1, k2, d2, r1, p1, r2, p2)
    return {
        "K1": k1, "D1": d1, "K2": k2, "D2": d2, "R": r, "T": t,
        "R1": r1, "R2": r2, "P1": p1, "P2": p2, "Q": q,
        "reproj_rms_px": float(rms),
        "epipolar_p95_px": float(np.percentile(epipolar, 95)),
        "epipolar_samples": int(epipolar.size),
    }


def matrix_list(value: np.ndarray) -> list[Any]:
    return np.asarray(value, dtype=float).tolist()


def require_session_dir(path_text: str) -> Path:
    path = Path(path_text).expanduser().resolve()
    if path.name in {"", "."}:
        raise ValueError("session_dir_must_not_be_a_filesystem_root")
    return path


def source_revision() -> str:
    """Best-effort revision for audit reports; never blocks calibration."""
    for parent in Path(__file__).resolve().parents:
        if (parent / ".git").exists():
            try:
                return subprocess.check_output(
                    ["git", "-C", str(parent), "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL
                ).strip()
            except (OSError, subprocess.CalledProcessError):
                break
    return "unknown"


class TopicCapture:
    def __init__(self, args: argparse.Namespace) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image

        self.args = args
        self.board_inner_corners = normalize_board_inner_corners((args.board_cols, args.board_rows))
        if self.board_inner_corners is None:
            raise ValueError("board_inner_corners_must_be_explicit_integers_at_least_4x4")
        self.rclpy = rclpy
        self.node: Node = rclpy.create_node("physical_stereo_calibration_capture")
        self.left_queue: Deque[Any] = deque(maxlen=8)
        self.right_queue: Deque[Any] = deque(maxlen=8)
        self.descriptors: list[np.ndarray] = []
        self.samples: list[dict[str, Any]] = []
        self.rejects: Counter[str] = Counter()
        self.cells: set[tuple[int, int]] = set()
        self.running = True
        self.session_dir = require_session_dir(args.session_dir)
        self.session_dir.mkdir(parents=True, exist_ok=False)
        (self.session_dir / "left").mkdir()
        (self.session_dir / "right").mkdir()
        self.node.create_subscription(Image, args.left_topic, self._left, qos_profile_sensor_data)
        self.node.create_subscription(Image, args.right_topic, self._right, qos_profile_sensor_data)

    def _left(self, message: Any) -> None:
        self.left_queue.append(message)
        self._pair()

    def _right(self, message: Any) -> None:
        self.right_queue.append(message)
        self._pair()

    def _pair(self) -> None:
        if not self.running or not self.left_queue or not self.right_queue:
            return
        left = self.left_queue[-1]
        right = min(self.right_queue, key=lambda item: abs(stamp_ns(item) - stamp_ns(left)))
        skew_ms = abs(stamp_ns(left) - stamp_ns(right)) / 1_000_000.0
        if skew_ms > MAX_PAIR_SKEW_MS:
            self.rejects["pair_skew_gt_10ms"] += 1
            return
        self.left_queue.clear()
        self.right_queue.remove(right)
        self._evaluate_pair(left, right, skew_ms)

    def _evaluate_pair(self, left_message: Any, right_message: Any, skew_ms: float) -> None:
        if (int(left_message.width), int(left_message.height)) != CALIBRATION_SIZE or (
            int(right_message.width), int(right_message.height)
        ) != CALIBRATION_SIZE:
            self.rejects["resolution_not_640x360"] += 1
            return
        try:
            left_gray, right_gray = image_to_gray(left_message), image_to_gray(right_message)
        except ValueError as error:
            self.rejects[str(error)] += 1
            return
        left_blur, right_blur = blur_score(left_gray), blur_score(right_gray)
        if min(left_blur, right_blur) < self.args.min_blur_score:
            self.rejects["motion_blur"] += 1
            return
        left_corners = find_corners(left_gray, self.board_inner_corners)
        right_corners = find_corners(right_gray, self.board_inner_corners)
        if left_corners is None or right_corners is None:
            self.rejects["checkerboard_not_found"] += 1
            return
        descriptor = board_descriptor(left_corners)
        if is_duplicate_pose(descriptor, self.descriptors):
            self.rejects["duplicate_pose"] += 1
            return
        index = len(self.samples)
        left_file = f"left/{index:03d}.png"
        right_file = f"right/{index:03d}.png"
        if not cv2.imwrite(str(self.session_dir / left_file), left_gray) or not cv2.imwrite(str(self.session_dir / right_file), right_gray):
            self.rejects["image_write_failed"] += 1
            return
        centre = left_corners.mean(axis=0)
        self.cells.update(coverage_cells([centre], *CALIBRATION_SIZE))
        self.descriptors.append(descriptor)
        self.samples.append({
            "index": index, "left": left_file, "right": right_file,
            "left_stamp_ns": stamp_ns(left_message), "right_stamp_ns": stamp_ns(right_message),
            "pair_skew_ms": skew_ms, "left_blur_score": left_blur, "right_blur_score": right_blur,
            "left_board_centre_px": [float(centre[0]), float(centre[1])],
            "board_inner_corners": list(self.board_inner_corners),
        })
        self.node.get_logger().info(
            f"accepted {len(self.samples)}/{self.args.samples}; skew_ms={skew_ms:.3f}; coverage={len(self.cells)}/9"
        )
        if len(self.samples) >= self.args.samples:
            self.running = False

    def manifest(self) -> dict[str, Any]:
        return {
            "schema_version": 2, "source": "ros_topics", "board_inner_corners": list(self.board_inner_corners),
            "square_size_m": self.args.square_size_m, "resolution": list(CALIBRATION_SIZE),
            "left_topic": self.args.left_topic, "right_topic": self.args.right_topic,
            "max_pair_skew_ms": MAX_PAIR_SKEW_MS, "min_blur_score": self.args.min_blur_score,
            "captured_at_utc": datetime.now(timezone.utc).isoformat(), "samples": self.samples,
            "reject_counts": dict(sorted(self.rejects.items())), "coverage_cells": sorted([list(cell) for cell in self.cells]),
            "capture_command": sys.argv, "source_revision": source_revision(),
        }

    def run(self) -> None:
        self.node.get_logger().info(
            f"waiting for ROS images: {self.args.left_topic}, {self.args.right_topic}; Ctrl-C ends capture"
        )
        try:
            while self.rclpy.ok() and self.running:
                self.rclpy.spin_once(self.node, timeout_sec=0.2)
        finally:
            (self.session_dir / MANIFEST_NAME).write_text(json.dumps(self.manifest(), indent=2) + "\n", encoding="utf-8")
            self.node.destroy_node()


def command_capture(args: argparse.Namespace) -> int:
    errors = validate_capture_arguments(
        square_size_m=args.square_size_m, requested_samples=args.samples, width=args.width, height=args.height,
        board_inner_corners=(args.board_cols, args.board_rows),
    )
    if errors:
        raise ValueError(", ".join(errors))
    if args.left_topic != LEFT_TOPIC or args.right_topic != RIGHT_TOPIC:
        raise ValueError("physical_capture_must_use_the_formal_x1_raw_image_topics")
    import rclpy

    rclpy.init()
    capture = TopicCapture(args)
    try:
        capture.run()
    finally:
        rclpy.shutdown()
    print(f"capture manifest: {capture.session_dir / MANIFEST_NAME}")
    return 0 if len(capture.samples) >= MIN_SAMPLES else 2


def load_observations(
    session_dir: Path, manifest: dict[str, Any], board_inner_corners: tuple[int, int],
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    left_points: list[np.ndarray] = []
    right_points: list[np.ndarray] = []
    for sample in manifest["samples"]:
        if tuple(sample.get("board_inner_corners", ())) != board_inner_corners:
            raise ValueError("sample_board_does_not_match_capture_session_board")
        if float(sample["pair_skew_ms"]) > MAX_PAIR_SKEW_MS:
            continue
        left = cv2.imread(str(session_dir / sample["left"]), cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(str(session_dir / sample["right"]), cv2.IMREAD_GRAYSCALE)
        if left is None or right is None:
            continue
        left_corners = find_corners(left, board_inner_corners)
        right_corners = find_corners(right, board_inner_corners)
        if left_corners is not None and right_corners is not None:
            left_points.append(left_corners)
            right_points.append(right_corners)
    return left_points, right_points


def markdown_report(report: dict[str, Any]) -> str:
    gate = report["validation"]
    reasons = gate["reasons"] or ["all_validation_gates_passed"]
    return "\n".join([
        f"# {report['board_inner_corners'][0]}x{report['board_inner_corners'][1]} 物理双目标定报告", "", f"- 候选 ID：`{report['calibration_id']}`",
        f"- 结果：`validated={gate['validated']}`", f"- 分辨率：`{report['resolution'][0]}x{report['resolution'][1]}`",
        f"- 棋盘内角点：`{report['board_inner_corners'][0]}x{report['board_inner_corners'][1]}`",
        f"- 实测格长：`{report['square_size_m']:.6f} m`",
        f"- 有效样本：`{report['valid_sample_count']}`", f"- 重投影 RMS：`{report['reproj_rms_px']:.4f} px`",
        f"- 校正后垂直极线 P95：`{report['epipolar_p95_px']:.4f} px`", "", "## 验证原因", *[f"- {reason}" for reason in reasons], "",
        "## 采集拒绝统计", *[f"- {key}: {value}" for key, value in report["reject_counts"].items()], "",
        "该报告仅在实体棋盘、实测格长和正式 ROS 话题采集同时满足时有效。",
    ]) + "\n"


def command_compute(args: argparse.Namespace) -> int:
    session_dir = require_session_dir(args.session_dir)
    manifest = json.loads((session_dir / MANIFEST_NAME).read_text(encoding="utf-8"))
    if manifest.get("source") != "ros_topics" or manifest.get("left_topic") != LEFT_TOPIC or manifest.get("right_topic") != RIGHT_TOPIC:
        raise ValueError("manifest_is_not_a_formal_ros_topic_capture")
    board_inner_corners = normalize_board_inner_corners((args.board_cols, args.board_rows))
    if board_inner_corners is None:
        raise ValueError("board_inner_corners_must_be_explicit_integers_at_least_4x4")
    if tuple(manifest.get("board_inner_corners", ())) != board_inner_corners:
        raise ValueError("compute_board_does_not_match_capture_session")
    square_size_m = float(manifest.get("square_size_m", 0.0))
    if not math.isclose(square_size_m, float(args.square_size_m), rel_tol=0.0, abs_tol=1e-12):
        raise ValueError("compute_square_size_does_not_match_capture_session")
    preflight = validate_capture_arguments(
        square_size_m=square_size_m, requested_samples=len(manifest.get("samples", [])),
        width=int(manifest["resolution"][0]), height=int(manifest["resolution"][1]),
        board_inner_corners=board_inner_corners,
    )
    if preflight:
        raise ValueError(", ".join(preflight))
    left_points, right_points = load_observations(session_dir, manifest, board_inner_corners)
    if not MIN_SAMPLES <= len(left_points) <= MAX_SAMPLES:
        raise ValueError("rechecked_valid_samples_not_in_40_to_60")
    solved = solve_stereo(left_points, right_points, square_size_m, board_inner_corners)
    cells = {tuple(cell) for cell in manifest.get("coverage_cells", [])}
    gate = validation_gate(
        sample_count=len(left_points), resolution=manifest["resolution"], reproj_rms_px=solved["reproj_rms_px"],
        epipolar_p95_px=solved["epipolar_p95_px"], source="physical_checkerboard",
        left_right_confirmed=args.confirm_left_right, baseline_sign_confirmed=args.confirm_baseline_sign,
        scale_confirmed=args.confirm_scale, coverage_complete=coverage_complete(cells),
        board_inner_corners=board_inner_corners, square_size_m=square_size_m,
    )
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    calibration_id = f"{args.robot_id}-{args.camera_pair_id}-640x360-{timestamp}"
    candidate: dict[str, Any] = {
        "calibration_id": calibration_id, "robot_id": args.robot_id, "camera_pair_id": args.camera_pair_id,
        "img_size": list(CALIBRATION_SIZE), "board_inner_corners": list(board_inner_corners),
        "square_size_m": square_size_m, "reproj_rms_px": solved["reproj_rms_px"],
        "epipolar_p95_px": solved["epipolar_p95_px"], "date": timestamp, "source": "physical_checkerboard",
        "validated": gate.validated, "validation_reasons": list(gate.reasons),
        "operator_confirmations": {"left_right": args.confirm_left_right, "baseline_sign": args.confirm_baseline_sign, "scale": args.confirm_scale},
        "capture_manifest": str(session_dir / MANIFEST_NAME),
    }
    candidate.update({key: matrix_list(solved[key]) for key in ("K1", "D1", "K2", "D2", "R", "T", "P1", "P2", "Q")})
    report = {
        "calibration_id": calibration_id, "source": "physical_checkerboard", "resolution": list(CALIBRATION_SIZE),
        "board_inner_corners": list(board_inner_corners), "square_size_m": square_size_m,
        "valid_sample_count": len(left_points), "captured_sample_count": len(manifest["samples"]),
        "reproj_rms_px": solved["reproj_rms_px"], "epipolar_p95_px": solved["epipolar_p95_px"],
        "epipolar_sample_count": solved["epipolar_samples"], "coverage_cells": sorted([list(cell) for cell in cells]),
        "coverage_complete": coverage_complete(cells), "reject_counts": manifest.get("reject_counts", {}),
        "capture_command": manifest.get("capture_command", []), "compute_command": sys.argv,
        "source_revision": manifest.get("source_revision", "unknown"),
        "validation": {"validated": gate.validated, "reasons": list(gate.reasons)},
    }
    yaml_path = session_dir / "stereo_calib_candidate.yaml"
    json_path = session_dir / "stereo_calib_report.json"
    markdown_path = session_dir / "stereo_calib_report.md"
    yaml_path.write_text(yaml.safe_dump(candidate, sort_keys=False), encoding="utf-8")
    json_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    markdown_path.write_text(markdown_report(report), encoding="utf-8")
    print(f"candidate: {yaml_path}\nreport: {json_path}\nvalidated={gate.validated}")
    return 0 if gate.validated else 3


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(description=__doc__)
    commands = root.add_subparsers(dest="command", required=True)
    capture = commands.add_parser("capture", help="capture 40-60 real ROS topic stereo pairs")
    capture.add_argument("--session-dir", required=True, help="outside-repository output directory")
    capture.add_argument("--square-size-m", required=True, type=float, help="caliper-measured square edge in metres")
    capture.add_argument("--board-cols", type=int, default=DEFAULT_BOARD_INNER_CORNERS[0], help="physical checkerboard inner-corner columns (official board: 9)")
    capture.add_argument("--board-rows", type=int, default=DEFAULT_BOARD_INNER_CORNERS[1], help="physical checkerboard inner-corner rows (official board: 6)")
    capture.add_argument("--samples", type=int, default=50, help="accepted samples; 40..60")
    capture.add_argument("--width", type=int, default=640)
    capture.add_argument("--height", type=int, default=360)
    capture.add_argument("--left-topic", default=LEFT_TOPIC)
    capture.add_argument("--right-topic", default=RIGHT_TOPIC)
    capture.add_argument("--min-blur-score", type=float, default=80.0)
    compute = commands.add_parser("compute", help="solve a captured session and write candidate/report")
    compute.add_argument("--session-dir", required=True)
    compute.add_argument("--robot-id", required=True)
    compute.add_argument("--camera-pair-id", required=True)
    compute.add_argument("--square-size-m", required=True, type=float, help="must exactly match the capture manifest")
    compute.add_argument("--board-cols", type=int, default=DEFAULT_BOARD_INNER_CORNERS[0], help="must match the capture manifest (official board: 9)")
    compute.add_argument("--board-rows", type=int, default=DEFAULT_BOARD_INNER_CORNERS[1], help="must match the capture manifest (official board: 6)")
    compute.add_argument("--confirm-left-right", action="store_true")
    compute.add_argument("--confirm-baseline-sign", action="store_true")
    compute.add_argument("--confirm-scale", action="store_true")
    return root


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parser().parse_args(argv)
    try:
        return command_capture(args) if args.command == "capture" else command_compute(args)
    except (ValueError, FileNotFoundError, json.JSONDecodeError) as error:
        print(f"physical_stereo_calibration: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
