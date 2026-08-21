"""Desktop assistant for formal Mercury X1 physical stereo calibration.

The GUI captures evidence only.  It deliberately delegates solving and the
``validated`` decision to :mod:`physical_stereo_calibration`, so a pleasant
operator workflow cannot weaken the production geometry contract.
"""

from __future__ import annotations

import argparse
from collections import Counter, deque
import json
import math
from pathlib import Path
import threading
import time
from typing import Any, Deque

import cv2
import numpy as np

from .physical_stereo_calibration import (
    LEFT_TOPIC,
    MANIFEST_NAME,
    RIGHT_TOPIC,
    blur_score,
    board_descriptor,
    command_compute,
    find_corners,
    image_to_gray,
    is_duplicate_pose,
    source_revision,
    stamp_ns,
)
from .stereo_calibration_contract import CALIBRATION_SIZE, DEFAULT_BOARD_INNER_CORNERS, MAX_PAIR_SKEW_MS, coverage_cells
from .stereo_calibration_gui_contract import CaptureSettings, StabilityTracker, make_manifest, session_directory


def _corners_with_fallback(gray: np.ndarray, board: tuple[int, int]) -> np.ndarray | None:
    """Keep the formal detector first, with an OpenCV-4.2 friendly fallback."""
    result = find_corners(gray, board)
    if result is not None:
        return result
    found, corners = cv2.findChessboardCorners(gray, board)
    if not found:
        return None
    refined = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01),
    )
    return refined.reshape(-1, 2).astype(np.float32)


def _pair_preview(
    left: np.ndarray, right: np.ndarray, left_corners: np.ndarray | None, right_corners: np.ndarray | None,
    board: tuple[int, int],
) -> np.ndarray:
    def decorate(image: np.ndarray, corners: np.ndarray | None, caption: str) -> np.ndarray:
        colour = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        if corners is not None:
            cv2.drawChessboardCorners(colour, board, corners.reshape(-1, 1, 2), True)
        cv2.putText(colour, caption, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)
        return colour
    return np.hstack((decorate(left, left_corners, "LEFT"), decorate(right, right_corners, "RIGHT")))


class StereoRosWorker:  # QObject inheritance is installed dynamically after Qt import below.
    pass


def _make_worker_class(QtCore: Any, QtGui: Any) -> type:
    class Worker(QtCore.QThread):
        preview_ready = QtCore.pyqtSignal(object)
        status_ready = QtCore.pyqtSignal(dict)
        capture_finished = QtCore.pyqtSignal(str)
        fault = QtCore.pyqtSignal(str)

        def __init__(self) -> None:
            super().__init__()
            self._lock = threading.Lock()
            self._start_request: tuple[CaptureSettings, Path] | None = None
            self._stop_requested = False
            self._shutdown_requested = False
            self._capturing = False
            self._settings: CaptureSettings | None = None
            self._session: Path | None = None
            self._left: Deque[Any] = deque(maxlen=8)
            self._right: Deque[Any] = deque(maxlen=8)
            self._descriptors: list[np.ndarray] = []
            self._samples: list[dict[str, Any]] = []
            self._rejects: Counter[str] = Counter()
            self._coverage: set[tuple[int, int]] = set()
            self._stable: StabilityTracker | None = None

        def begin_capture(self, settings: CaptureSettings, root: Path) -> None:
            with self._lock:
                self._start_request = (settings, root)

        def stop_capture(self) -> None:
            with self._lock:
                self._stop_requested = True

        def request_shutdown(self) -> None:
            with self._lock:
                self._shutdown_requested = True
                self._stop_requested = True

        def _start_if_requested(self) -> None:
            with self._lock:
                request, self._start_request = self._start_request, None
            if request is None:
                return
            if self._capturing:
                self._finish_capture()
            settings, root = request
            root = root.expanduser()
            root.mkdir(parents=True, exist_ok=True)
            candidate = session_directory(root)
            suffix = 1
            while candidate.exists():
                candidate = root / f"{session_directory(root).name}_{suffix:02d}"
                suffix += 1
            candidate.mkdir()
            (candidate / "left").mkdir()
            (candidate / "right").mkdir()
            self._settings, self._session, self._capturing = settings, candidate, True
            self._descriptors, self._samples = [], []
            self._rejects, self._coverage = Counter(), set()
            self._stable = StabilityTracker(settings.stable_hold_s, settings.max_corner_motion_px)
            self.status_ready(self._status("capturing"))

        def _finish_capture(self) -> None:
            if not self._capturing or self._settings is None or self._session is None:
                return
            manifest = make_manifest(
                settings=self._settings, samples=self._samples, reject_counts=dict(self._rejects),
                coverage=self._coverage, left_topic=LEFT_TOPIC, right_topic=RIGHT_TOPIC, revision=source_revision(),
            )
            (self._session / MANIFEST_NAME).write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            finished = str(self._session)
            self._capturing = False
            self.status_ready(self._status("stopped"))
            self.capture_finished.emit(finished)

        def _status(self, state: str, **extra: Any) -> dict[str, Any]:
            return {
                "state": state, "samples": len(self._samples), "target": self._settings.target_samples if self._settings else 0,
                "coverage": sorted(self._coverage), "rejects": dict(self._rejects), "session": str(self._session or ""), **extra,
            }

        def _on_left(self, message: Any) -> None:
            self._left.append(message)
            self._pair()

        def _on_right(self, message: Any) -> None:
            self._right.append(message)
            self._pair()

        def _pair(self) -> None:
            if not self._left or not self._right:
                return
            left = self._left[-1]
            right = min(self._right, key=lambda item: abs(stamp_ns(item) - stamp_ns(left)))
            skew = abs(stamp_ns(left) - stamp_ns(right)) / 1_000_000.0
            self._left.clear()
            self._right.remove(right)
            self._evaluate(left, right, skew)

        def _evaluate(self, left_message: Any, right_message: Any, skew_ms: float) -> None:
            try:
                left, right = image_to_gray(left_message), image_to_gray(right_message)
            except ValueError as error:
                self._rejects[str(error)] += 1
                self.status_ready(self._status("reject", reason=str(error), skew_ms=skew_ms))
                return
            if left.shape[::-1] != CALIBRATION_SIZE or right.shape[::-1] != CALIBRATION_SIZE:
                self._rejects["resolution_not_640x360"] += 1
                self.status_ready(self._status("reject", reason="resolution_not_640x360", skew_ms=skew_ms))
                return
            board = (
                (self._settings.board_cols, self._settings.board_rows)
                if self._settings else DEFAULT_BOARD_INNER_CORNERS
            )
            left_corners, right_corners = _corners_with_fallback(left, board), _corners_with_fallback(right, board)
            self.preview_ready(_pair_preview(left, right, left_corners, right_corners, board))
            if not self._capturing or self._settings is None:
                self.status_ready(self._status("preview", skew_ms=skew_ms, corners=left_corners is not None and right_corners is not None))
                return
            if skew_ms > MAX_PAIR_SKEW_MS:
                self._rejects["pair_skew_gt_10ms"] += 1
                self.status_ready(self._status("reject", reason="pair_skew_gt_10ms", skew_ms=skew_ms))
                return
            if left_corners is None or right_corners is None:
                self._rejects["checkerboard_not_found"] += 1
                self.status_ready(self._status("reject", reason="checkerboard_not_found", skew_ms=skew_ms))
                return
            left_blur, right_blur = blur_score(left), blur_score(right)
            if min(left_blur, right_blur) < self._settings.min_blur_score:
                self._rejects["motion_blur"] += 1
                self.status_ready(self._status("reject", reason="motion_blur", skew_ms=skew_ms, blur=min(left_blur, right_blur)))
                return
            stable, motion, stable_elapsed = self._stable.update(left_corners, time.monotonic()) if self._stable else (False, math.inf, 0.0)
            if not stable:
                self._rejects["board_not_stable"] += 1
                self.status_ready(self._status("waiting_stable", skew_ms=skew_ms, motion_px=motion, stable_s=stable_elapsed, blur=min(left_blur, right_blur)))
                return
            descriptor = board_descriptor(left_corners)
            if is_duplicate_pose(descriptor, self._descriptors):
                self._rejects["duplicate_pose"] += 1
                self.status_ready(self._status("reject", reason="duplicate_pose", skew_ms=skew_ms, motion_px=motion))
                return
            index = len(self._samples)
            left_file, right_file = f"left/{index:03d}.png", f"right/{index:03d}.png"
            if self._session is None or not cv2.imwrite(str(self._session / left_file), left) or not cv2.imwrite(str(self._session / right_file), right):
                self._rejects["image_write_failed"] += 1
                self.status_ready(self._status("reject", reason="image_write_failed"))
                return
            centre = left_corners.mean(axis=0)
            self._coverage.update(coverage_cells([centre], *CALIBRATION_SIZE))
            self._descriptors.append(descriptor)
            self._samples.append({
                "index": index, "left": left_file, "right": right_file,
                "left_stamp_ns": stamp_ns(left_message), "right_stamp_ns": stamp_ns(right_message),
                "pair_skew_ms": skew_ms, "left_blur_score": left_blur, "right_blur_score": right_blur,
                "left_board_centre_px": [float(centre[0]), float(centre[1])], "board_inner_corners": list(board),
            })
            if self._stable:
                self._stable.reset()
            self.status_ready(self._status("accepted", skew_ms=skew_ms, motion_px=motion, blur=min(left_blur, right_blur)))
            if len(self._samples) >= self._settings.target_samples:
                self._finish_capture()

        def run(self) -> None:
            try:
                import rclpy
                from rclpy.qos import qos_profile_sensor_data
                from sensor_msgs.msg import Image
                rclpy.init(args=None)
                node = rclpy.create_node("stereo_calibration_gui")
                node.create_subscription(Image, LEFT_TOPIC, self._on_left, qos_profile_sensor_data)
                node.create_subscription(Image, RIGHT_TOPIC, self._on_right, qos_profile_sensor_data)
                while rclpy.ok():
                    self._start_if_requested()
                    with self._lock:
                        should_stop, should_shutdown = self._stop_requested, self._shutdown_requested
                        self._stop_requested = False
                    if should_stop:
                        self._finish_capture()
                    if should_shutdown:
                        break
                    rclpy.spin_once(node, timeout_sec=0.1)
                self._finish_capture()
                node.destroy_node()
                rclpy.shutdown()
            except Exception as error:  # surface ROS/display environment faults to the operator
                self.fault.emit(f"ROS 标定服务异常：{error}")
    return Worker


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--session-root", default=str(Path.home() / "deyes_calibration_runs"))
    parser.add_argument("--robot-id", default="mercury_x1")
    parser.add_argument("--camera-pair-id", default="imx219_stereo")
    options, qt_args = parser.parse_known_args(argv)
    from PyQt5 import QtCore, QtGui, QtWidgets

    Worker = _make_worker_class(QtCore, QtGui)

    class Window(QtWidgets.QMainWindow):
        def __init__(self) -> None:
            super().__init__()
            self.setWindowTitle("Mercury X1 双目物理标定")
            self.resize(1240, 760)
            self.worker = Worker()
            self.worker.preview_ready.connect(self._preview)
            self.worker.status_ready.connect(self._status)
            self.worker.capture_finished.connect(self._capture_finished)
            self.worker.fault.connect(self._fault)
            self.session = ""
            self._build()
            self.worker.start()

        def _build(self) -> None:
            central = QtWidgets.QWidget(); self.setCentralWidget(central)
            layout = QtWidgets.QHBoxLayout(central)
            self.image = QtWidgets.QLabel("等待左右相机话题…")
            self.image.setAlignment(QtCore.Qt.AlignCenter); self.image.setMinimumSize(820, 460)
            self.image.setStyleSheet("background:#202124;color:#ddd;")
            layout.addWidget(self.image, 3)
            panel = QtWidgets.QWidget(); form = QtWidgets.QFormLayout(panel); layout.addWidget(panel, 1)
            self.cols = QtWidgets.QSpinBox(); self.cols.setRange(4, 30); self.cols.setValue(DEFAULT_BOARD_INNER_CORNERS[0])
            self.rows = QtWidgets.QSpinBox(); self.rows.setRange(4, 30); self.rows.setValue(DEFAULT_BOARD_INNER_CORNERS[1])
            self.square = QtWidgets.QDoubleSpinBox(); self.square.setRange(1.0, 200.0); self.square.setValue(20.0); self.square.setSuffix(" mm")
            self.samples = QtWidgets.QSpinBox(); self.samples.setRange(40, 60); self.samples.setValue(50)
            self.root = QtWidgets.QLineEdit(options.session_root)
            self.robot = QtWidgets.QLineEdit(options.robot_id); self.pair = QtWidgets.QLineEdit(options.camera_pair_id)
            for label, widget in (("棋盘内角点列", self.cols), ("棋盘内角点行", self.rows), ("实测格长", self.square), ("目标样本", self.samples), ("会话保存目录", self.root), ("机器人 ID", self.robot), ("相机对 ID", self.pair)):
                form.addRow(label, widget)
            self.confirm_lr = QtWidgets.QCheckBox("已确认左右相机顺序")
            self.confirm_baseline = QtWidgets.QCheckBox("已确认基线正负方向")
            self.confirm_scale = QtWidgets.QCheckBox("已用卡尺确认格长")
            form.addRow(self.confirm_lr); form.addRow(self.confirm_baseline); form.addRow(self.confirm_scale)
            self.start = QtWidgets.QPushButton("开始新采集"); self.stop = QtWidgets.QPushButton("停止并保存"); self.solve = QtWidgets.QPushButton("求解候选并验证")
            form.addRow(self.start); form.addRow(self.stop); form.addRow(self.solve)
            self.status_text = QtWidgets.QLabel("预览模式：等待图像")
            self.status_text.setWordWrap(True); form.addRow("状态", self.status_text)
            self.coverage = QtWidgets.QLabel("覆盖：0/9"); form.addRow(self.coverage)
            self.path = QtWidgets.QLabel("尚未创建会话"); self.path.setWordWrap(True); form.addRow("会话", self.path)
            self.start.clicked.connect(self._start); self.stop.clicked.connect(self.worker.stop_capture); self.solve.clicked.connect(self._solve)

        def _settings(self) -> CaptureSettings:
            return CaptureSettings(board_cols=self.cols.value(), board_rows=self.rows.value(), square_size_m=self.square.value() / 1000.0, target_samples=self.samples.value())

        def _start(self) -> None:
            settings = self._settings(); errors = settings.errors()
            if errors:
                QtWidgets.QMessageBox.warning(self, "参数无效", "\n".join(errors)); return
            self.session = ""; self.path.setText("正在创建会话…")
            self.worker.begin_capture(settings, Path(self.root.text()).expanduser())

        def _preview(self, image: np.ndarray) -> None:
            rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            qimage = QtGui.QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QtGui.QImage.Format_RGB888).copy()
            self.image.setPixmap(QtGui.QPixmap.fromImage(qimage).scaled(self.image.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

        def _status(self, state: dict[str, Any]) -> None:
            coverage = {tuple(item) for item in state.get("coverage", [])}
            self.coverage.setText(f"覆盖：{len(coverage)}/9  " + "✓" * len(coverage))
            detail = f"{state.get('state')} | 样本 {state.get('samples')}/{state.get('target')} | 偏差 {state.get('skew_ms', 0):.2f} ms"
            if "reason" in state: detail += f" | {state['reason']}"
            if "stable_s" in state: detail += f" | 稳定 {state['stable_s']:.1f}s"
            self.status_text.setText(detail)
            if state.get("session"): self.path.setText(state["session"])

        def _capture_finished(self, session: str) -> None:
            self.session = session; self.path.setText(session)
            QtWidgets.QMessageBox.information(self, "采集已保存", f"已写入正式采集清单：\n{session}\n\n可勾选三项现场确认后进行求解。")

        def _solve(self) -> None:
            if not self.session:
                QtWidgets.QMessageBox.warning(self, "没有会话", "请先完成一次采集。"); return
            if not (self.confirm_lr.isChecked() and self.confirm_baseline.isChecked() and self.confirm_scale.isChecked()):
                QtWidgets.QMessageBox.warning(self, "现场确认缺失", "三个确认项均为正式验证的必要条件。"); return
            args = argparse.Namespace(session_dir=self.session, robot_id=self.robot.text().strip(), camera_pair_id=self.pair.text().strip(), square_size_m=self.square.value() / 1000.0, board_cols=self.cols.value(), board_rows=self.rows.value(), confirm_left_right=True, confirm_baseline_sign=True, confirm_scale=True)
            try:
                result = command_compute(args)
            except (ValueError, FileNotFoundError, json.JSONDecodeError, cv2.error) as error:
                QtWidgets.QMessageBox.critical(self, "求解失败", str(error)); return
            report = Path(self.session) / "stereo_calib_report.md"
            message = "验证通过，可由后续受控部署流程使用。" if result == 0 else "候选已输出，但未通过正式门禁；不得用于抓取。"
            QtWidgets.QMessageBox.information(self, "求解完成", f"{message}\n\n报告：{report}")

        def _fault(self, text: str) -> None:
            self.status_text.setText(text)
            QtWidgets.QMessageBox.critical(self, "标定工具错误", text)

        def closeEvent(self, event: Any) -> None:  # noqa: N802
            self.worker.request_shutdown(); self.worker.wait(2500); event.accept()

    app = QtWidgets.QApplication(["stereo_calibration_gui", *qt_args])
    window = Window(); window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
