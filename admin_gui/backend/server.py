#!/usr/bin/env python3
from __future__ import annotations

import argparse
import cv2
import html
import json
import mimetypes
import os
import platform
import re
import shlex
import socket
import subprocess
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse


APP_ROOT = Path(__file__).resolve().parent
RUNTIME_ROOT = APP_ROOT / "runtime"
RUNTIME_ROOT.mkdir(parents=True, exist_ok=True)
FRONTEND_DIST = APP_ROOT.parent / "dist"

FORM_PATH = RUNTIME_ROOT / "calibration_form.json"
WIZARD_PATH = RUNTIME_ROOT / "wizard_state.json"
TASK_HISTORY_PATH = RUNTIME_ROOT / "task_history.json"
VISION_MODE_PATH = RUNTIME_ROOT / "vision_mode.json"
DEPTH_COORDINATE_TUNING_PATH = RUNTIME_ROOT / "depth_coordinate_tuning.json"
VISION_CACHE = {
    "left": RUNTIME_ROOT / "vision_left.jpg",
    "right": RUNTIME_ROOT / "vision_right.jpg",
}
DEPTH_HEATMAP_CACHE = RUNTIME_ROOT / "depth_heatmap.json"


def utc_now() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def shell_command(command: str) -> list[str]:
    if os.name == "nt":
        return ["powershell", "-NoProfile", "-Command", command]
    return ["/bin/bash", "-lc", command]


def clone_json(data: dict[str, Any]) -> dict[str, Any]:
    return json.loads(json.dumps(data))


def read_json_file(path: Path, default: dict[str, Any]) -> dict[str, Any]:
    if not path.exists():
        return clone_json(default)
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return clone_json(default)


def write_json_file(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def iso_to_epoch(value: str | None) -> float:
    if not value:
        return 0.0
    try:
        return datetime.fromisoformat(value).timestamp()
    except ValueError:
        return 0.0


def float_or_none(value: Any) -> float | None:
    try:
        if value in ("", None):
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


@dataclass
class ManagedTask:
    task_id: str
    label: str
    command: str
    log_path: str
    started_at: str
    pid: int | None
    running: bool
    return_code: int | None = None
    finished_at: str | None = None
    recorded: bool = False


class TaskManager:
    def __init__(self, runtime_root: Path, history_path: Path) -> None:
        self.runtime_root = runtime_root
        self.history_path = history_path
        self._lock = threading.Lock()
        self._process: subprocess.Popen[str] | None = None
        self._task: ManagedTask | None = None

    def _history(self) -> list[dict[str, Any]]:
        data = read_json_file(self.history_path, {"tasks": []})
        return list(data.get("tasks", []))

    def _append_history(self, task: ManagedTask) -> None:
        history = self._history()
        history.append(asdict(task))
        history = history[-24:]
        write_json_file(self.history_path, {"tasks": history})

    def _finalize_if_done(self) -> None:
        if self._process is None or self._task is None:
            return
        if self._process.poll() is None:
            return
        self._task.running = False
        self._task.return_code = self._process.returncode
        self._task.finished_at = utc_now()
        if not self._task.recorded:
            self._append_history(self._task)
            self._task.recorded = True
        self._process = None

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            self._finalize_if_done()
            return asdict(self._task) if self._task else {}

    def history(self) -> list[dict[str, Any]]:
        with self._lock:
            self._finalize_if_done()
            return self._history()

    def start(self, task_id: str, label: str, command: str) -> dict[str, Any]:
        with self._lock:
            self._finalize_if_done()
            if self._process is not None and self._process.poll() is None:
                raise RuntimeError("已有任务正在运行，请先停止当前任务。")

            log_name = f"{int(time.time())}_{task_id}.log"
            log_path = self.runtime_root / log_name
            log_file = log_path.open("w", encoding="utf-8")
            process = subprocess.Popen(
                shell_command(command),
                stdout=log_file,
                stderr=subprocess.STDOUT,
                cwd=str(self.runtime_root),
                text=True,
                env=os.environ.copy(),
            )
            self._process = process
            self._task = ManagedTask(
                task_id=task_id,
                label=label,
                command=command,
                log_path=str(log_path),
                started_at=utc_now(),
                pid=process.pid,
                running=True,
            )
            return asdict(self._task)

    def stop(self) -> dict[str, Any]:
        with self._lock:
            if self._process is None or self._task is None or self._process.poll() is not None:
                self._finalize_if_done()
                return {"stopped": False, "message": "当前没有正在运行的任务。"}
            self._process.terminate()
            try:
                self._process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=5)
            self._task.running = False
            self._task.return_code = self._process.returncode
            self._task.finished_at = utc_now()
            if not self._task.recorded:
                self._append_history(self._task)
                self._task.recorded = True
            self._process = None
            return {"stopped": True, "task": asdict(self._task)}


class AdminContext:
    def __init__(self) -> None:
        self.repo_root = Path(os.environ.get("ADMIN_GUI_REPO_ROOT", APP_ROOT.parent.parent)).resolve()
        self.deyes_root = Path(os.environ.get("DEYES_REPO_ROOT", self.repo_root / "Deyes")).resolve()
        self.workspace_root = Path(os.environ.get("DEYES_WORKSPACE", "/home/elephant/deyes_ws"))
        self.mercury_root = Path(os.environ.get("DEYES_MERCURY_ROOT", "/home/elephant/mercury_grasp"))
        self.runtime_root = Path(os.environ.get("ADMIN_GUI_RUNTIME", RUNTIME_ROOT))
        self.runtime_root.mkdir(parents=True, exist_ok=True)
        self.left_image_topic = os.environ.get("DEYES_LEFT_IMAGE_TOPIC", "/x1/left_camera/image_raw")
        self.right_image_topic = os.environ.get("DEYES_RIGHT_IMAGE_TOPIC", "/x1/right_camera/image_raw")
        self.depth_heatmap_topic = os.environ.get("DEYES_DEPTH_HEATMAP_TOPIC", "/x1/stereo/base_heatmap")
        self.depth_heatmap_status_topic = os.environ.get(
            "DEYES_DEPTH_HEATMAP_STATUS_TOPIC", "/x1/stereo/base_heatmap_status"
        )
        self.task_manager = TaskManager(self.runtime_root, TASK_HISTORY_PATH)
        self._vision_state_lock = threading.Lock()
        self._vision_capture_lock = threading.Lock()
        self._depth_heatmap_lock = threading.Lock()
        self._vision_state: dict[str, dict[str, Any]] = {
            "left": {
                "refreshing": False,
                "last_error": "",
                "last_ok_at": "",
                "last_attempt_at": "",
            },
            "right": {
                "refreshing": False,
                "last_error": "",
                "last_ok_at": "",
                "last_attempt_at": "",
            },
        }
        self._depth_heatmap_state: dict[str, Any] = {
            "refreshing": False,
            "last_error": "",
            "last_ok_at": "",
            "last_attempt_at": "",
        }

    @property
    def calib_tool(self) -> Path:
        return self.mercury_root / "calibrate_stereo.py"

    @property
    def calib_dir(self) -> Path:
        return self.mercury_root / "data" / "calib"

    @property
    def placeholder_calib(self) -> Path:
        return self.mercury_root / "config" / "stereo_calib.yaml"

    @property
    def repo_calib_dir(self) -> Path:
        return self.deyes_root / "config" / "camera"

    @property
    def vision_helper(self) -> Path:
        return APP_ROOT / "vision_snapshot.py"

    @property
    def depth_heatmap_helper(self) -> Path:
        return APP_ROOT / "depth_heatmap_snapshot.py"

    def ros_shell_prefix(self) -> str:
        ros_setup = "source /opt/ros/galactic/setup.bash"
        local_setup = (
            f"if [ -f {shlex.quote(str(self.workspace_root / 'install' / 'setup.bash'))} ]; "
            f"then source {shlex.quote(str(self.workspace_root / 'install' / 'setup.bash'))}; fi"
        )
        workspace_cd = f"cd {shlex.quote(str(self.workspace_root))}"
        return f"{ros_setup}; {local_setup}; {workspace_cd}; "

    def default_form(self) -> dict[str, Any]:
        return {
            "board_id": "checkerboard_9x6_board01",
            "square_size_mm": "",
            "inner_corners": "9 x 6",
            "print_scale": "100%",
        }

    def load_form(self) -> dict[str, Any]:
        return read_json_file(FORM_PATH, self.default_form())

    def save_form(self, payload: dict[str, Any]) -> dict[str, Any]:
        form = self.load_form()
        form["board_id"] = str(payload.get("board_id", form["board_id"])).strip()
        form["square_size_mm"] = str(payload.get("square_size_mm", form["square_size_mm"])).strip()
        write_json_file(FORM_PATH, form)
        return form

    def default_wizard_state(self) -> dict[str, Any]:
        return {
            "current_step": "precheck",
            "review_passed": None,
            "review_note": "",
        }

    def load_wizard_state(self) -> dict[str, Any]:
        return read_json_file(WIZARD_PATH, self.default_wizard_state())

    def save_wizard_state(self, payload: dict[str, Any]) -> dict[str, Any]:
        state = self.load_wizard_state()
        if "current_step" in payload:
            state["current_step"] = str(payload["current_step"])
        if "review_passed" in payload:
            state["review_passed"] = payload["review_passed"]
        if "review_note" in payload:
            state["review_note"] = str(payload["review_note"])
        write_json_file(WIZARD_PATH, state)
        return state

    def default_vision_mode(self) -> dict[str, Any]:
        return {
            "mode": "snapshot",
            "stream_available": True,
            "refresh_ms": 2000,
        }

    def load_vision_mode(self) -> dict[str, Any]:
        return read_json_file(VISION_MODE_PATH, self.default_vision_mode())

    def save_vision_mode(self, payload: dict[str, Any]) -> dict[str, Any]:
        state = self.load_vision_mode()
        mode = str(payload.get("mode", state["mode"]))
        if mode not in {"snapshot", "stream"}:
            raise ValueError("视觉模式只允许 snapshot 或 stream")
        state["mode"] = mode
        write_json_file(VISION_MODE_PATH, state)
        return state

    def default_depth_coordinate_tuning(self) -> dict[str, Any]:
        return {
            "use_tf_transform": True,
            "use_manual_transform_fallback": True,
            "manual_translation_m": [0.0, 0.0, 0.0],
            "manual_rpy_deg": [0.0, 0.0, 0.0],
            "source_frame_override": "",
            "sample_step": 2,
            "max_points": 4000,
        }

    def load_depth_coordinate_tuning(self) -> dict[str, Any]:
        state = read_json_file(DEPTH_COORDINATE_TUNING_PATH, self.default_depth_coordinate_tuning())
        state["use_tf_transform"] = bool(state.get("use_tf_transform", True))
        state["use_manual_transform_fallback"] = bool(
            state.get("use_manual_transform_fallback", True)
        )
        state["manual_translation_m"] = [
            float(v) for v in list(state.get("manual_translation_m", [0.0, 0.0, 0.0]))[:3]
        ]
        while len(state["manual_translation_m"]) < 3:
            state["manual_translation_m"].append(0.0)
        state["manual_rpy_deg"] = [
            float(v) for v in list(state.get("manual_rpy_deg", [0.0, 0.0, 0.0]))[:3]
        ]
        while len(state["manual_rpy_deg"]) < 3:
            state["manual_rpy_deg"].append(0.0)
        state["source_frame_override"] = str(state.get("source_frame_override", "")).strip()
        state["sample_step"] = max(1, int(state.get("sample_step", 2) or 2))
        state["max_points"] = max(32, int(state.get("max_points", 4000) or 4000))
        return state

    def _ros_param_set(self, node_name: str, param_name: str, value_literal: str) -> tuple[bool, str]:
        command = (
            self.ros_shell_prefix()
            + "export ROS2CLI_NO_DAEMON=1; "
            + f"ros2 param set {shlex.quote(node_name)} {shlex.quote(param_name)} "
            + value_literal
        )
        result = subprocess.run(
            shell_command(command),
            capture_output=True,
            text=True,
            timeout=12,
        )
        output = (result.stdout or "").strip()
        error = (result.stderr or "").strip()
        if result.returncode == 0:
            return True, output or "ok"
        return False, error or output or f"设置参数 {param_name} 失败"

    def save_depth_coordinate_tuning(self, payload: dict[str, Any]) -> dict[str, Any]:
        state = self.load_depth_coordinate_tuning()
        state["use_tf_transform"] = bool(payload.get("use_tf_transform", state["use_tf_transform"]))
        state["use_manual_transform_fallback"] = bool(
            payload.get("use_manual_transform_fallback", state["use_manual_transform_fallback"])
        )
        if "manual_translation_m" in payload:
            values = [float(v) for v in list(payload["manual_translation_m"])[:3]]
            while len(values) < 3:
                values.append(0.0)
            state["manual_translation_m"] = values
        if "manual_rpy_deg" in payload:
            values = [float(v) for v in list(payload["manual_rpy_deg"])[:3]]
            while len(values) < 3:
                values.append(0.0)
            state["manual_rpy_deg"] = values
        if "source_frame_override" in payload:
            state["source_frame_override"] = str(payload["source_frame_override"]).strip()
        if "sample_step" in payload:
            state["sample_step"] = max(1, int(payload["sample_step"]))
        if "max_points" in payload:
            state["max_points"] = max(32, int(payload["max_points"]))

        write_json_file(DEPTH_COORDINATE_TUNING_PATH, state)

        node_name = "/depth_coordinate_node"
        results = []
        literals = {
            "use_tf_transform": "true" if state["use_tf_transform"] else "false",
            "use_manual_transform_fallback": (
                "true" if state["use_manual_transform_fallback"] else "false"
            ),
            "manual_translation_m": "'" + json.dumps(state["manual_translation_m"]) + "'",
            "manual_rpy_deg": "'" + json.dumps(state["manual_rpy_deg"]) + "'",
            "source_frame_override": shlex.quote(state["source_frame_override"] or ""),
            "sample_step": str(state["sample_step"]),
            "max_points": str(state["max_points"]),
        }
        for param_name, literal in literals.items():
            ok, message = self._ros_param_set(node_name, param_name, literal)
            results.append({"param": param_name, "ok": ok, "message": message})
            if not ok:
                return {
                    "ok": False,
                    "error": f"{param_name} 更新失败: {message}",
                    "state": state,
                    "results": results,
                }

        return {"ok": True, "state": state, "results": results}

    def command_catalog(self) -> list[dict[str, str]]:
        launch_base = "ros2 launch deyes_bringup imx219_stereo.launch.py enable_monitor:=true use_cpp_capture:=true"
        precheck_common = (
            "target_publish_hz:=30.0 pair_max_skew_ms:=20.0 frame_stale_sec:=0.2 history_size:=8 "
            "monitor_expected_min_rate_hz:=20.0 monitor_hard_sync_max_ms:=3.0 "
            "monitor_soft_sync_max_ms:=10.0 monitor_allow_soft_sync:=false"
        )
        prefix = self.ros_shell_prefix()
        return [
            {
                "id": "precheck_1280",
                "label": "1280 预检",
                "description": "35 秒预热检查 1280x720@30 的同步与发布状态。",
                "command": (
                    prefix
                    + "timeout --signal=INT 35s "
                    + launch_base
                    + " width:=1280 height:=720 fps:=30 "
                    + precheck_common
                ),
            },
            {
                "id": "precheck_720",
                "label": "720 预检",
                "description": "35 秒预热检查 640x360@30，作为回退验证。",
                "command": (
                    prefix
                    + "timeout --signal=INT 35s "
                    + launch_base
                    + " width:=640 height:=360 fps:=30 "
                    + precheck_common
                ),
            },
            {
                "id": "capture",
                "label": "采集棋盘格",
                "description": "调用 mercury_grasp 的棋盘格采集流程。",
                "command": prefix + f"python3 {shlex.quote(str(self.calib_tool))} capture",
            },
            {
                "id": "compute",
                "label": "计算标定",
                "description": "基于现有采样运行 stereo calibration compute。",
                "command": prefix + f"python3 {shlex.quote(str(self.calib_tool))} compute",
            },
            {
                "id": "stereo_image_proc",
                "label": "官方基线",
                "description": "启动 stereo_image_proc 基线，检查标定后的几何链路。",
                "command": prefix
                + "timeout --signal=INT 35s ros2 launch deyes_bringup stereo_image_proc_baseline.launch.py",
            },
            {
                "id": "sgbm",
                "label": "SGBM 基线",
                "description": "启动 OpenCV StereoSGBM 基线做 debug 复验。",
                "command": prefix
                + "timeout --signal=INT 35s ros2 launch deyes_bringup sgbm_baseline.launch.py",
            },
        ]

    def command_by_id(self, task_id: str) -> dict[str, str] | None:
        for item in self.command_catalog():
            if item["id"] == task_id:
                return item
        return None

    def next_calib_pair_index(self) -> int:
        highest = -1
        if self.calib_dir.exists():
            for path in self.calib_dir.glob("pair*_left.png"):
                stem = path.name.split("pair", 1)[-1].split("_left", 1)[0]
                if stem.isdigit():
                    highest = max(highest, int(stem))
        return highest + 1

    def inspect_tool(self) -> dict[str, Any]:
        if not self.calib_tool.exists():
            return {
                "exists": False,
                "mode": "unknown",
                "checkerboard_supported": False,
                "charuco_supported": False,
                "checkerboard_spec": None,
            }
        text = self.calib_tool.read_text(encoding="utf-8", errors="ignore")
        lower = text.lower()
        match = re.search(r"CHECKERBOARD\s*=\s*\((\d+)\s*,\s*(\d+)\)", text)
        spec = f"{match.group(1)} x {match.group(2)}" if match else None
        return {
            "exists": True,
            "mode": "checkerboard" if "findchessboardcorners" in lower else "unknown",
            "checkerboard_supported": "findchessboardcorners" in lower,
            "charuco_supported": "charuco" in lower or "aruco" in lower,
            "checkerboard_spec": spec,
        }

    def tail_log(self, path: Path | None, max_lines: int = 200) -> str:
        if path is None or not path.exists():
            return ""
        text = path.read_text(encoding="utf-8", errors="ignore")
        lines = text.splitlines()
        return "\n".join(lines[-max_lines:])

    def latest_successful_task(self, task_ids: list[str]) -> dict[str, Any] | None:
        candidates = [
            item
            for item in self.task_manager.history()
            if item.get("task_id") in task_ids and item.get("return_code") == 0
        ]
        if not candidates:
            return None
        candidates.sort(key=lambda item: iso_to_epoch(item.get("finished_at") or item.get("started_at")), reverse=True)
        return candidates[0]

    def latest_compute_summary(self) -> dict[str, Any]:
        compute_task = self.latest_successful_task(["compute"])
        summary = {
            "available": False,
            "reproj_error": None,
            "yaml_path": None,
            "has_core_matrices": False,
            "source_log": compute_task.get("log_path") if compute_task else None,
        }
        if not compute_task:
            return summary
        log_path = Path(str(compute_task.get("log_path", "")))
        if not log_path.exists():
            return summary
        text = log_path.read_text(encoding="utf-8", errors="ignore")
        reproj_match = re.search(r"reproj(?:ection)?[_\s-]*error[^0-9]*([0-9]+(?:\.[0-9]+)?)", text, re.I)
        yaml_match = re.search(r"(/[^\s]+\.ya?ml)", text)
        core_keys = all(key in text for key in ["K1", "D1", "K2", "D2", "R", "T"])
        summary["available"] = True
        summary["reproj_error"] = float(reproj_match.group(1)) if reproj_match else None
        summary["yaml_path"] = yaml_match.group(1) if yaml_match else None
        summary["has_core_matrices"] = core_keys
        return summary

    def build_wizard(self) -> dict[str, Any]:
        form = self.load_form()
        wizard_state = self.load_wizard_state()
        compute_summary = self.latest_compute_summary()
        precheck_ok = self.latest_successful_task(["precheck_1280"]) is not None
        form_ok = bool(form.get("board_id")) and (float_or_none(form.get("square_size_mm")) or 0.0) > 0.0
        capture_ok = self.calib_dir.exists() and len(list(self.calib_dir.glob("*"))) > 0
        compute_ok = compute_summary["available"]
        review_ok = wizard_state.get("review_passed") is True
        baseline_ok = (
            self.latest_successful_task(["stereo_image_proc"]) is not None
            and self.latest_successful_task(["sgbm"]) is not None
        )
        ordered = [
            ("precheck", "步骤 1 / 预检", "先确认 1280x720@30 的同步与发布链稳定。", precheck_ok),
            ("board", "步骤 2 / 板信息", "录入棋盘格板编号与单方格边长。", form_ok),
            ("capture", "步骤 3 / 采集", "边看双路反馈边采集至少 30 组样本。", capture_ok),
            ("compute", "步骤 4 / 计算", "运行标定求解并提取 reproj_error 与 YAML 路径。", compute_ok),
            ("review", "步骤 5 / 验收", "根据 reproj_error 与停止条件判定是否通过。", review_ok),
            ("baseline", "步骤 6 / 复验", "执行官方基线与 SGBM 基线复验。", baseline_ok),
        ]

        current = wizard_state.get("current_step", "precheck")
        if current not in {item[0] for item in ordered}:
            current = "precheck"

        steps = []
        seen_block = False
        for step_id, title, description, done in ordered:
            if done:
                status = "completed"
            elif not seen_block:
                status = "current"
                current = step_id
                seen_block = True
            else:
                status = "pending"
            steps.append({"id": step_id, "title": title, "description": description, "status": status})

        return {
            "current_step": current,
            "steps": steps,
            "review": {
                "passed": wizard_state.get("review_passed"),
                "note": wizard_state.get("review_note", ""),
            },
            "form_ready": form_ok,
            "compute_summary": compute_summary,
        }

    def metrics(self) -> dict[str, Any]:
        history = self.task_manager.history()
        return {
            "task_history_total": len(history),
            "recent_success_count": sum(1 for item in history if item.get("return_code") == 0),
            "recent_failure_count": sum(1 for item in history if item.get("return_code") not in (0, None)),
            "calib_samples": len(list(self.calib_dir.glob("*"))) if self.calib_dir.exists() else 0,
            "calib_pairs": len(list(self.calib_dir.glob("pair*_left.png"))) if self.calib_dir.exists() else 0,
        }

    def status(self) -> dict[str, Any]:
        repo_calibs = sorted(path.name for path in self.repo_calib_dir.glob("*.yaml")) if self.repo_calib_dir.exists() else []
        task = self.task_manager.snapshot()
        active_log = self.tail_log(Path(task["log_path"])) if task.get("log_path") else ""
        return {
            "hostname": socket.gethostname(),
            "platform": platform.platform(),
            "python": platform.python_version(),
            "time": utc_now(),
            "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
            "paths": {
                "repo_root": str(self.repo_root),
                "workspace_root": str(self.workspace_root),
                "mercury_root": str(self.mercury_root),
                "repo_calib_dir": str(self.repo_calib_dir),
                "calib_dir": str(self.calib_dir),
                "placeholder_calib": str(self.placeholder_calib),
            },
            "files": {
                "calib_tool_exists": self.calib_tool.exists(),
                "calib_dir_exists": self.calib_dir.exists(),
                "placeholder_calib_exists": self.placeholder_calib.exists(),
                "repo_calib_count": len(repo_calibs),
                "repo_calibs": repo_calibs,
                "calib_samples": len(list(self.calib_dir.glob("*"))) if self.calib_dir.exists() else 0,
                "calib_pairs": len(list(self.calib_dir.glob("pair*_left.png"))) if self.calib_dir.exists() else 0,
            },
            "tool": self.inspect_tool(),
            "commands": self.command_catalog(),
            "task": task,
            "task_history": self.task_manager.history(),
            "log_tail": active_log,
            "checkerboard_guidance": {
                "inner_corners": "9 x 6",
                "print_scale": "100%",
                "required_notes": [
                    "先记录单个方格边长（毫米）",
                    "采集前预热到 publish_hz 接近 30Hz",
                    "至少采集 30 组，覆盖中心、四角、近中远和倾斜姿态",
                ],
            },
            "topics": {
                "left_image": self.left_image_topic,
                "right_image": self.right_image_topic,
                "depth_heatmap": self.depth_heatmap_topic,
                "depth_heatmap_status": self.depth_heatmap_status_topic,
            },
            "calibration": {
                "form": self.load_form(),
                "wizard": self.build_wizard(),
            },
            "vision": self.load_vision_mode(),
            "vision_runtime": {
                "left": self.vision_runtime_state("left"),
                "right": self.vision_runtime_state("right"),
            },
            "depth_heatmap_runtime": self.depth_heatmap_runtime_state(),
            "depth_coordinate_tuning": self.load_depth_coordinate_tuning(),
            "metrics": self.metrics(),
        }

    def vision_runtime_state(self, side: str) -> dict[str, Any]:
        with self._vision_state_lock:
            return dict(self._vision_state[side])

    def _update_vision_state(self, side: str, **changes: Any) -> None:
        with self._vision_state_lock:
            self._vision_state[side].update(changes)

    def depth_heatmap_runtime_state(self) -> dict[str, Any]:
        with self._depth_heatmap_lock:
            return dict(self._depth_heatmap_state)

    def _update_depth_heatmap_state(self, **changes: Any) -> None:
        with self._depth_heatmap_lock:
            self._depth_heatmap_state.update(changes)

    def _run_snapshot_command(
        self,
        side: str,
        timeout_sec: float,
        max_width: int,
        jpeg_quality: int,
    ) -> tuple[bool, str]:
        cache_path = VISION_CACHE[side]
        topic = self.left_image_topic if side == "left" else self.right_image_topic
        command = (
            self.ros_shell_prefix()
            + f"python3 {shlex.quote(str(self.vision_helper))} "
            + f"--topic {shlex.quote(topic)} --output {shlex.quote(str(cache_path))} "
            + f"--timeout {max(0.5, timeout_sec):.2f} "
            + f"--max-width {max(0, max_width)} --jpeg-quality {max(25, min(jpeg_quality, 95))}"
        )
        result = subprocess.run(
            shell_command(command),
            capture_output=True,
            text=True,
            timeout=max(6, int(timeout_sec) + 6),
        )
        if result.returncode == 0 and cache_path.exists():
            return True, ""
        message = result.stderr.strip() or result.stdout.strip() or f"{side} 画面暂不可用"
        return False, message

    def save_current_pair(self, timeout_sec: float = 4.0) -> dict[str, Any]:
        if os.name == "nt":
            raise RuntimeError("成对记录仅支持在机器人端执行。")

        task = self.task_manager.snapshot()
        if task.get("running"):
            raise RuntimeError("当前已有后台任务在运行，请先停止后再记录。")

        self.calib_dir.mkdir(parents=True, exist_ok=True)
        with self._vision_capture_lock:
            left_ok, left_message = self._run_snapshot_command("left", timeout_sec, 0, 95)
            if not left_ok:
                raise RuntimeError(left_message or "左目抓图失败")
            right_ok, right_message = self._run_snapshot_command("right", timeout_sec, 0, 95)
            if not right_ok:
                raise RuntimeError(right_message or "右目抓图失败")

        left_image = cv2.imread(str(VISION_CACHE["left"]))
        right_image = cv2.imread(str(VISION_CACHE["right"]))
        if left_image is None or right_image is None:
            raise RuntimeError("已抓到图像，但读取缓存失败。")

        pair_index = self.next_calib_pair_index()
        left_path = self.calib_dir / f"pair{pair_index}_left.png"
        right_path = self.calib_dir / f"pair{pair_index}_right.png"
        if not cv2.imwrite(str(left_path), left_image):
            raise RuntimeError(f"写入失败: {left_path}")
        if not cv2.imwrite(str(right_path), right_image):
            raise RuntimeError(f"写入失败: {right_path}")

        total_pairs = len(list(self.calib_dir.glob("pair*_left.png")))
        return {
            "ok": True,
            "pair_index": pair_index,
            "left_path": str(left_path),
            "right_path": str(right_path),
            "total_pairs": total_pairs,
        }

    def _refresh_vision_cache(
        self,
        side: str,
        timeout_sec: float,
        max_width: int,
        jpeg_quality: int,
    ) -> None:
        self._update_vision_state(side, refreshing=True, last_attempt_at=utc_now())
        try:
            with self._vision_capture_lock:
                ok, message = self._run_snapshot_command(side, timeout_sec, max_width, jpeg_quality)
            if ok:
                self._update_vision_state(
                    side,
                    refreshing=False,
                    last_error="",
                    last_ok_at=utc_now(),
                )
                return
            self._update_vision_state(side, refreshing=False, last_error=message)
        except subprocess.TimeoutExpired:
            self._update_vision_state(side, refreshing=False, last_error=f"{side} 抓图进程超时")
        except Exception as exc:
            self._update_vision_state(side, refreshing=False, last_error=str(exc))

    def request_vision_refresh(
        self,
        side: str,
        timeout_sec: float,
        max_width: int,
        jpeg_quality: int,
    ) -> None:
        with self._vision_state_lock:
            if self._vision_state[side].get("refreshing"):
                return
            self._vision_state[side]["refreshing"] = True
            self._vision_state[side]["last_attempt_at"] = utc_now()
        worker = threading.Thread(
            target=self._refresh_vision_cache,
            args=(side, timeout_sec, max_width, jpeg_quality),
            daemon=True,
            name=f"vision-refresh-{side}",
        )
        worker.start()

    def vision_response(
        self,
        side: str,
        force_refresh: bool = False,
        timeout_sec: float = 4.0,
        max_width: int = 0,
        jpeg_quality: int = 85,
    ) -> tuple[bytes, str]:
        cache_path = VISION_CACHE[side]
        if os.name == "nt":
            return self.placeholder_image("视觉反馈仅在机器人端可用。"), "image/svg+xml; charset=utf-8"

        mode = self.load_vision_mode()
        now = time.time()
        refresh_window = max(1.0, mode.get("refresh_ms", 2000) / 1000.0)
        cache_exists = cache_path.exists()
        cache_fresh = cache_exists and now - cache_path.stat().st_mtime < refresh_window
        if not force_refresh and cache_fresh:
            return cache_path.read_bytes(), "image/jpeg"

        self.request_vision_refresh(side, timeout_sec, max_width, jpeg_quality)
        if cache_exists:
            return cache_path.read_bytes(), "image/jpeg"

        runtime = self.vision_runtime_state(side)
        if runtime.get("refreshing"):
            message = f"{side} 首帧抓取中，请等待下一轮刷新"
        else:
            message = runtime.get("last_error") or f"{side} 画面暂不可用"
        return self.placeholder_image(message), "image/svg+xml; charset=utf-8"

    def _run_depth_heatmap_command(self, timeout_sec: float) -> tuple[bool, str]:
        command = (
            self.ros_shell_prefix()
            + f"python3 {shlex.quote(str(self.depth_heatmap_helper))} "
            + f"--topic {shlex.quote(self.depth_heatmap_topic)} "
            + f"--output {shlex.quote(str(DEPTH_HEATMAP_CACHE))} "
            + f"--timeout {max(0.5, timeout_sec):.2f}"
        )
        result = subprocess.run(
            shell_command(command),
            capture_output=True,
            text=True,
            timeout=max(6, int(timeout_sec) + 6),
        )
        if result.returncode == 0 and DEPTH_HEATMAP_CACHE.exists():
            return True, ""
        message = result.stderr.strip() or result.stdout.strip() or "热力图数据暂不可用"
        return False, message

    def _depth_heatmap_cache_signature(self) -> str:
        if not DEPTH_HEATMAP_CACHE.exists():
            return ""
        try:
            payload = json.loads(DEPTH_HEATMAP_CACHE.read_text(encoding="utf-8"))
            stamp_sec = payload.get("stamp_sec")
            stamp_nanosec = payload.get("stamp_nanosec")
            point_count = payload.get("point_count")
            if stamp_sec is not None and stamp_nanosec is not None:
                return f"stamp:{int(stamp_sec)}:{int(stamp_nanosec)}:{int(point_count or 0)}"
        except Exception:
            pass
        stat = DEPTH_HEATMAP_CACHE.stat()
        return f"file:{stat.st_mtime_ns}:{stat.st_size}"

    def refresh_depth_heatmap_sync(
        self, timeout_sec: float, previous_signature: str = ""
    ) -> tuple[bool, str]:
        with self._depth_heatmap_lock:
            if self._depth_heatmap_state.get("refreshing"):
                return False, "热力图刷新仍在进行中"
            self._depth_heatmap_state["refreshing"] = True
            self._depth_heatmap_state["last_attempt_at"] = utc_now()

        try:
            ok, message = self._run_depth_heatmap_command(timeout_sec)
            if ok and DEPTH_HEATMAP_CACHE.exists():
                new_signature = self._depth_heatmap_cache_signature()
                if previous_signature and new_signature == previous_signature:
                    self._update_depth_heatmap_state(refreshing=False, last_error="热力图未刷新到新帧")
                    return False, "热力图未刷新到新帧"
                self._update_depth_heatmap_state(
                    refreshing=False,
                    last_error="",
                    last_ok_at=utc_now(),
                )
                return True, ""
            self._update_depth_heatmap_state(refreshing=False, last_error=message)
            return False, message
        except subprocess.TimeoutExpired:
            self._update_depth_heatmap_state(refreshing=False, last_error="热力图抓取进程超时")
            return False, "热力图抓取进程超时"
        except Exception as exc:
            self._update_depth_heatmap_state(refreshing=False, last_error=str(exc))
            return False, str(exc)

    def _refresh_depth_heatmap_cache(self, timeout_sec: float) -> None:
        self._update_depth_heatmap_state(refreshing=True, last_attempt_at=utc_now())
        try:
            ok, message = self._run_depth_heatmap_command(timeout_sec)
            if ok:
                self._update_depth_heatmap_state(
                    refreshing=False,
                    last_error="",
                    last_ok_at=utc_now(),
                )
                return
            self._update_depth_heatmap_state(refreshing=False, last_error=message)
        except subprocess.TimeoutExpired:
            self._update_depth_heatmap_state(refreshing=False, last_error="热力图抓取进程超时")
        except Exception as exc:
            self._update_depth_heatmap_state(refreshing=False, last_error=str(exc))

    def request_depth_heatmap_refresh(self, timeout_sec: float) -> None:
        with self._depth_heatmap_lock:
            if self._depth_heatmap_state.get("refreshing"):
                return
            self._depth_heatmap_state["refreshing"] = True
            self._depth_heatmap_state["last_attempt_at"] = utc_now()
        worker = threading.Thread(
            target=self._refresh_depth_heatmap_cache,
            args=(timeout_sec,),
            daemon=True,
            name="depth-heatmap-refresh",
        )
        worker.start()

    def depth_heatmap_response(
        self,
        force_refresh: bool = False,
        timeout_sec: float = 2.5,
        refresh_window_sec: float = 0.7,
    ) -> tuple[bytes, str]:
        if os.name == "nt":
            payload = {"ok": False, "error": "热力图预览仅在机器人端可用。"}
            return json.dumps(payload, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8"

        now = time.time()
        cache_exists = DEPTH_HEATMAP_CACHE.exists()
        cache_fresh = cache_exists and now - DEPTH_HEATMAP_CACHE.stat().st_mtime < refresh_window_sec
        if not force_refresh and cache_fresh:
            return DEPTH_HEATMAP_CACHE.read_bytes(), "application/json; charset=utf-8"

        if force_refresh:
            previous_signature = self._depth_heatmap_cache_signature() if cache_exists else ""
            ok, message = self.refresh_depth_heatmap_sync(timeout_sec, previous_signature)
            if ok and DEPTH_HEATMAP_CACHE.exists():
                return DEPTH_HEATMAP_CACHE.read_bytes(), "application/json; charset=utf-8"
            payload = {"ok": False, "error": message or "热力图数据暂不可用", "topic": self.depth_heatmap_topic}
            return json.dumps(payload, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8"

        self.request_depth_heatmap_refresh(timeout_sec)
        if cache_exists:
            return DEPTH_HEATMAP_CACHE.read_bytes(), "application/json; charset=utf-8"

        runtime = self.depth_heatmap_runtime_state()
        if runtime.get("refreshing"):
            message = "热力图首帧抓取中，请稍候"
        else:
            message = runtime.get("last_error") or "热力图数据暂不可用"
        payload = {"ok": False, "error": message, "topic": self.depth_heatmap_topic}
        return json.dumps(payload, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8"

    def stereo_preview_page(self) -> bytes:
        left_topic = html.escape(self.left_image_topic)
        right_topic = html.escape(self.right_image_topic)
        html_body = f"""
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Stereo Mainline Preview</title>
  <style>
    :root {{
      --bg: #050816;
      --panel: rgba(11, 18, 33, 0.92);
      --panel-border: rgba(111, 210, 255, 0.24);
      --text: #edf5ff;
      --muted: #8ea0ba;
      --accent: #6fe5ff;
      --accent-2: #89ffb5;
      --warning: #ffd36f;
      --danger: #ff8f8f;
      --shadow: 0 24px 80px rgba(0, 0, 0, 0.42);
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: "Segoe UI", "PingFang SC", "Microsoft YaHei", sans-serif;
      color: var(--text);
      background:
        radial-gradient(circle at top left, rgba(79, 171, 255, 0.22), transparent 28%),
        radial-gradient(circle at top right, rgba(111, 255, 181, 0.16), transparent 24%),
        linear-gradient(180deg, #09111f 0%, var(--bg) 50%, #03060f 100%);
      min-height: 100vh;
    }}
    .shell {{
      width: min(1440px, calc(100vw - 32px));
      margin: 24px auto;
      display: grid;
      gap: 18px;
    }}
    .hero, .panel {{
      background: var(--panel);
      border: 1px solid var(--panel-border);
      border-radius: 24px;
      box-shadow: var(--shadow);
      backdrop-filter: blur(16px);
    }}
    .hero {{
      padding: 24px 28px;
      display: grid;
      grid-template-columns: 1.2fr 0.8fr;
      gap: 18px;
      overflow: hidden;
      position: relative;
    }}
    .hero::after {{
      content: "";
      position: absolute;
      inset: auto -40px -80px auto;
      width: 220px;
      height: 220px;
      border-radius: 50%;
      background: radial-gradient(circle, rgba(111,229,255,.22), transparent 68%);
      pointer-events: none;
    }}
    .eyebrow {{
      color: var(--accent);
      text-transform: uppercase;
      letter-spacing: 0.24em;
      font-size: 12px;
      margin-bottom: 12px;
    }}
    h1 {{
      margin: 0 0 10px;
      font-size: clamp(28px, 4vw, 52px);
      line-height: 1;
      letter-spacing: -0.04em;
    }}
    .hero p {{
      margin: 0;
      color: var(--muted);
      max-width: 64ch;
      line-height: 1.65;
    }}
    .chips {{
      margin-top: 18px;
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
    }}
    .chip {{
      border: 1px solid rgba(111, 229, 255, 0.2);
      border-radius: 999px;
      padding: 9px 14px;
      color: #d7f9ff;
      background: rgba(10, 22, 41, 0.64);
      font-size: 13px;
    }}
    .hero-side {{
      display: grid;
      gap: 12px;
      align-content: start;
    }}
    .metric {{
      padding: 16px 18px;
      border-radius: 18px;
      background: rgba(7, 14, 28, 0.78);
      border: 1px solid rgba(255, 255, 255, 0.06);
    }}
    .metric strong {{
      display: block;
      font-size: 13px;
      color: var(--muted);
      margin-bottom: 8px;
      font-weight: 600;
    }}
    .metric span {{
      display: block;
      font-size: 24px;
      letter-spacing: -0.03em;
    }}
    .controls {{
      display: flex;
      flex-wrap: wrap;
      gap: 12px;
      align-items: center;
      justify-content: space-between;
      padding: 18px 20px;
    }}
    .controls-left {{
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      align-items: center;
    }}
    .controls label {{
      color: var(--muted);
      font-size: 13px;
    }}
    .controls input[type="range"] {{
      width: 180px;
      accent-color: var(--accent);
    }}
    .button {{
      border: 0;
      border-radius: 14px;
      padding: 11px 16px;
      color: #02111f;
      background: linear-gradient(135deg, var(--accent), #b8f9ff);
      font-weight: 700;
      cursor: pointer;
    }}
    .button.secondary {{
      color: var(--text);
      background: rgba(255,255,255,0.06);
      border: 1px solid rgba(255,255,255,0.08);
    }}
    .button.warning {{
      color: #201300;
      background: linear-gradient(135deg, var(--warning), #ffe59e);
    }}
    .button:disabled {{
      opacity: 0.55;
      cursor: wait;
    }}
    .ops-panel {{
      padding: 18px 20px;
      display: grid;
      gap: 14px;
    }}
    .ops-row {{
      display: flex;
      flex-wrap: wrap;
      gap: 12px;
      align-items: center;
      justify-content: space-between;
    }}
    .ops-actions {{
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
    }}
    .ops-summary {{
      display: flex;
      flex-wrap: wrap;
      gap: 18px;
      color: var(--muted);
      font-size: 13px;
    }}
    .ops-summary strong {{
      color: var(--text);
    }}
    .ops-status {{
      min-height: 22px;
      color: #dffaff;
      font-size: 13px;
    }}
    .stream-grid {{
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 18px;
    }}
    .stream-card {{
      padding: 16px;
      display: grid;
      gap: 12px;
    }}
    .stream-top {{
      display: flex;
      justify-content: space-between;
      gap: 12px;
      align-items: center;
    }}
    .stream-title {{
      display: grid;
      gap: 4px;
    }}
    .stream-title strong {{
      font-size: 20px;
      letter-spacing: -0.03em;
    }}
    .stream-title span {{
      color: var(--muted);
      font-size: 12px;
      font-family: Consolas, "SFMono-Regular", monospace;
      word-break: break-all;
    }}
    .pulse {{
      display: inline-flex;
      align-items: center;
      gap: 8px;
      font-size: 12px;
      color: #cbffee;
      padding: 8px 12px;
      border-radius: 999px;
      background: rgba(14, 31, 26, 0.9);
      border: 1px solid rgba(111, 255, 181, 0.18);
    }}
    .pulse::before {{
      content: "";
      width: 10px;
      height: 10px;
      border-radius: 50%;
      background: var(--accent-2);
      box-shadow: 0 0 0 0 rgba(137,255,181,.45);
      animation: pulse 1.5s infinite;
    }}
    @keyframes pulse {{
      0% {{ box-shadow: 0 0 0 0 rgba(137,255,181,.45); }}
      70% {{ box-shadow: 0 0 0 12px rgba(137,255,181,0); }}
      100% {{ box-shadow: 0 0 0 0 rgba(137,255,181,0); }}
    }}
    .viewport {{
      position: relative;
      border-radius: 20px;
      overflow: hidden;
      border: 1px solid rgba(255,255,255,0.06);
      background: #02050d;
      aspect-ratio: 16 / 9;
    }}
    .viewport img {{
      width: 100%;
      height: 100%;
      object-fit: contain;
      display: block;
      background: #000;
    }}
    .viewport-overlay {{
      position: absolute;
      left: 14px;
      right: 14px;
      bottom: 14px;
      display: flex;
      justify-content: space-between;
      gap: 10px;
      align-items: center;
      padding: 10px 12px;
      border-radius: 14px;
      background: linear-gradient(180deg, rgba(4,8,16,0.14), rgba(4,8,16,0.84));
      font-size: 12px;
    }}
    .timestamp {{ color: #dffaff; font-family: Consolas, monospace; }}
    .status {{
      color: var(--warning);
      font-size: 12px;
      min-height: 18px;
    }}
    .footer {{
      padding: 18px 20px;
      display: flex;
      flex-wrap: wrap;
      gap: 12px 18px;
      color: var(--muted);
      font-size: 13px;
      justify-content: space-between;
    }}
    .footer code {{
      color: #d7f9ff;
      font-family: Consolas, "SFMono-Regular", monospace;
    }}
    @media (max-width: 1100px) {{
      .hero {{ grid-template-columns: 1fr; }}
      .stream-grid {{ grid-template-columns: 1fr; }}
    }}
  </style>
</head>
<body>
  <main class="shell">
    <section class="hero">
      <div>
        <div class="eyebrow">Stereo Calibration Mainline</div>
        <h1>双目主链实时预览</h1>
        <p>这个页面直接调用和当前标定采集一致的 ROS/C++ 主链话题，不走 `grab_stereo.py` 直连旁路。当前已切成低带宽模式：顺序抓左再抓右，默认缩到 480 宽、JPEG 质量 45、1.8 秒一轮，更适合现场链路测试。</p>
        <div class="chips">
          <div class="chip">左目：<code>{left_topic}</code></div>
          <div class="chip">右目：<code>{right_topic}</code></div>
          <div class="chip">模式：低带宽连续快照</div>
        </div>
      </div>
      <div class="hero-side">
        <div class="metric">
          <strong>当前链路</strong>
          <span>ROS / C++ Mainline</span>
        </div>
        <div class="metric">
          <strong>用途</strong>
          <span>标定前视野确认</span>
        </div>
        <div class="metric">
          <strong>建议</strong>
          <span>先中央对中，再扩姿态</span>
        </div>
      </div>
    </section>

    <section class="panel controls">
      <div class="controls-left">
        <label for="refreshRange">刷新间隔</label>
        <input id="refreshRange" type="range" min="800" max="5000" step="100" value="1800" />
        <strong id="refreshLabel">1800 ms</strong>
      </div>
      <div class="controls-left">
        <button id="toggleButton" class="button">暂停刷新</button>
        <button id="reloadButton" class="button secondary">立刻重抓一帧</button>
      </div>
    </section>

    <section class="panel ops-panel">
      <div class="ops-row">
        <div class="ops-summary">
          <span>已记录样本：<strong id="pairCount">--</strong> 对</span>
          <span>后台任务：<strong id="taskState">空闲</strong></span>
        </div>
        <div class="ops-actions">
          <button id="recordPairButton" class="button warning">记录当前一对</button>
          <button id="computeButton" class="button">计算标定</button>
          <button id="stopTaskButton" class="button secondary">停止任务</button>
        </div>
      </div>
      <div id="opsStatus" class="ops-status">页面已连接，可直接记录当前左右图像为一对标定样本。</div>
    </section>

    <section class="stream-grid">
      <article class="panel stream-card">
        <div class="stream-top">
          <div class="stream-title">
            <strong>左目预览</strong>
            <span>{left_topic}</span>
          </div>
          <div class="pulse">LIVE</div>
        </div>
        <div class="viewport">
          <img id="leftFrame" alt="left stream" src="/api/vision/left/stream?width=480&quality=45&timeout=2.4&interval=1.8" />
          <div class="viewport-overlay">
            <span class="timestamp" id="leftTimestamp">等待首帧...</span>
            <span class="status" id="leftStatus"></span>
          </div>
        </div>
      </article>

      <article class="panel stream-card">
        <div class="stream-top">
          <div class="stream-title">
            <strong>右目预览</strong>
            <span>{right_topic}</span>
          </div>
          <div class="pulse">LIVE</div>
        </div>
        <div class="viewport">
          <img id="rightFrame" alt="right stream" src="/api/vision/right/stream?width=480&quality=45&timeout=2.4&interval=1.8" />
          <div class="viewport-overlay">
            <span class="timestamp" id="rightTimestamp">等待首帧...</span>
            <span class="status" id="rightStatus"></span>
          </div>
        </div>
      </article>
    </section>

    <section class="panel footer">
      <span>入口：<code>/stereo-preview</code></span>
      <span>接口：<code>/api/vision/left/stream?width=480&quality=45</code> / <code>/api/vision/right/stream?width=480&quality=45</code></span>
      <span>说明：当前是 MJPEG 连续流，底层仍来自与标定一致的 ROS/C++ 主链。</span>
    </section>
  </main>

  <script>
    const state = {{
      running: true,
      refreshMs: 1800,
      maxWidth: 480,
      quality: 45,
      timeoutSec: 2.4,
    }};

    const refreshRange = document.getElementById("refreshRange");
    const refreshLabel = document.getElementById("refreshLabel");
    const toggleButton = document.getElementById("toggleButton");
    const reloadButton = document.getElementById("reloadButton");
    const pairCount = document.getElementById("pairCount");
    const taskState = document.getElementById("taskState");
    const opsStatus = document.getElementById("opsStatus");
    const recordPairButton = document.getElementById("recordPairButton");
    const computeButton = document.getElementById("computeButton");
    const stopTaskButton = document.getElementById("stopTaskButton");

    function stampNow() {{
      return new Date().toLocaleTimeString("zh-CN", {{ hour12: false }});
    }}

    function setOpsStatus(message, isError = false) {{
      opsStatus.textContent = message;
      opsStatus.style.color = isError ? "var(--danger)" : "#dffaff";
    }}

    function updateStreamSource(side) {{
      const img = document.getElementById(`${{side}}Frame`);
      const ts = document.getElementById(`${{side}}Timestamp`);
      const status = document.getElementById(`${{side}}Status`);
      status.textContent = state.running ? "连续流连接中..." : "已暂停在当前画面";
      const url = `/api/vision/${{side}}/stream?width=${{state.maxWidth}}&quality=${{state.quality}}&timeout=${{state.timeoutSec}}&interval=${{(state.refreshMs / 1000).toFixed(1)}}&t=${{Date.now()}}`;
      img.onload = () => {{
        ts.textContent = `连续流在线 ${{stampNow()}}`;
        status.textContent = state.running ? "" : "已暂停在当前画面";
      }};
      img.onerror = () => {{
        ts.textContent = `流连接失败 ${{stampNow()}}`;
        status.textContent = "请检查 ROS 主链是否仍在发布";
      }};
      if (state.running) {{
        img.src = url;
      }}
    }}

    function refreshStreams() {{
      updateStreamSource("left");
      updateStreamSource("right");
    }}

    async function loadStatus() {{
      try {{
        const response = await fetch("/api/status", {{ cache: "no-store" }});
        const data = await response.json();
        pairCount.textContent = String(data.files?.calib_pairs || 0);
        const task = data.task || {{}};
        taskState.textContent = task.running ? `${{task.label || task.task_id || "运行中"}}` : "空闲";
      }} catch (error) {{
        setOpsStatus(`状态读取失败：${{error.message}}`, true);
      }}
    }}

    async function postJson(url, payload = {{}}) {{
      const response = await fetch(url, {{
        method: "POST",
        headers: {{ "Content-Type": "application/json" }},
        body: JSON.stringify(payload),
      }});
      const data = await response.json();
      if (!response.ok || data.ok === false) {{
        throw new Error(data.error || `请求失败 (${{response.status}})`);
      }}
      return data;
    }}

    async function recordPair() {{
      recordPairButton.disabled = true;
      computeButton.disabled = true;
      setOpsStatus("正在记录当前一对，请保持棋盘格静止...");
      try {{
        const data = await postJson("/api/vision/save-pair", {{ timeout: 4.0 }});
        setOpsStatus(`已保存 pair${{data.pair_index}}，当前共 ${{data.total_pairs}} 对样本。`);
        refreshStreams();
        await loadStatus();
      }} catch (error) {{
        setOpsStatus(`记录失败：${{error.message}}`, true);
      }} finally {{
        recordPairButton.disabled = false;
        computeButton.disabled = false;
      }}
    }}

    async function startCompute() {{
      computeButton.disabled = true;
      recordPairButton.disabled = true;
      setOpsStatus("正在启动标定计算任务，请稍候...");
      try {{
        const data = await postJson("/api/tasks/start", {{ task_id: "compute" }});
        const task = data.task || {{}};
        setOpsStatus(`已启动计算任务：${{task.label || task.task_id || "compute"}}`);
        await loadStatus();
      }} catch (error) {{
        setOpsStatus(`启动计算失败：${{error.message}}`, true);
      }} finally {{
        computeButton.disabled = false;
        recordPairButton.disabled = false;
      }}
    }}

    async function stopTask() {{
      stopTaskButton.disabled = true;
      setOpsStatus("正在停止后台任务...");
      try {{
        const data = await postJson("/api/tasks/stop", {{}});
        if (data.stopped) {{
          setOpsStatus("后台任务已停止。");
        }} else {{
          setOpsStatus(data.message || "当前没有运行中的任务。");
        }}
        await loadStatus();
      }} catch (error) {{
        setOpsStatus(`停止任务失败：${{error.message}}`, true);
      }} finally {{
        stopTaskButton.disabled = false;
      }}
    }}

    refreshRange.addEventListener("input", () => {{
      state.refreshMs = Number(refreshRange.value);
      refreshLabel.textContent = `${{state.refreshMs}} ms`;
      if (state.running) {{
        refreshStreams();
      }}
    }});

    toggleButton.addEventListener("click", () => {{
      state.running = !state.running;
      toggleButton.textContent = state.running ? "暂停刷新" : "恢复刷新";
      if (state.running) {{
        refreshStreams();
      }} else {{
        document.getElementById("leftStatus").textContent = "已暂停在当前画面";
        document.getElementById("rightStatus").textContent = "已暂停在当前画面";
      }}
    }});

    reloadButton.addEventListener("click", refreshStreams);
    recordPairButton.addEventListener("click", recordPair);
    computeButton.addEventListener("click", startCompute);
    stopTaskButton.addEventListener("click", stopTask);

    refreshStreams();
    loadStatus();
    setInterval(loadStatus, 3000);
  </script>
</body>
</html>
""".strip()
        return html_body.encode("utf-8")

    def depth_heatmap_preview_page(self) -> bytes:
        heatmap_topic = html.escape(self.depth_heatmap_topic)
        status_topic = html.escape(self.depth_heatmap_status_topic)
        html_body = f"""
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Depth Coordinate Heatmap</title>
  <style>
    :root {{
      --bg: #050814;
      --panel: rgba(9, 14, 28, 0.9);
      --panel-border: rgba(117, 215, 255, 0.18);
      --text: #edf5ff;
      --muted: #90a4c2;
      --accent: #6ee4ff;
      --accent-2: #ffb86d;
      --good: #88ffbf;
      --danger: #ff8f9e;
      --shadow: 0 24px 80px rgba(0, 0, 0, 0.46);
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      min-height: 100vh;
      color: var(--text);
      font-family: "Segoe UI", "PingFang SC", "Microsoft YaHei", sans-serif;
      background:
        radial-gradient(circle at 10% 10%, rgba(110, 228, 255, 0.22), transparent 28%),
        radial-gradient(circle at 88% 16%, rgba(255, 184, 109, 0.16), transparent 24%),
        linear-gradient(180deg, #091121 0%, var(--bg) 54%, #02050d 100%);
    }}
    .shell {{
      width: min(1520px, calc(100vw - 28px));
      margin: 20px auto 28px;
      display: grid;
      gap: 16px;
    }}
    .hero, .panel {{
      background: var(--panel);
      border: 1px solid var(--panel-border);
      border-radius: 24px;
      box-shadow: var(--shadow);
      backdrop-filter: blur(18px);
    }}
    .hero {{
      padding: 24px 28px;
      display: grid;
      grid-template-columns: 1.2fr 0.8fr;
      gap: 16px;
    }}
    .eyebrow {{
      color: var(--accent);
      text-transform: uppercase;
      letter-spacing: 0.26em;
      font-size: 12px;
      margin-bottom: 12px;
    }}
    h1 {{
      margin: 0 0 12px;
      font-size: clamp(30px, 4vw, 56px);
      line-height: 0.98;
      letter-spacing: -0.05em;
    }}
    .hero p {{
      margin: 0;
      color: var(--muted);
      max-width: 66ch;
      line-height: 1.68;
    }}
    .chips {{
      margin-top: 18px;
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
    }}
    .chip {{
      padding: 9px 14px;
      border-radius: 999px;
      border: 1px solid rgba(255, 255, 255, 0.07);
      background: rgba(255, 255, 255, 0.04);
      color: #d8f9ff;
      font-size: 13px;
    }}
    .hero-metrics {{
      display: grid;
      gap: 12px;
      align-content: start;
    }}
    .metric {{
      padding: 16px 18px;
      border-radius: 18px;
      background: rgba(4, 9, 20, 0.72);
      border: 1px solid rgba(255, 255, 255, 0.05);
    }}
    .metric strong {{
      display: block;
      color: var(--muted);
      font-size: 13px;
      margin-bottom: 8px;
    }}
    .metric span {{
      display: block;
      font-size: 26px;
      letter-spacing: -0.04em;
    }}
    .controls {{
      padding: 18px 20px;
      display: flex;
      flex-wrap: wrap;
      gap: 14px;
      align-items: center;
      justify-content: space-between;
    }}
    .controls-left {{
      display: flex;
      flex-wrap: wrap;
      gap: 12px;
      align-items: center;
    }}
    .controls label {{
      color: var(--muted);
      font-size: 13px;
    }}
    .controls input[type="range"] {{
      width: 180px;
      accent-color: var(--accent);
    }}
    .button {{
      border: 0;
      border-radius: 14px;
      padding: 11px 16px;
      font-weight: 700;
      cursor: pointer;
      color: #04101a;
      background: linear-gradient(135deg, var(--accent), #dcfbff);
    }}
    .button.secondary {{
      color: var(--text);
      background: rgba(255, 255, 255, 0.05);
      border: 1px solid rgba(255, 255, 255, 0.08);
    }}
    .layout {{
      display: grid;
      grid-template-columns: 1.3fr 0.7fr;
      gap: 16px;
    }}
    .canvas-panel {{
      padding: 16px;
      display: grid;
      gap: 12px;
    }}
    .canvas-shell {{
      position: relative;
      min-height: 680px;
      border-radius: 24px;
      overflow: hidden;
      border: 1px solid rgba(255,255,255,0.05);
      background:
        radial-gradient(circle at top, rgba(110,228,255,0.08), transparent 36%),
        linear-gradient(180deg, rgba(3, 8, 18, 0.92), rgba(1, 4, 10, 1));
    }}
    canvas {{
      width: 100%;
      height: 100%;
      display: block;
    }}
    .canvas-overlay {{
      position: absolute;
      inset: 0;
      pointer-events: none;
      display: flex;
      flex-direction: column;
      justify-content: space-between;
      padding: 16px;
    }}
    .badge {{
      align-self: flex-start;
      border-radius: 999px;
      padding: 8px 12px;
      font-size: 12px;
      background: rgba(10, 20, 40, 0.7);
      border: 1px solid rgba(111, 255, 181, 0.16);
      color: #dbfff0;
    }}
    .legend {{
      align-self: flex-end;
      width: 220px;
      padding: 10px 12px;
      border-radius: 16px;
      background: rgba(4, 10, 20, 0.74);
      border: 1px solid rgba(255, 255, 255, 0.05);
    }}
    .legend-bar {{
      height: 12px;
      border-radius: 999px;
      background: linear-gradient(90deg, #38bdf8 0%, #22d3ee 25%, #34d399 50%, #facc15 75%, #fb7185 100%);
      margin-bottom: 8px;
    }}
    .legend-labels {{
      display: flex;
      justify-content: space-between;
      color: var(--muted);
      font-size: 12px;
    }}
    .side {{
      display: grid;
      gap: 16px;
    }}
    .stats-grid {{
      padding: 18px;
      display: grid;
      gap: 12px;
      grid-template-columns: repeat(2, minmax(0, 1fr));
    }}
    .stat {{
      padding: 14px;
      border-radius: 18px;
      background: rgba(255,255,255,0.04);
      border: 1px solid rgba(255,255,255,0.05);
    }}
    .stat strong {{
      display: block;
      font-size: 12px;
      color: var(--muted);
      margin-bottom: 8px;
    }}
    .stat span {{
      font-size: 22px;
      letter-spacing: -0.03em;
    }}
    .detail-panel {{
      padding: 18px;
      display: grid;
      gap: 10px;
    }}
    .detail-row {{
      display: flex;
      justify-content: space-between;
      gap: 12px;
      color: var(--muted);
      font-size: 13px;
    }}
    .detail-row strong {{
      color: var(--text);
      font-weight: 600;
    }}
    .status-box {{
      padding: 16px 18px;
      border-radius: 18px;
      background: rgba(255,255,255,0.04);
      border: 1px solid rgba(255,255,255,0.05);
      color: var(--muted);
      font-size: 13px;
      line-height: 1.6;
      min-height: 94px;
    }}
    .status-box strong {{
      color: var(--text);
      display: block;
      margin-bottom: 8px;
    }}
    .tuning-panel {{
      padding: 18px;
      display: grid;
      gap: 12px;
    }}
    .tuning-grid {{
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 10px;
    }}
    .tuning-field {{
      display: grid;
      gap: 6px;
    }}
    .tuning-field label {{
      color: var(--muted);
      font-size: 12px;
    }}
    .tuning-field input[type="number"],
    .tuning-field input[type="text"] {{
      width: 100%;
      border-radius: 12px;
      border: 1px solid rgba(255,255,255,0.08);
      background: rgba(255,255,255,0.04);
      color: var(--text);
      padding: 10px 12px;
      font-size: 13px;
    }}
    .tuning-switches {{
      display: flex;
      flex-wrap: wrap;
      gap: 14px;
      color: var(--muted);
      font-size: 13px;
    }}
    .tuning-switches label {{
      display: inline-flex;
      align-items: center;
      gap: 8px;
    }}
    .tuning-actions {{
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
    }}
    .tuning-note {{
      color: var(--muted);
      font-size: 12px;
      line-height: 1.6;
    }}
    .footer {{
      padding: 18px 20px;
      display: flex;
      flex-wrap: wrap;
      gap: 12px 18px;
      justify-content: space-between;
      color: var(--muted);
      font-size: 13px;
    }}
    code {{
      color: #d5f9ff;
      font-family: Consolas, "SFMono-Regular", monospace;
    }}
    @media (max-width: 1180px) {{
      .hero {{ grid-template-columns: 1fr; }}
      .layout {{ grid-template-columns: 1fr; }}
      .canvas-shell {{ min-height: 520px; }}
    }}
  </style>
</head>
<body>
  <main class="shell">
    <section class="hero">
      <div>
        <div class="eyebrow">Depth Coordinate Preview</div>
        <h1>机器人基座 3D 热力图</h1>
        <p>这个独立调试页不再直接看原始深度图，而是显示已经转换到机器人基座系下的实时采样点。你可以在机器人运行时观察点云热分布、拖拽旋转视角，并用它辅助校正相机外参、TF 和工作空间边界。</p>
        <div class="chips">
          <div class="chip">热力图话题：<code>{heatmap_topic}</code></div>
          <div class="chip">状态话题：<code>{status_topic}</code></div>
          <div class="chip">入口：<code>/depth-heatmap-preview</code></div>
        </div>
      </div>
      <div class="hero-metrics">
        <div class="metric">
          <strong>坐标参考</strong>
          <span id="targetFrame">base_link</span>
        </div>
        <div class="metric">
          <strong>当前状态</strong>
          <span id="runtimeState">等待首帧</span>
        </div>
        <div class="metric">
          <strong>点数</strong>
          <span id="pointCountHero">--</span>
        </div>
      </div>
    </section>

    <section class="panel controls">
      <div class="controls-left">
        <label for="refreshRange">刷新间隔</label>
        <input id="refreshRange" type="range" min="300" max="2500" step="100" value="700" />
        <strong id="refreshLabel">700 ms</strong>
        <label for="pointRange">点尺寸</label>
        <input id="pointRange" type="range" min="1" max="8" step="1" value="3" />
      </div>
      <div class="controls-left">
        <button id="toggleButton" class="button secondary">暂停拉流</button>
        <button id="reloadButton" class="button">立刻刷新</button>
      </div>
    </section>

    <section class="layout">
      <article class="panel canvas-panel">
        <div class="canvas-shell" id="canvasShell">
          <canvas id="heatmapCanvas"></canvas>
          <div class="canvas-overlay">
            <div class="badge" id="badgeLabel">drag orbit / wheel zoom / auto rotate</div>
            <div class="legend">
              <div class="legend-bar"></div>
              <div class="legend-labels">
                <span>低</span>
                <span>高度热力</span>
                <span>高</span>
              </div>
            </div>
          </div>
        </div>
      </article>

      <aside class="side">
        <section class="panel stats-grid">
          <div class="stat"><strong>采样点</strong><span id="pointCount">--</span></div>
          <div class="stat"><strong>源坐标系</strong><span id="sourceFrame">--</span></div>
          <div class="stat"><strong>深度范围</strong><span id="depthRange">--</span></div>
          <div class="stat"><strong>高度范围</strong><span id="heightRange">--</span></div>
        </section>

        <section class="panel detail-panel">
          <div class="detail-row"><span>质心</span><strong id="centroidValue">--</strong></div>
          <div class="detail-row"><span>空间范围 X</span><strong id="boundsX">--</strong></div>
          <div class="detail-row"><span>空间范围 Y</span><strong id="boundsY">--</strong></div>
          <div class="detail-row"><span>空间范围 Z</span><strong id="boundsZ">--</strong></div>
          <div class="detail-row"><span>最后更新时间</span><strong id="stampValue">--</strong></div>
        </section>

        <section class="panel status-box">
          <strong>运行状态</strong>
          <div id="statusText">等待热力图消息...</div>
        </section>

        <section class="panel tuning-panel">
          <strong>在线校正</strong>
          <div class="tuning-switches">
            <label><input id="useTfTransform" type="checkbox" checked /> 优先使用 TF</label>
            <label><input id="useManualFallback" type="checkbox" checked /> TF 失败回退手动外参</label>
          </div>
          <div class="tuning-grid">
            <div class="tuning-field"><label for="txInput">tx (m)</label><input id="txInput" type="number" step="0.01" value="0" /></div>
            <div class="tuning-field"><label for="tyInput">ty (m)</label><input id="tyInput" type="number" step="0.01" value="0" /></div>
            <div class="tuning-field"><label for="tzInput">tz (m)</label><input id="tzInput" type="number" step="0.01" value="0" /></div>
            <div class="tuning-field"><label for="rollInput">roll (deg)</label><input id="rollInput" type="number" step="0.5" value="0" /></div>
            <div class="tuning-field"><label for="pitchInput">pitch (deg)</label><input id="pitchInput" type="number" step="0.5" value="0" /></div>
            <div class="tuning-field"><label for="yawInput">yaw (deg)</label><input id="yawInput" type="number" step="0.5" value="0" /></div>
            <div class="tuning-field"><label for="sampleStepInput">sample_step</label><input id="sampleStepInput" type="number" min="1" step="1" value="2" /></div>
            <div class="tuning-field"><label for="maxPointsInput">max_points</label><input id="maxPointsInput" type="number" min="32" step="64" value="4000" /></div>
          </div>
          <div class="tuning-field">
            <label for="sourceFrameOverride">source_frame_override</label>
            <input id="sourceFrameOverride" type="text" value="" placeholder="默认留空，沿用深度消息 frame_id" />
          </div>
          <div class="tuning-actions">
            <button id="applyTuningButton" class="button">应用校正</button>
            <button id="reloadTuningButton" class="button secondary">读取当前参数</button>
          </div>
          <div class="tuning-note" id="tuningStatus">默认优先使用 TF；若当前机器人还没补齐 `base_link -> left_camera_optical_frame`，可临时关闭 TF 并用手动外参做现场校正。</div>
        </section>
      </aside>
    </section>

    <section class="panel footer">
      <span>接口：<code>/api/depth/heatmap?force=1</code></span>
      <span>数据：<code>xyzhd = [x, y, z, heat, distance]</code></span>
      <span>说明：颜色按基座系 <code>z</code> 高度映射，适合观察工作面与校正偏移。</span>
    </section>
  </main>

  <script>
    const canvas = document.getElementById("heatmapCanvas");
    const shell = document.getElementById("canvasShell");
    const ctx = canvas.getContext("2d");
    const state = {{
      running: true,
      refreshMs: 700,
      pointRadius: 3,
      yaw: -0.75,
      pitch: 0.58,
      zoom: 1.0,
      dragging: false,
      lastX: 0,
      lastY: 0,
      payload: null,
      lastFetch: 0,
    }};

    const fields = {{
      runtimeState: document.getElementById("runtimeState"),
      targetFrame: document.getElementById("targetFrame"),
      pointCountHero: document.getElementById("pointCountHero"),
      pointCount: document.getElementById("pointCount"),
      sourceFrame: document.getElementById("sourceFrame"),
      depthRange: document.getElementById("depthRange"),
      heightRange: document.getElementById("heightRange"),
      centroidValue: document.getElementById("centroidValue"),
      boundsX: document.getElementById("boundsX"),
      boundsY: document.getElementById("boundsY"),
      boundsZ: document.getElementById("boundsZ"),
      stampValue: document.getElementById("stampValue"),
      statusText: document.getElementById("statusText"),
      badgeLabel: document.getElementById("badgeLabel"),
      tuningStatus: document.getElementById("tuningStatus"),
    }};

    function resizeCanvas() {{
      const rect = shell.getBoundingClientRect();
      const scale = window.devicePixelRatio || 1;
      canvas.width = Math.floor(rect.width * scale);
      canvas.height = Math.floor(rect.height * scale);
      canvas.style.width = `${{rect.width}}px`;
      canvas.style.height = `${{rect.height}}px`;
      ctx.setTransform(scale, 0, 0, scale, 0, 0);
    }}

    function fmtRange(pair) {{
      if (!pair || pair.length < 2) return "--";
      return `${{pair[0].toFixed(3)}} ~ ${{pair[1].toFixed(3)}} m`;
    }}

    function fmtVec(values) {{
      if (!values || values.length < 3) return "--";
      return values.map((v) => v.toFixed(3)).join(", ");
    }}

    function heatColor(t, alpha = 1) {{
      const stops = [
        [0.0, [56, 189, 248]],
        [0.25, [34, 211, 238]],
        [0.5, [52, 211, 153]],
        [0.75, [250, 204, 21]],
        [1.0, [251, 113, 133]],
      ];
      let left = stops[0];
      let right = stops[stops.length - 1];
      for (let i = 0; i < stops.length - 1; i += 1) {{
        if (t >= stops[i][0] && t <= stops[i + 1][0]) {{
          left = stops[i];
          right = stops[i + 1];
          break;
        }}
      }}
      const span = Math.max(0.0001, right[0] - left[0]);
      const p = Math.min(1, Math.max(0, (t - left[0]) / span));
      const rgb = left[1].map((value, index) => Math.round(value + (right[1][index] - value) * p));
      return `rgba(${{rgb[0]}}, ${{rgb[1]}}, ${{rgb[2]}}, ${{alpha}})`;
    }}

    function rotatePoint(point) {{
      const cy = Math.cos(state.yaw);
      const sy = Math.sin(state.yaw);
      const cp = Math.cos(state.pitch);
      const sp = Math.sin(state.pitch);
      const x1 = point.x * cy - point.y * sy;
      const y1 = point.x * sy + point.y * cy;
      const z1 = point.z;
      const y2 = y1 * cp - z1 * sp;
      const z2 = y1 * sp + z1 * cp;
      return {{ x: x1, y: y2, z: z2 }};
    }}

    function renderAxes(center, scale) {{
      const axes = [
        {{ end: {{ x: 0.18, y: 0, z: 0 }}, color: "rgba(110,228,255,0.95)", label: "X" }},
        {{ end: {{ x: 0, y: 0.18, z: 0 }}, color: "rgba(255,184,109,0.95)", label: "Y" }},
        {{ end: {{ x: 0, y: 0, z: 0.18 }}, color: "rgba(136,255,191,0.95)", label: "Z" }},
      ];
      ctx.save();
      ctx.font = "12px Consolas";
      axes.forEach((axis) => {{
        const rotated = rotatePoint(axis.end);
        ctx.strokeStyle = axis.color;
        ctx.lineWidth = 1.6;
        ctx.beginPath();
        ctx.moveTo(center.x, center.y);
        ctx.lineTo(center.x + rotated.x * scale, center.y - rotated.y * scale);
        ctx.stroke();
        ctx.fillStyle = axis.color;
        ctx.fillText(axis.label, center.x + rotated.x * scale + 6, center.y - rotated.y * scale - 6);
      }});
      ctx.restore();
    }}

    function renderScene() {{
      const width = shell.clientWidth;
      const height = shell.clientHeight;
      ctx.clearRect(0, 0, width, height);

      const gradient = ctx.createLinearGradient(0, 0, 0, height);
      gradient.addColorStop(0, "rgba(13,22,40,0.94)");
      gradient.addColorStop(1, "rgba(2,6,14,1)");
      ctx.fillStyle = gradient;
      ctx.fillRect(0, 0, width, height);

      const center = {{ x: width * 0.5, y: height * 0.56 }};
      const payload = state.payload;
      if (!payload || !payload.points || !payload.points.length) {{
        ctx.fillStyle = "rgba(144,164,194,0.88)";
        ctx.font = "18px Segoe UI";
        ctx.fillText("等待热力图数据...", 36, 42);
        renderAxes(center, Math.min(width, height) * 0.28 * state.zoom);
        return;
      }}

      const bounds = payload.stats?.bounds_m;
      const minBounds = bounds?.min || [-0.3, -0.3, -0.1];
      const maxBounds = bounds?.max || [0.3, 0.3, 0.4];
      const span = Math.max(
        0.25,
        maxBounds[0] - minBounds[0],
        maxBounds[1] - minBounds[1],
        maxBounds[2] - minBounds[2],
      );
      const scale = Math.min(width, height) * 0.46 * state.zoom / span;
      const centroid = payload.stats?.centroid_m || [0, 0, 0];
      const projected = payload.points.map((item) => {{
        const rotated = rotatePoint({{
          x: item[0] - centroid[0],
          y: item[1] - centroid[1],
          z: item[2] - centroid[2],
        }});
        return {{
          sx: center.x + rotated.x * scale,
          sy: center.y - rotated.y * scale,
          depth: rotated.z,
          heat: item[3],
          radius: state.pointRadius + item[3] * 2.2,
        }};
      }});

      projected.sort((a, b) => a.depth - b.depth);

      ctx.save();
      ctx.globalCompositeOperation = "lighter";
      projected.forEach((item) => {{
        const glow = ctx.createRadialGradient(item.sx, item.sy, 0, item.sx, item.sy, item.radius * 4.5);
        glow.addColorStop(0, heatColor(item.heat, 0.92));
        glow.addColorStop(1, heatColor(item.heat, 0));
        ctx.fillStyle = glow;
        ctx.beginPath();
        ctx.arc(item.sx, item.sy, item.radius * 4.5, 0, Math.PI * 2);
        ctx.fill();
      }});
      ctx.restore();

      projected.forEach((item) => {{
        ctx.fillStyle = heatColor(item.heat, 0.95);
        ctx.beginPath();
        ctx.arc(item.sx, item.sy, item.radius, 0, Math.PI * 2);
        ctx.fill();
      }});

      renderAxes(center, Math.min(width, height) * 0.24 * state.zoom);
    }}

    function tuningPayloadFromInputs() {{
      return {{
        use_tf_transform: document.getElementById("useTfTransform").checked,
        use_manual_transform_fallback: document.getElementById("useManualFallback").checked,
        manual_translation_m: [
          Number(document.getElementById("txInput").value || 0),
          Number(document.getElementById("tyInput").value || 0),
          Number(document.getElementById("tzInput").value || 0),
        ],
        manual_rpy_deg: [
          Number(document.getElementById("rollInput").value || 0),
          Number(document.getElementById("pitchInput").value || 0),
          Number(document.getElementById("yawInput").value || 0),
        ],
        source_frame_override: document.getElementById("sourceFrameOverride").value || "",
        sample_step: Math.max(1, Number(document.getElementById("sampleStepInput").value || 2)),
        max_points: Math.max(32, Number(document.getElementById("maxPointsInput").value || 4000)),
      }};
    }}

    function fillTuningForm(statePayload) {{
      if (!statePayload) return;
      document.getElementById("useTfTransform").checked = !!statePayload.use_tf_transform;
      document.getElementById("useManualFallback").checked = !!statePayload.use_manual_transform_fallback;
      const translation = statePayload.manual_translation_m || [0, 0, 0];
      const rotation = statePayload.manual_rpy_deg || [0, 0, 0];
      document.getElementById("txInput").value = translation[0] ?? 0;
      document.getElementById("tyInput").value = translation[1] ?? 0;
      document.getElementById("tzInput").value = translation[2] ?? 0;
      document.getElementById("rollInput").value = rotation[0] ?? 0;
      document.getElementById("pitchInput").value = rotation[1] ?? 0;
      document.getElementById("yawInput").value = rotation[2] ?? 0;
      document.getElementById("sourceFrameOverride").value = statePayload.source_frame_override || "";
      document.getElementById("sampleStepInput").value = statePayload.sample_step ?? 2;
      document.getElementById("maxPointsInput").value = statePayload.max_points ?? 4000;
    }}

    async function loadTuningConfig() {{
      try {{
        const response = await fetch("/api/depth/coordinate-config", {{ cache: "no-store" }});
        const data = await response.json();
        fillTuningForm(data);
        fields.tuningStatus.textContent =
          `当前配置已同步：use_tf=${{data.use_tf_transform ? "true" : "false"}}，manual_fallback=${{data.use_manual_transform_fallback ? "true" : "false"}}，sample_step=${{data.sample_step}}，max_points=${{data.max_points}}`;
      }} catch (error) {{
        fields.tuningStatus.textContent = `读取校正参数失败：${{error.message}}`;
      }}
    }}

    async function applyTuningConfig() {{
      const payload = tuningPayloadFromInputs();
      fields.tuningStatus.textContent = "正在把校正参数下发到 depth_coordinate_node...";
      try {{
        const response = await fetch("/api/depth/coordinate-config", {{
          method: "POST",
          headers: {{ "Content-Type": "application/json" }},
          body: JSON.stringify(payload),
        }});
        const data = await response.json();
        if (!response.ok || data.ok === false) {{
          throw new Error(data.error || "校正参数下发失败");
        }}
        fillTuningForm(data.state);
        fields.tuningStatus.textContent =
          `校正参数已应用：transform_fallback=${{data.state.use_manual_transform_fallback ? "on" : "off"}}，translation=${{data.state.manual_translation_m.join(", ")}}，rpy=${{data.state.manual_rpy_deg.join(", ")}}，sample_step=${{data.state.sample_step}}，max_points=${{data.state.max_points}}`;
        loadHeatmap(true);
      }} catch (error) {{
        fields.tuningStatus.textContent = `校正参数应用失败：${{error.message}}`;
      }}
    }}

    async function loadHeatmap(force = false) {{
      if (!state.running && !force) return;
      try {{
        const url = `/api/depth/heatmap?timeout=2.4&force=${{force ? 1 : 0}}&t=${{Date.now()}}`;
        const response = await fetch(url, {{ cache: "no-store" }});
        const data = await response.json();
        if (!response.ok || data.ok === false || !data.points) {{
          fields.runtimeState.textContent = "等待数据";
          fields.statusText.textContent = data.error || "热力图数据暂不可用";
          return;
        }}
        state.payload = data;
        state.lastFetch = Date.now();
        fields.runtimeState.textContent = "在线";
        fields.targetFrame.textContent = data.target_frame || "--";
        fields.pointCountHero.textContent = String(data.point_count || 0);
        fields.pointCount.textContent = String(data.point_count || 0);
        fields.sourceFrame.textContent = data.source_frame || "--";
        fields.depthRange.textContent = fmtRange(data.stats?.depth_range_m);
        fields.heightRange.textContent = fmtRange(data.stats?.height_range_m);
        fields.centroidValue.textContent = fmtVec(data.stats?.centroid_m);
        fields.boundsX.textContent = fmtRange(data.stats?.bounds_m ? [data.stats.bounds_m.min[0], data.stats.bounds_m.max[0]] : null);
        fields.boundsY.textContent = fmtRange(data.stats?.bounds_m ? [data.stats.bounds_m.min[1], data.stats.bounds_m.max[1]] : null);
        fields.boundsZ.textContent = fmtRange(data.stats?.bounds_m ? [data.stats.bounds_m.min[2], data.stats.bounds_m.max[2]] : null);
        fields.stampValue.textContent = `${{data.stamp_sec || 0}}.${{String(data.stamp_nanosec || 0).padStart(9, "0")}}`;
        fields.statusText.textContent =
          `源坐标系 ${{data.source_frame}} -> 目标坐标系 ${{data.target_frame}}。当前载荷 ${{data.point_count}} 点，颜色按基座系 z 高度映射，当前变换来源：${{data.transform_source || "unknown"}}。`;
      }} catch (error) {{
        fields.runtimeState.textContent = "错误";
        fields.statusText.textContent = `拉取热力图失败：${{error.message}}`;
      }}
    }}

    function animate() {{
      if (!state.dragging && state.running) {{
        state.yaw += 0.0022;
      }}
      renderScene();
      requestAnimationFrame(animate);
    }}

    document.getElementById("refreshRange").addEventListener("input", (event) => {{
      state.refreshMs = Number(event.target.value);
      document.getElementById("refreshLabel").textContent = `${{state.refreshMs}} ms`;
    }});
    document.getElementById("pointRange").addEventListener("input", (event) => {{
      state.pointRadius = Number(event.target.value);
    }});
    document.getElementById("toggleButton").addEventListener("click", (event) => {{
      state.running = !state.running;
      event.target.textContent = state.running ? "暂停拉流" : "恢复拉流";
      fields.badgeLabel.textContent = state.running ? "drag orbit / wheel zoom / auto rotate" : "paused / drag orbit / wheel zoom";
      if (state.running) {{
        loadHeatmap(true);
      }}
    }});
    document.getElementById("reloadButton").addEventListener("click", () => loadHeatmap(true));
    document.getElementById("applyTuningButton").addEventListener("click", applyTuningConfig);
    document.getElementById("reloadTuningButton").addEventListener("click", loadTuningConfig);

    shell.addEventListener("pointerdown", (event) => {{
      state.dragging = true;
      state.lastX = event.clientX;
      state.lastY = event.clientY;
      shell.setPointerCapture(event.pointerId);
    }});
    shell.addEventListener("pointermove", (event) => {{
      if (!state.dragging) return;
      const dx = event.clientX - state.lastX;
      const dy = event.clientY - state.lastY;
      state.lastX = event.clientX;
      state.lastY = event.clientY;
      state.yaw += dx * 0.01;
      state.pitch = Math.max(-1.25, Math.min(1.25, state.pitch + dy * 0.01));
    }});
    shell.addEventListener("pointerup", () => {{
      state.dragging = false;
    }});
    shell.addEventListener("pointerleave", () => {{
      state.dragging = false;
    }});
    shell.addEventListener("wheel", (event) => {{
      event.preventDefault();
      state.zoom = Math.max(0.35, Math.min(2.8, state.zoom - event.deltaY * 0.001));
    }}, {{ passive: false }});

    resizeCanvas();
    window.addEventListener("resize", resizeCanvas);
    setInterval(() => loadHeatmap(false), state.refreshMs);
    loadTuningConfig();
    loadHeatmap(true);
    animate();
  </script>
</body>
</html>
""".strip()
        return html_body.encode("utf-8")

    def placeholder_image(self, message: str) -> bytes:
        safe = (
            message.replace("&", "&amp;")
            .replace("<", "&lt;")
            .replace(">", "&gt;")
            .replace('"', "&quot;")
        )
        svg = f"""
<svg xmlns="http://www.w3.org/2000/svg" width="960" height="540" viewBox="0 0 960 540">
  <rect width="960" height="540" fill="#0c1016"/>
  <rect x="24" y="24" width="912" height="492" rx="24" fill="#10161f" stroke="#55c1ff" stroke-opacity="0.35"/>
  <text x="64" y="112" fill="#7fe6ff" font-size="28" font-family="monospace">admin_gui / Deyes Vision</text>
  <text x="64" y="170" fill="#f5f1e8" font-size="18" font-family="monospace">{safe}</text>
  <text x="64" y="220" fill="#b5ac9f" font-size="16" font-family="monospace">请确认 ROS 图像话题已发布，或先执行 1280 预检。</text>
</svg>
""".strip()
        return svg.encode("utf-8")


CONTEXT = AdminContext()


class AdminHandler(BaseHTTPRequestHandler):
    server_version = "AdminGUIBackend/0.1"

    def log_message(self, format: str, *args: Any) -> None:
        return

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)
        if parsed.path == "/api/status":
            self.respond_json(CONTEXT.status())
            return
        if parsed.path == "/api/metrics":
            self.respond_json(CONTEXT.metrics())
            return
        if parsed.path == "/api/logs":
            path_value = query.get("path", [""])[0]
            log_path = Path(path_value) if path_value else None
            if log_path is None:
                task = CONTEXT.task_manager.snapshot()
                log_path = Path(task["log_path"]) if task.get("log_path") else None
            self.respond_json({"log": CONTEXT.tail_log(log_path)})
            return
        if parsed.path == "/api/calibration/form":
            self.respond_json(CONTEXT.load_form())
            return
        if parsed.path == "/api/calibration/wizard":
            self.respond_json(CONTEXT.build_wizard())
            return
        if parsed.path == "/api/vision/mode":
            self.respond_json(CONTEXT.load_vision_mode())
            return
        if parsed.path == "/api/depth/coordinate-config":
            self.respond_json(CONTEXT.load_depth_coordinate_tuning())
            return
        if parsed.path == "/stereo-preview":
            self.respond_binary(CONTEXT.stereo_preview_page(), "text/html; charset=utf-8")
            return
        if parsed.path == "/depth-heatmap-preview":
            self.respond_binary(CONTEXT.depth_heatmap_preview_page(), "text/html; charset=utf-8")
            return
        if parsed.path == "/api/vision/left":
            force_refresh = query.get("force", ["0"])[0] in {"1", "true", "yes"}
            width = int(query.get("width", ["0"])[0] or "0")
            quality = int(query.get("quality", ["85"])[0] or "85")
            timeout_sec = float(query.get("timeout", ["4"])[0] or "4")
            self.respond_binary(
                *CONTEXT.vision_response(
                    "left",
                    force_refresh=force_refresh,
                    timeout_sec=timeout_sec,
                    max_width=width,
                    jpeg_quality=quality,
                )
            )
            return
        if parsed.path == "/api/vision/left/stream":
            width = int(query.get("width", ["0"])[0] or "0")
            quality = int(query.get("quality", ["85"])[0] or "85")
            timeout_sec = float(query.get("timeout", ["4"])[0] or "4")
            interval_sec = float(query.get("interval", ["1.8"])[0] or "1.8")
            self.stream_mjpeg("left", width, quality, timeout_sec, interval_sec)
            return
        if parsed.path == "/api/vision/right":
            force_refresh = query.get("force", ["0"])[0] in {"1", "true", "yes"}
            width = int(query.get("width", ["0"])[0] or "0")
            quality = int(query.get("quality", ["85"])[0] or "85")
            timeout_sec = float(query.get("timeout", ["4"])[0] or "4")
            self.respond_binary(
                *CONTEXT.vision_response(
                    "right",
                    force_refresh=force_refresh,
                    timeout_sec=timeout_sec,
                    max_width=width,
                    jpeg_quality=quality,
                )
            )
            return
        if parsed.path == "/api/vision/right/stream":
            width = int(query.get("width", ["0"])[0] or "0")
            quality = int(query.get("quality", ["85"])[0] or "85")
            timeout_sec = float(query.get("timeout", ["4"])[0] or "4")
            interval_sec = float(query.get("interval", ["1.8"])[0] or "1.8")
            self.stream_mjpeg("right", width, quality, timeout_sec, interval_sec)
            return
        if parsed.path == "/api/depth/heatmap":
            force_refresh = query.get("force", ["0"])[0] in {"1", "true", "yes"}
            timeout_sec = float(query.get("timeout", ["2.5"])[0] or "2.5")
            self.respond_binary(
                *CONTEXT.depth_heatmap_response(
                    force_refresh=force_refresh,
                    timeout_sec=timeout_sec,
                )
            )
            return
        self.serve_static(parsed.path)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/vision/save-pair":
            payload = self.read_json()
            timeout_sec = float(payload.get("timeout", 4.0) or 4.0)
            try:
                self.respond_json(CONTEXT.save_current_pair(timeout_sec=max(1.0, timeout_sec)))
            except RuntimeError as exc:
                self.respond_json({"ok": False, "error": str(exc)}, status=HTTPStatus.CONFLICT)
            return
        if parsed.path == "/api/tasks/start":
            payload = self.read_json()
            task_id = payload.get("task_id", "")
            item = CONTEXT.command_by_id(task_id)
            if item is None:
                self.respond_json({"ok": False, "error": "未知任务。"}, status=HTTPStatus.BAD_REQUEST)
                return
            try:
                data = CONTEXT.task_manager.start(item["id"], item["label"], item["command"])
                self.respond_json({"ok": True, "task": data})
            except RuntimeError as exc:
                self.respond_json({"ok": False, "error": str(exc)}, status=HTTPStatus.CONFLICT)
            return
        if parsed.path == "/api/tasks/stop":
            self.respond_json(CONTEXT.task_manager.stop())
            return
        if parsed.path == "/api/calibration/form":
            payload = self.read_json()
            self.respond_json(CONTEXT.save_form(payload))
            return
        if parsed.path == "/api/calibration/wizard":
            payload = self.read_json()
            self.respond_json(CONTEXT.save_wizard_state(payload))
            return
        if parsed.path == "/api/vision/mode":
            payload = self.read_json()
            try:
                self.respond_json(CONTEXT.save_vision_mode(payload))
            except ValueError as exc:
                self.respond_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
            return
        if parsed.path == "/api/depth/coordinate-config":
            payload = self.read_json()
            try:
                result = CONTEXT.save_depth_coordinate_tuning(payload)
                status = HTTPStatus.OK if result.get("ok", False) else HTTPStatus.CONFLICT
                self.respond_json(result, status=status)
            except (ValueError, RuntimeError) as exc:
                self.respond_json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
            return
        self.respond_json({"ok": False, "error": "Not found"}, status=HTTPStatus.NOT_FOUND)

    def read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        raw = self.rfile.read(length).decode("utf-8")
        return json.loads(raw) if raw else {}

    def serve_static(self, path: str) -> None:
        relative = "index.html" if path in ("", "/") else path.lstrip("/")
        candidate = FRONTEND_DIST / relative
        if not candidate.exists() or not candidate.is_file():
            candidate = FRONTEND_DIST / "index.html"
        if not candidate.exists():
            self.send_error(HTTPStatus.NOT_FOUND, "Frontend dist not found")
            return
        body = candidate.read_bytes()
        mime, _ = mimetypes.guess_type(str(candidate))
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", mime or "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def respond_json(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def respond_binary(self, body: bytes, content_type: str, status: HTTPStatus = HTTPStatus.OK) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def stream_mjpeg(
        self,
        side: str,
        width: int,
        quality: int,
        timeout_sec: float,
        interval_sec: float,
    ) -> None:
        boundary = "frame"
        cache_path = VISION_CACHE[side]
        self.send_response(HTTPStatus.OK)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
        self.end_headers()

        last_mtime = 0.0
        frame_interval = max(0.6, interval_sec)
        try:
            while True:
                CONTEXT.request_vision_refresh(
                    side,
                    timeout_sec=max(1.0, timeout_sec),
                    max_width=max(0, width),
                    jpeg_quality=max(25, min(quality, 95)),
                )
                if cache_path.exists():
                    mtime = cache_path.stat().st_mtime
                    if mtime >= last_mtime:
                        frame = cache_path.read_bytes()
                        self.wfile.write(f"--{boundary}\r\n".encode("ascii"))
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                        last_mtime = mtime
                time.sleep(frame_interval)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            return


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def main() -> None:
    parser = argparse.ArgumentParser(description="admin_gui Python backend")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    server = ReusableThreadingHTTPServer((args.host, args.port), AdminHandler)
    print(f"[admin_gui backend] serving on http://{args.host}:{args.port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
