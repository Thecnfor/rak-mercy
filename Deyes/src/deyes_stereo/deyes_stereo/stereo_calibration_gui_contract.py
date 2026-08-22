"""ROS/Qt-independent state used by the stereo calibration desktop tool."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from .stereo_calibration_contract import (
    CALIBRATION_SIZE,
    DEFAULT_BOARD_INNER_CORNERS,
    MAX_PAIR_SKEW_MS,
    MAX_SAMPLES,
    MIN_SAMPLES,
)


@dataclass(frozen=True)
class CaptureSettings:
    board_cols: int = DEFAULT_BOARD_INNER_CORNERS[0]
    board_rows: int = DEFAULT_BOARD_INNER_CORNERS[1]
    square_size_m: float = 0.020
    target_samples: int = 50
    min_blur_score: float = 80.0
    stable_hold_s: float = 0.8
    max_corner_motion_px: float = 0.8

    def errors(self) -> tuple[str, ...]:
        errors: list[str] = []
        if self.board_cols < 4 or self.board_rows < 4:
            errors.append("棋盘内角点必须至少为 4x4")
        if not 0.001 <= self.square_size_m <= 0.2:
            errors.append("格长必须在 1–200 mm 之间")
        if not MIN_SAMPLES <= self.target_samples <= MAX_SAMPLES:
            errors.append("样本数必须在 40–60 之间")
        if self.min_blur_score <= 0:
            errors.append("清晰度阈值必须为正数")
        if self.stable_hold_s < 0.2 or self.max_corner_motion_px <= 0:
            errors.append("稳定判定参数无效")
        return tuple(errors)


class StabilityTracker:
    """Require small corner motion continuously before a pair may be saved."""

    def __init__(self, hold_s: float, max_motion_px: float) -> None:
        self.hold_s = float(hold_s)
        self.max_motion_px = float(max_motion_px)
        self.reset()

    def reset(self) -> None:
        self._previous: np.ndarray | None = None
        self._stable_since: float | None = None

    def update(self, corners: np.ndarray, now_s: float) -> tuple[bool, float, float]:
        current = np.asarray(corners, dtype=np.float32).reshape(-1, 2)
        if self._previous is None or self._previous.shape != current.shape:
            self._previous = current.copy()
            self._stable_since = None
            return False, float("inf"), 0.0
        motion = float(np.linalg.norm(current - self._previous, axis=1).mean())
        self._previous = current.copy()
        if motion > self.max_motion_px:
            self._stable_since = None
            return False, motion, 0.0
        if self._stable_since is None:
            self._stable_since = float(now_s)
        elapsed = max(0.0, float(now_s) - self._stable_since)
        return elapsed >= self.hold_s, motion, elapsed


def session_directory(root: str | Path, now: datetime | None = None) -> Path:
    timestamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
    return Path(root).expanduser() / f"stereo_{timestamp}"


def make_manifest(
    *,
    settings: CaptureSettings,
    samples: Sequence[dict[str, Any]],
    reject_counts: dict[str, int],
    coverage: set[tuple[int, int]],
    left_topic: str,
    right_topic: str,
    revision: str,
) -> dict[str, Any]:
    return {
        "schema_version": 2,
        "source": "ros_topics",
        "board_inner_corners": [settings.board_cols, settings.board_rows],
        "square_size_m": settings.square_size_m,
        "resolution": list(CALIBRATION_SIZE),
        "left_topic": left_topic,
        "right_topic": right_topic,
        "max_pair_skew_ms": MAX_PAIR_SKEW_MS,
        "min_blur_score": settings.min_blur_score,
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "samples": list(samples),
        "reject_counts": dict(sorted(reject_counts.items())),
        "coverage_cells": sorted([list(cell) for cell in coverage]),
        "capture_command": ["stereo_calibration_gui"],
        "source_revision": revision,
    }
