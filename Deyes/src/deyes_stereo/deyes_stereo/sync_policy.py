from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Deque, Dict, List, Optional


class MonitorSeverity(IntEnum):
    OK = 0
    WARN = 1
    ERROR = 2


@dataclass
class MonitorParams:
    expected_min_rate_hz: float = 10.0
    image_timeout_sec: float = 0.5
    camera_info_timeout_sec: float = 1.0
    hard_sync_max_ms: float = 3.0
    soft_sync_max_ms: float = 10.0
    allow_soft_sync: bool = False
    drop_gap_factor: float = 1.8
    stale_after_missed_frames: float = 3.0
    fps_window_size: int = 30


@dataclass
class StreamSnapshot:
    stamp_ns: int
    width: int
    height: int
    frame_id: str
    receipt_time_sec: float
    encoding: str = ""


@dataclass
class StreamState:
    latest: Optional[StreamSnapshot] = None
    receipt_history: Deque[float] = field(default_factory=lambda: deque(maxlen=30))
    non_monotonic_stamp_count: int = 0
    large_gap_count: int = 0
    message_count: int = 0

    def update(self, snapshot: StreamSnapshot, params: MonitorParams) -> None:
        if self.latest is not None and snapshot.stamp_ns <= self.latest.stamp_ns:
            self.non_monotonic_stamp_count += 1

        if self.receipt_history:
            gap_sec = snapshot.receipt_time_sec - self.receipt_history[-1]
            expected_period = (
                1.0 / params.expected_min_rate_hz if params.expected_min_rate_hz > 0.0 else 0.0
            )
            if expected_period > 0.0 and gap_sec > expected_period * params.drop_gap_factor:
                self.large_gap_count += 1

        self.latest = snapshot
        self.receipt_history.append(snapshot.receipt_time_sec)
        self.message_count += 1

    def rate_hz(self) -> float:
        if len(self.receipt_history) < 2:
            return 0.0
        elapsed = self.receipt_history[-1] - self.receipt_history[0]
        if elapsed <= 0.0:
            return 0.0
        return (len(self.receipt_history) - 1) / elapsed


@dataclass
class MonitorEvaluation:
    severity: MonitorSeverity
    gate_ok: bool
    summary: str
    reasons: List[str]
    metrics: Dict[str, str]


class SyncHealthMonitor:
    def __init__(self, params: Optional[MonitorParams] = None) -> None:
        self.params = params or MonitorParams()
        self.image_streams = {"left": StreamState(), "right": StreamState()}
        self.info_streams = {"left": StreamState(), "right": StreamState()}

    def update_image(
        self,
        side: str,
        *,
        stamp_ns: int,
        width: int,
        height: int,
        frame_id: str,
        encoding: str,
        receipt_time_sec: float,
    ) -> None:
        snapshot = StreamSnapshot(
            stamp_ns=stamp_ns,
            width=width,
            height=height,
            frame_id=frame_id,
            encoding=encoding,
            receipt_time_sec=receipt_time_sec,
        )
        self.image_streams[side].update(snapshot, self.params)

    def update_camera_info(
        self,
        side: str,
        *,
        stamp_ns: int,
        width: int,
        height: int,
        frame_id: str,
        receipt_time_sec: float,
    ) -> None:
        snapshot = StreamSnapshot(
            stamp_ns=stamp_ns,
            width=width,
            height=height,
            frame_id=frame_id,
            receipt_time_sec=receipt_time_sec,
        )
        self.info_streams[side].update(snapshot, self.params)

    def evaluate(self, now_sec: float) -> MonitorEvaluation:
        reasons: List[str] = []
        metrics: Dict[str, str] = {}
        severity = MonitorSeverity.OK
        gate_ok = True

        for side in ("left", "right"):
            image_state = self.image_streams[side]
            info_state = self.info_streams[side]

            image_fps = image_state.rate_hz()
            info_fps = info_state.rate_hz()
            metrics[f"{side}_image_fps"] = f"{image_fps:.2f}"
            metrics[f"{side}_camera_info_fps"] = f"{info_fps:.2f}"
            metrics[f"{side}_image_large_gap_count"] = str(image_state.large_gap_count)
            metrics[f"{side}_image_non_monotonic_stamp_count"] = str(
                image_state.non_monotonic_stamp_count
            )

            if image_state.latest is None:
                gate_ok = False
                severity = MonitorSeverity.ERROR
                reasons.append(f"{side}_image_missing")
                continue

            if info_state.latest is None:
                gate_ok = False
                severity = MonitorSeverity.ERROR
                reasons.append(f"{side}_camera_info_missing")
                continue

            if image_fps < self.params.expected_min_rate_hz:
                severity = max(severity, MonitorSeverity.WARN)
                reasons.append(f"{side}_image_low_rate")

            if info_fps == 0.0:
                severity = max(severity, MonitorSeverity.WARN)
                reasons.append(f"{side}_camera_info_low_rate")

            expected_period = (
                1.0 / self.params.expected_min_rate_hz if self.params.expected_min_rate_hz > 0.0 else 0.0
            )
            stale_limit = max(
                self.params.image_timeout_sec,
                expected_period * self.params.stale_after_missed_frames,
            )

            if now_sec - image_state.latest.receipt_time_sec > stale_limit:
                gate_ok = False
                severity = MonitorSeverity.ERROR
                reasons.append(f"{side}_image_stale")

            if now_sec - info_state.latest.receipt_time_sec > self.params.camera_info_timeout_sec:
                gate_ok = False
                severity = MonitorSeverity.ERROR
                reasons.append(f"{side}_camera_info_stale")

            if (
                image_state.latest.width != info_state.latest.width
                or image_state.latest.height != info_state.latest.height
            ):
                gate_ok = False
                severity = MonitorSeverity.ERROR
                reasons.append(f"{side}_camera_info_size_mismatch")

            if not image_state.latest.frame_id:
                gate_ok = False
                severity = MonitorSeverity.ERROR
                reasons.append(f"{side}_image_frame_id_missing")

        left_image = self.image_streams["left"].latest
        right_image = self.image_streams["right"].latest

        if left_image is not None and right_image is not None:
            if (
                left_image.width != right_image.width
                or left_image.height != right_image.height
            ):
                gate_ok = False
                severity = MonitorSeverity.ERROR
                reasons.append("stereo_image_size_mismatch")

            stamp_diff_ms = abs(left_image.stamp_ns - right_image.stamp_ns) / 1_000_000.0
            metrics["left_right_stamp_diff_ms"] = f"{stamp_diff_ms:.3f}"
            metrics["left_stamp_ns"] = str(left_image.stamp_ns)
            metrics["right_stamp_ns"] = str(right_image.stamp_ns)

            if stamp_diff_ms > self.params.soft_sync_max_ms:
                gate_ok = False
                severity = MonitorSeverity.ERROR
                reasons.append("stereo_out_of_sync")
            elif stamp_diff_ms > self.params.hard_sync_max_ms:
                reasons.append("stereo_soft_sync_only")
                if self.params.allow_soft_sync:
                    severity = max(severity, MonitorSeverity.WARN)
                else:
                    gate_ok = False
                    severity = MonitorSeverity.ERROR
        else:
            metrics["left_right_stamp_diff_ms"] = "nan"

        if not reasons:
            summary = "stereo sync monitor OK"
        else:
            summary = ", ".join(dict.fromkeys(reasons))

        return MonitorEvaluation(
            severity=severity,
            gate_ok=gate_ok,
            summary=summary,
            reasons=list(dict.fromkeys(reasons)),
            metrics=metrics,
        )
