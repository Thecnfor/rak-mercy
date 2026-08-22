"""ROS-free stability and one-shot transaction contracts."""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, degrees, isfinite
from typing import Any

import numpy as np


@dataclass(frozen=True)
class SnapshotLimits:
    required_samples: int = 5
    stable_hold_sec: float = 0.5
    max_pair_skew_ms: float = 10.0
    max_plane_center_delta_m: float = 0.005
    max_plane_normal_delta_deg: float = 2.0
    max_base_linear_m_s: float = 0.01
    max_base_angular_rad_s: float = 0.02
    max_joint_delta_deg: float = 0.3
    state_timeout_sec: float = 0.5


@dataclass(frozen=True)
class StabilitySample:
    stamp_ns: int
    receipt_sec: float
    pair_skew_ms: float
    plane_center_m: tuple[float, float, float]
    plane_normal: tuple[float, float, float]
    plane_valid: bool
    base_linear_m_s: float | None
    base_angular_rad_s: float | None
    joint_positions_deg: tuple[float, ...] | None
    odom_age_sec: float | None
    joint_age_sec: float | None
    diagnostics_age_sec: float = 0.0
    pair_reject_count: int = 0


@dataclass(frozen=True)
class NavGate:
    """Immutable navigation-arrival evidence that authorizes one pick window."""

    mission_id: str
    nav_epoch: int
    arrival_stamp_ns: int
    odom_stationary_sec: float
    linear_speed_m_s: float
    angular_speed_rad_s: float


def validate_nav_gate(payload: Any, *, receipt_age_sec: float, limits: SnapshotLimits = SnapshotLimits()) -> tuple[NavGate | None, str]:
    """Accept only fresh, explicit navigation arrival evidence.

    The receipt age deliberately gates a transient-local message as well: a
    previous mission's latched authorization cannot silently arm a later pick.
    """
    if not isinstance(payload, dict):
        return None, "nav_gate_not_object"
    if payload.get("schema") != "pick_nav_gate/v1":
        return None, "nav_gate_schema_invalid"
    if payload.get("state") != "PICK_ARMED":
        return None, "nav_gate_state_not_pick_armed"
    if payload.get("pick_authorized") is not True:
        return None, "nav_gate_not_authorized"
    mission_id = payload.get("mission_id")
    if not isinstance(mission_id, str) or not mission_id.strip():
        return None, "nav_gate_mission_id_missing"
    try:
        nav_epoch = int(payload.get("nav_epoch"))
        evidence = payload["arrival_evidence"]
        if isinstance(payload.get("nav_epoch"), bool) or not isinstance(evidence, dict):
            raise ValueError
        arrival_stamp_ns = int(evidence["stamp_ns"])
        stationary_sec = float(evidence["odom_stationary_sec"])
        linear_speed = float(evidence["linear_speed_m_s"])
        angular_speed = float(evidence["angular_speed_rad_s"])
    except (KeyError, TypeError, ValueError):
        return None, "nav_gate_arrival_evidence_invalid"
    values = (receipt_age_sec, stationary_sec, linear_speed, angular_speed)
    if nav_epoch <= 0 or arrival_stamp_ns <= 0 or not all(isfinite(value) for value in values):
        return None, "nav_gate_arrival_evidence_invalid"
    if receipt_age_sec < 0.0 or receipt_age_sec > limits.state_timeout_sec:
        return None, "nav_gate_arrival_evidence_stale"
    if stationary_sec < limits.stable_hold_sec:
        return None, "nav_gate_odom_not_stable_long_enough"
    if abs(linear_speed) > limits.max_base_linear_m_s:
        return None, "nav_gate_base_linear_velocity_exceeds_limit"
    if abs(angular_speed) > limits.max_base_angular_rad_s:
        return None, "nav_gate_base_angular_velocity_exceeds_limit"
    return NavGate(mission_id.strip(), nav_epoch, arrival_stamp_ns, stationary_sec, linear_speed, angular_speed), "ok"


def _normal_delta_deg(left: np.ndarray, right: np.ndarray) -> float:
    left_norm, right_norm = float(np.linalg.norm(left)), float(np.linalg.norm(right))
    if left_norm <= 1e-9 or right_norm <= 1e-9:
        return float("inf")
    cosine = float(np.clip(abs(float(left @ right)) / (left_norm * right_norm), -1.0, 1.0))
    return degrees(acos(cosine))


class StabilityTracker:
    """Require a contiguous, bounded-motion run before yielding exactly once."""

    def __init__(self, limits: SnapshotLimits = SnapshotLimits(), *, live_mode: bool = False) -> None:
        self.limits = limits
        self.live_mode = bool(live_mode)
        self.samples: list[StabilitySample] = []
        self.locked = False
        self.last_reason = "waiting_for_complete_snapshot"

    def reset(self) -> None:
        self.samples.clear()
        self.locked = False
        self.last_reason = "waiting_for_complete_snapshot"

    def _reject(self, reason: str) -> tuple[bool, str]:
        self.samples.clear()
        self.last_reason = reason
        return False, reason

    def update(self, sample: StabilitySample) -> tuple[bool, str]:
        if self.locked:
            return False, "transaction_already_frozen"
        numeric = (sample.pair_skew_ms, *sample.plane_center_m, *sample.plane_normal)
        if sample.stamp_ns <= 0 or not all(isfinite(float(value)) for value in numeric):
            return self._reject("sample_invalid")
        if not sample.plane_valid:
            return self._reject("plane_not_valid")
        if sample.pair_skew_ms > self.limits.max_pair_skew_ms:
            return self._reject("pair_skew_exceeds_limit")
        if sample.diagnostics_age_sec > self.limits.state_timeout_sec:
            return self._reject("pair_diagnostics_stale")
        if self.live_mode:
            if sample.base_linear_m_s is None or sample.base_angular_rad_s is None or sample.odom_age_sec is None:
                return self._reject("odom_missing")
            if sample.joint_positions_deg is None or sample.joint_age_sec is None:
                return self._reject("right_arm_feedback_missing")
            if sample.odom_age_sec > self.limits.state_timeout_sec:
                return self._reject("odom_stale")
            if sample.joint_age_sec > self.limits.state_timeout_sec:
                return self._reject("right_arm_feedback_stale")
            if abs(sample.base_linear_m_s) > self.limits.max_base_linear_m_s:
                return self._reject("base_linear_velocity_exceeds_limit")
            if abs(sample.base_angular_rad_s) > self.limits.max_base_angular_rad_s:
                return self._reject("base_angular_velocity_exceeds_limit")
        if self.samples:
            previous = self.samples[-1]
            if sample.stamp_ns <= previous.stamp_ns:
                return self._reject("snapshot_stamp_not_increasing")
            if sample.pair_reject_count > previous.pair_reject_count:
                return self._reject("new_pair_rejection_observed")
            center_delta = float(np.linalg.norm(np.asarray(sample.plane_center_m) - np.asarray(previous.plane_center_m)))
            if center_delta > self.limits.max_plane_center_delta_m:
                return self._reject("plane_center_motion_exceeds_limit")
            if _normal_delta_deg(np.asarray(sample.plane_normal), np.asarray(previous.plane_normal)) > self.limits.max_plane_normal_delta_deg:
                return self._reject("plane_normal_motion_exceeds_limit")
            if self.live_mode:
                assert previous.joint_positions_deg is not None and sample.joint_positions_deg is not None
                if len(previous.joint_positions_deg) != 6 or len(sample.joint_positions_deg) != 6:
                    return self._reject("right_arm_feedback_must_have_six_joints")
                if max(abs(a - b) for a, b in zip(previous.joint_positions_deg, sample.joint_positions_deg)) > self.limits.max_joint_delta_deg:
                    return self._reject("right_arm_motion_exceeds_limit")
        self.samples.append(sample)
        if len(self.samples) > self.limits.required_samples:
            self.samples.pop(0)
        held = self.samples[-1].receipt_sec - self.samples[0].receipt_sec
        if len(self.samples) < self.limits.required_samples or held < self.limits.stable_hold_sec:
            self.last_reason = "waiting_for_stability_window"
            return False, self.last_reason
        self.locked = True
        self.last_reason = "stable_snapshot_ready"
        return True, self.last_reason


def new_transaction_id(stamp_ns: int) -> str:
    # The exact sensor stamp is the immutable identity shared by every node.
    return f"pick-{int(stamp_ns)}"


def diagnostic_values(message: Any) -> dict[str, str]:
    result: dict[str, str] = {}
    for status in getattr(message, "status", []):
        for item in getattr(status, "values", []):
            result[str(item.key)] = str(item.value)
    return result
