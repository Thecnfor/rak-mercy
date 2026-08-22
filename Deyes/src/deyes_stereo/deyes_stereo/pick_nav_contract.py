"""Fail-closed navigation-to-pick transaction contract.

The contract deliberately contains no ROS clients or motion commands.  It
turns independently observed navigation, odometry and pick-terminal evidence
into a small state machine that downstream code can audit.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping


IDLE = "IDLE"
NAVIGATING = "NAVIGATING"
ARRIVED_VERIFY = "ARRIVED_VERIFY"
PICK_ARMED = "PICK_ARMED"
WAIT_PICK_TERMINAL = "WAIT_PICK_TERMINAL"
LEAVE_GRANTED = "LEAVE_GRANTED"
LOCKED = "LOCKED"

TERMINAL_PICK_STATES = frozenset({"succeeded", "failed", "timeout", "timed_out", "cancelled", "rejected"})


@dataclass(frozen=True)
class NavPickLimits:
    max_position_error_m: float = 0.05
    max_yaw_error_rad: float = 0.08
    max_linear_speed_mps: float = 0.01
    max_angular_speed_radps: float = 0.02
    stable_duration_sec: float = 0.5
    max_evidence_age_sec: float = 0.35


def _nonempty(value: Any) -> str:
    return str(value).strip() if value is not None else ""


def _number(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if result == result and abs(result) != float("inf") else None


def _mission(payload: Mapping[str, Any]) -> tuple[dict[str, Any] | None, str | None]:
    if not isinstance(payload, Mapping):
        return None, "mission_not_mapping"
    mission_id = _nonempty(payload.get("mission_id"))
    try:
        nav_epoch = int(payload.get("nav_epoch"))
    except (TypeError, ValueError):
        nav_epoch = 0
    if not mission_id:
        return None, "mission_id_missing"
    if nav_epoch <= 0:
        return None, "nav_epoch_invalid"
    # The snapshot is created only after navigation succeeds, so transaction
    # and calibration identities must not be guessed at mission-start time.
    return {"mission_id": mission_id, "nav_epoch": nav_epoch, "transaction_id": "", "calibration_id": ""}, None


class PickNavCoordinator:
    """Evidence-driven state machine; a locked transaction needs explicit reset."""

    def __init__(self, limits: NavPickLimits = NavPickLimits()) -> None:
        self.limits = limits
        self.state = IDLE
        self.reason = "idle"
        self._mission: dict[str, Any] | None = None
        self._stable_since_ns: int | None = None
        self._arrival_evidence: dict[str, Any] | None = None

    def reset(self, *, explicit: bool = False) -> dict[str, Any]:
        if not explicit:
            return self._gate("manual_reset_requires_explicit_true")
        self.state, self.reason, self._mission, self._stable_since_ns, self._arrival_evidence = IDLE, "manual_reset", None, None, None
        return self._gate("manual_reset")

    def start(self, mission: Mapping[str, Any]) -> dict[str, Any]:
        if self.state != IDLE:
            return self._gate("transaction_not_idle")
        parsed, error = _mission(mission)
        if error:
            return self._lock(error)
        self._mission, self.state, self.reason = parsed, NAVIGATING, "navigation_requested"
        return self._gate("navigation_requested")

    def navigation_evidence(self, evidence: Mapping[str, Any], *, now_ns: int) -> dict[str, Any]:
        if self.state not in {NAVIGATING, ARRIVED_VERIFY}:
            return self._gate("navigation_evidence_not_expected")
        if not isinstance(evidence, Mapping):
            return self._lock("navigation_evidence_not_mapping")
        mismatch = self._identity_error(evidence, ("mission_id", "nav_epoch"))
        if mismatch:
            return self._lock(mismatch)
        evidence_ns = self._stamp(evidence)
        if evidence_ns is None or now_ns < evidence_ns or now_ns - evidence_ns > int(self.limits.max_evidence_age_sec * 1e9):
            return self._lock("navigation_evidence_stale_or_invalid")
        result = _nonempty(evidence.get("result")).lower()
        if result in {"failed", "timeout", "timed_out", "cancelled", "rejected", "aborted"}:
            return self._lock("navigation_" + result)
        if result != "succeeded":
            return self._gate("navigation_not_succeeded")
        position_error = _number(evidence.get("position_error_m"))
        yaw_error = _number(evidence.get("yaw_error_rad"))
        linear_speed = _number(evidence.get("linear_speed_mps"))
        angular_speed = _number(evidence.get("angular_speed_radps"))
        if None in (position_error, yaw_error, linear_speed, angular_speed):
            return self._lock("navigation_evidence_fields_invalid")
        if position_error > self.limits.max_position_error_m or abs(yaw_error) > self.limits.max_yaw_error_rad:
            return self._lock("arrival_pose_out_of_tolerance")
        self.state = ARRIVED_VERIFY
        if abs(linear_speed) > self.limits.max_linear_speed_mps or abs(angular_speed) > self.limits.max_angular_speed_radps:
            self._stable_since_ns = None
            return self._gate("arrival_not_stationary")
        if self._stable_since_ns is None:
            self._stable_since_ns = evidence_ns
        if evidence_ns - self._stable_since_ns < int(self.limits.stable_duration_sec * 1e9):
            return self._gate("arrival_stability_window_pending")
        self._arrival_evidence = {
            "stamp_ns": evidence_ns,
            "odom_stationary_sec": (evidence_ns - self._stable_since_ns) / 1e9,
            "linear_speed_m_s": abs(linear_speed),
            "angular_speed_rad_s": abs(angular_speed),
        }
        self.state, self.reason = PICK_ARMED, "arrival_verified"
        return self._gate("arrival_verified")

    def begin_pick(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        if self.state != WAIT_PICK_TERMINAL:
            return self._gate("snapshot_not_bound_for_pick")
        mismatch = self._identity_error(payload, ("mission_id", "nav_epoch", "transaction_id"))
        if mismatch:
            return self._lock(mismatch)
        calibration_id = _nonempty(payload.get("calibration_id"))
        if not calibration_id:
            return self._lock("calibration_id_missing")
        if self._mission is None:
            return self._lock("transaction_context_missing")
        if self._mission["calibration_id"] and calibration_id != self._mission["calibration_id"]:
            return self._lock("calibration_id_mismatch")
        if payload.get("dry_run") is True or payload.get("trusted_for_execution") is not True:
            return self._lock("pick_execution_not_permitted")
        self._mission["calibration_id"] = calibration_id
        self.reason = "pick_started"
        return self._gate("pick_started")

    def bind_snapshot(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        """Bind the one frozen snapshot before accepting execution results."""
        if self.state != PICK_ARMED:
            return self._gate("snapshot_not_expected")
        mismatch = self._identity_error(payload, ("mission_id", "nav_epoch"))
        if mismatch:
            return self._lock(mismatch)
        if _nonempty(payload.get("state")) != "snapshot_frozen":
            return self._lock("snapshot_not_frozen")
        transaction_id = _nonempty(payload.get("transaction_id"))
        if not transaction_id:
            return self._lock("snapshot_transaction_id_missing")
        if self._mission is None:
            return self._lock("transaction_context_missing")
        self._mission["transaction_id"] = transaction_id
        self.state, self.reason = WAIT_PICK_TERMINAL, "snapshot_bound"
        return self._gate("snapshot_bound")

    def pick_terminal(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        if self.state != WAIT_PICK_TERMINAL:
            return self._gate("pick_terminal_not_expected")
        mismatch = self._identity_error(payload, ("mission_id", "nav_epoch", "transaction_id", "calibration_id"))
        if mismatch:
            return self._lock(mismatch)
        terminal = _nonempty(payload.get("state")).lower()
        if terminal not in TERMINAL_PICK_STATES:
            return self._lock("pick_terminal_state_invalid")
        if payload.get("dry_run") is True:
            return self._lock("pick_terminal_dry_run_rejected")
        if terminal != "succeeded":
            return self._lock("pick_" + terminal)
        self.state, self.reason = LEAVE_GRANTED, "pick_succeeded_leave_granted"
        return self._gate("pick_succeeded_leave_granted")

    def _identity_error(self, payload: Mapping[str, Any], keys: tuple[str, ...]) -> str | None:
        if not isinstance(payload, Mapping) or self._mission is None:
            return "transaction_context_missing"
        for key in keys:
            if payload.get(key) != self._mission[key]:
                return key + "_mismatch"
        return None

    @staticmethod
    def _stamp(payload: Mapping[str, Any]) -> int | None:
        try:
            stamp = int(payload.get("stamp_ns"))
        except (TypeError, ValueError):
            return None
        return stamp if stamp > 0 else None

    def _lock(self, reason: str) -> dict[str, Any]:
        self.state, self.reason, self._stable_since_ns = LOCKED, reason, None
        return self._gate(reason)

    def _gate(self, reason: str) -> dict[str, Any]:
        mission = self._mission or {"mission_id": "", "nav_epoch": 0, "transaction_id": "", "calibration_id": ""}
        evidence = {
            "stamp_ns": 0, "odom_stationary_sec": 0.0,
            "linear_speed_m_s": 0.0, "angular_speed_rad_s": 0.0,
        }
        if self._stable_since_ns is not None:
            evidence["stamp_ns"] = self._stable_since_ns
        # Save the accepted evidence in a flat schema so snapshot validation
        # never has to understand this coordinator's internal state.
        if self._arrival_evidence is not None:
            evidence = dict(self._arrival_evidence)
        return {
            "schema": "pick_nav_gate/v1", "state": self.state, "reason": reason,
            "pick_authorized": self.state == PICK_ARMED,
            "mission_id": mission["mission_id"], "nav_epoch": mission["nav_epoch"],
            "arrival_evidence": evidence,
            "transaction_id": mission["transaction_id"], "calibration_id": mission["calibration_id"],
            "commands_emitted": False,
        }
