"""Shared, dry-run-default pick executor for simulation and Mercury adapters.

This is deliberately an orchestration boundary, not a ROS or serial driver.
Both backends report the same :class:`StageResult`; the Mercury backend only
uses ``mercury_arm_safety_contract`` previews and consequently cannot move an
arm.  A future reviewed ROS2 adapter may implement the same small protocol.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Protocol

from .mercury_arm_safety_contract import MercuryArmSafetyProfile, validate_low_speed_motion_request


class PickState(str, Enum):
    VALIDATING = "validating"
    PRE_GRASP = "pre_grasp"
    APPROACH = "approach"
    GRASP = "grasp"
    CLOSE_GRIPPER = "close_gripper"
    LIFT = "lift"
    RETREAT = "safe_retreat"
    RECOVERING = "recovering"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass(frozen=True)
class StageResult:
    state: str
    code: str = "ok"
    detail: str = ""

    @property
    def succeeded(self) -> bool:
        return self.state == "succeeded"


class PickBackend(Protocol):
    """Minimal backend contract: all calls must honour timeout and cancellation."""
    def move(self, stage: str, pose: dict[str, Any], timeout_sec: float, *, cancelled: bool) -> StageResult: ...
    def close_gripper(self, timeout_sec: float, *, cancelled: bool) -> StageResult: ...
    def recover(self, timeout_sec: float) -> StageResult: ...


@dataclass(frozen=True)
class PickTimeouts:
    motion_sec: float = 8.0
    gripper_sec: float = 3.0
    recovery_sec: float = 8.0


@dataclass
class FakePickBackend:
    """Deterministic test/simulation backend.  It contains no hardware path."""
    outcomes: dict[str, str] = field(default_factory=dict)
    calls: list[dict[str, Any]] = field(default_factory=list)

    def _result(self, stage: str, timeout_sec: float, cancelled: bool) -> StageResult:
        self.calls.append({"stage": stage, "timeout_sec": timeout_sec, "cancelled": cancelled})
        if not isinstance(timeout_sec, (int, float)) or timeout_sec <= 0:
            return StageResult("failed", "timeout_invalid")
        if cancelled:
            return StageResult("cancelled", "cancelled")
        outcome = self.outcomes.get(stage, "succeeded")
        return StageResult("succeeded") if outcome == "succeeded" else StageResult(outcome if outcome in {"cancelled", "timed_out"} else "failed", outcome)

    def move(self, stage: str, pose: dict[str, Any], timeout_sec: float, *, cancelled: bool) -> StageResult:
        del pose
        return self._result(stage, timeout_sec, cancelled)

    def close_gripper(self, timeout_sec: float, *, cancelled: bool) -> StageResult:
        return self._result("close_gripper", timeout_sec, cancelled)

    def recover(self, timeout_sec: float) -> StageResult:
        return self._result("recovery", timeout_sec, False)


@dataclass
class MercurySafetyPickBackend:
    """Mercury-shaped adapter which validates only; serial/ROS execution is absent."""
    profile: MercuryArmSafetyProfile = field(default_factory=MercuryArmSafetyProfile)
    calls: list[dict[str, Any]] = field(default_factory=list)

    def move(self, stage: str, pose: dict[str, Any], timeout_sec: float, *, cancelled: bool) -> StageResult:
        self.calls.append({"stage": stage, "timeout_sec": timeout_sec, "cancelled": cancelled})
        if cancelled:
            return StageResult("cancelled", "cancelled")
        checked = validate_low_speed_motion_request({"kind": "cartesian_pose", **pose}, self.profile)
        # Even a valid preview never crosses the safety boundary into hardware.
        return StageResult("failed", checked["reason"] if checked["reason"] != "ok" else "live_motion_adapter_not_implemented")

    def close_gripper(self, timeout_sec: float, *, cancelled: bool) -> StageResult:
        self.calls.append({"stage": "close_gripper", "timeout_sec": timeout_sec, "cancelled": cancelled})
        if cancelled:
            return StageResult("cancelled", "cancelled")
        checked = validate_low_speed_motion_request({"kind": "gripper", "action": "close"}, self.profile)
        return StageResult("failed", checked["reason"] if checked["reason"] != "ok" else "live_gripper_adapter_not_implemented")

    def recover(self, timeout_sec: float) -> StageResult:
        self.calls.append({"stage": "recovery", "timeout_sec": timeout_sec, "cancelled": False})
        return StageResult("failed", "live_recovery_adapter_not_implemented")


def run_pick_state_machine(plan: dict[str, Any], backend: PickBackend, *, timeouts: PickTimeouts = PickTimeouts(),
                           calibration_verified: bool = False, cancel_at: str | None = None) -> dict[str, Any]:
    """Execute a prevalidated plan, fail closed, and attempt bounded fake recovery.

    ``plan`` must be from ``build_dry_run_plan``.  The extra calibration gate
    prevents a stale/inferred transform from being treated as execution input.
    """
    trace: dict[str, Any] = {"mode": "dry_run", "hardware_commands_emitted": False, "events": [], "state": PickState.VALIDATING.value}
    if plan.get("state") != "dry_run_ready":
        return {**trace, "state": PickState.FAILED.value, "reason": plan.get("reason", "plan_not_ready")}
    if not calibration_verified:
        return {**trace, "state": PickState.FAILED.value, "reason": "coordinate_or_calibration_not_verified"}
    steps = {str(item.get("name")): item for item in plan.get("steps", []) if isinstance(item, dict)}
    sequence = (PickState.PRE_GRASP, PickState.APPROACH, PickState.GRASP, PickState.CLOSE_GRIPPER, PickState.LIFT, PickState.RETREAT)
    entered_motion = False
    for state in sequence:
        name = state.value
        if name not in steps:
            return {**trace, "state": PickState.FAILED.value, "reason": f"plan_step_missing:{name}"}
        cancelled = cancel_at == name
        result = backend.close_gripper(timeouts.gripper_sec, cancelled=cancelled) if state is PickState.CLOSE_GRIPPER else backend.move(name, steps[name].get("pose", {}), timeouts.motion_sec, cancelled=cancelled)
        trace["events"].append({"state": name, "result": {"state": result.state, "code": result.code, "detail": result.detail}})
        entered_motion = entered_motion or state is not PickState.PRE_GRASP
        if not result.succeeded:
            terminal = PickState.CANCELLED if result.state == "cancelled" else PickState.FAILED
            trace.update({"state": terminal.value, "reason": result.code, "failed_state": name})
            if entered_motion and result.code not in {"collision", "workspace_violation", "serial_busy"}:
                recovery = backend.recover(timeouts.recovery_sec)
                trace["recovery"] = {"state": recovery.state, "code": recovery.code}
            return trace
    return {**trace, "state": PickState.SUCCEEDED.value, "reason": "ok"}
