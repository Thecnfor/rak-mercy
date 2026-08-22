"""Deterministic, ROS-free simulation of the dual pen co-grasp contract."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any

from .dual_pen_cograsp_contract import DualPenCograspSiteProfile, build_dual_pen_cograsp_plan


TRACE_SCHEMA = "dual_pen_cograsp_trace/v1"
SIDES = ("left", "right")


@dataclass
class ManualClock:
    """Clock controlled by tests; no wall-clock time is consulted."""
    now_sec: float = 0.0

    def advance(self, seconds: float) -> float:
        self.now_sec += max(0.0, float(seconds))
        return self.now_sec


@dataclass(frozen=True)
class AdapterResult:
    accepted: bool
    feedback: str
    state: str
    code: str
    completed_at_sec: float
    grasp_confirmed: bool = False

    def as_dict(self) -> dict[str, Any]:
        return {"goal_accepted": self.accepted, "feedback": self.feedback, "result": self.state, "code": self.code, "completed_at_sec": self.completed_at_sec, "grasp_confirmed": self.grasp_confirmed}


@dataclass
class FakeDualArmGripperAdapter:
    """Independent dual-side arm/gripper boundary with deterministic outcomes.

    ``outcomes`` accepts ``"left:approach"`` keys (or a phase mapping with
    side values). A value may be a result string, or a mapping containing
    ``accepted``, ``feedback``, ``result``, ``delay_sec``, and
    ``grasp_confirmed``. No method emits hardware commands.
    """
    outcomes: dict[Any, Any] = field(default_factory=dict)
    clock: ManualClock = field(default_factory=ManualClock)
    calls: list[dict[str, Any]] = field(default_factory=list)

    def _spec(self, side: str, phase: str) -> dict[str, Any]:
        absent = object()
        value = self.outcomes.get(f"{side}:{phase}", self.outcomes.get((side, phase), self.outcomes.get(phase, absent)))
        if value is absent:
            # The default fake models an explicit positive gripper sensor
            # indication, not an inferred success result.
            return {"result": "succeeded", "grasp_confirmed": phase == "close"}
        if isinstance(value, dict) and side in value and not {"accepted", "feedback", "result", "delay_sec", "grasp_confirmed"}.intersection(value):
            value = value[side]
        if isinstance(value, str):
            return {"result": value}
        return dict(value) if isinstance(value, dict) else {"result": "failed"}

    def accept_goal(self, side: str, phase: str, goal: dict[str, Any], deadline_sec: float) -> bool:
        accepted = bool(self._spec(side, phase).get("accepted", True))
        self.calls.append({"op": "goal", "side": side, "phase": phase, "goal": goal, "deadline_sec": deadline_sec, "accepted": accepted, "commands_emitted": False})
        return accepted

    def feedback(self, side: str, phase: str) -> str:
        feedback = str(self._spec(side, phase).get("feedback", "reached"))
        self.calls.append({"op": "feedback", "side": side, "phase": phase, "feedback": feedback, "commands_emitted": False})
        return feedback

    def result(self, side: str, phase: str, deadline_sec: float, *, accepted: bool, required_duration_sec: float = 0.0) -> AdapterResult:
        spec = self._spec(side, phase)
        if not accepted:
            return AdapterResult(False, "not_dispatched", "rejected", "goal_rejected", self.clock.now_sec)
        result = str(spec.get("result", "succeeded"))
        default_delay = required_duration_sec if phase == "hold" else 0.0
        completed = self.clock.now_sec + max(0.0, float(spec.get("delay_sec", default_delay)))
        if completed > deadline_sec and result == "succeeded":
            result, code = "timed_out", "deadline_exceeded"
        else:
            code = "ok" if result == "succeeded" else str(spec.get("code", result))
        feedback = self.feedback(side, phase)
        confirmed = bool(spec.get("grasp_confirmed", False)) if phase == "close" else False
        self.calls.append({"op": "result", "side": side, "phase": phase, "deadline_sec": deadline_sec, "result": result, "commands_emitted": False})
        return AdapterResult(True, feedback, result, code, completed, confirmed)

    def cancel(self, side: str, phase: str) -> None:
        self.calls.append({"op": "cancel", "side": side, "phase": phase, "commands_emitted": False})

    def stop_unverified(self, side: str, reason: str) -> None:
        self.calls.append({"op": "stop_unverified", "side": side, "reason": reason, "commands_emitted": False})


@dataclass(frozen=True)
class SimulationTimeouts:
    phase_sec: float | None = None
    hold_sec: float | None = None


def _failure_code(results: dict[str, AdapterResult], phase: str, max_skew: float) -> tuple[str | None, float]:
    skew = abs(results["left"].completed_at_sec - results["right"].completed_at_sec)
    if any(result.state == "stop_unverified" or result.code == "stop_unverified" for result in results.values()):
        return f"{phase}_stop_unverified", skew
    if any(result.state == "cancelled" or result.code == "cancelled" for result in results.values()):
        return f"{phase}_cancelled", skew
    failed = next((r for r in results.values() if r.state != "succeeded"), None)
    if failed is not None:
        return f"{phase}_{failed.code}", skew
    if any(result.feedback in {"missing", "lost", "none"} for result in results.values()):
        return f"{phase}_feedback_missing", skew
    if skew > max_skew:
        return "barrier_skew_exceeded", skew
    return None, skew


def run_dual_pen_cograsp_simulation(candidate: dict[str, Any], *, now_stamp_ns: int, profile: DualPenCograspSiteProfile = DualPenCograspSiteProfile(), adapter: FakeDualArmGripperAdapter | None = None, timeouts: SimulationTimeouts = SimulationTimeouts()) -> dict[str, Any]:
    """Run all co-grasp barriers. Any side failure cancels both and stops.

    Lift and hold are treated as safety-critical: their failure locks the trace
    for manual intervention and asks the fake boundary to stop unverified.
    """
    adapter = adapter or FakeDualArmGripperAdapter()
    plan = build_dual_pen_cograsp_plan(candidate, now_stamp_ns=now_stamp_ns, profile=profile)
    trace: dict[str, Any] = {"schema": TRACE_SCHEMA, "mode": "offline_simulation", "commands_emitted": False, "event_sequence": [], "events": [], "plan": plan, "terminal_state": "rejected"}
    if plan["state"] != "ready":
        trace.update({"failure_code": plan["reason"], "state": "rejected"})
        return trace
    state = "ready"
    for index, step in enumerate(plan["steps"], 1):
        phase = step["phase"]
        override = timeouts.hold_sec if phase == "hold" else timeouts.phase_sec
        timeout = step["deadline_sec"] if override is None else override
        phase_start = adapter.clock.now_sec
        deadline = phase_start + timeout
        required_hold = float(step.get("hold_sec", 0.0)) if phase == "hold" else 0.0
        results: dict[str, AdapterResult] = {}
        for side in SIDES:
            goal = step.get(side, {"semantic": step["name"]})
            accepted = adapter.accept_goal(side, phase, goal, deadline)
            results[side] = adapter.result(side, phase, deadline, accepted=accepted, required_duration_sec=required_hold)
        failure, skew = _failure_code(results, phase, profile.max_barrier_skew_sec)
        # Hold is a duration property, not merely a successful action result.
        # A too-short timeout or early terminal feedback never counts as holding.
        if phase == "hold" and (timeout < required_hold or any(result.completed_at_sec - phase_start < required_hold for result in results.values())):
            failure = "hold_duration_not_met"
        safety_stop = any(result.state == "stop_unverified" or result.code == "stop_unverified" for result in results.values())
        cancelled = any(result.state == "cancelled" or result.code == "cancelled" for result in results.values())
        next_state = f"{phase}_complete" if failure is None else ("locked_manual_intervention" if safety_stop or phase in {"lift", "hold"} else ("cancelled" if cancelled else "failed"))
        event = {"sequence": index, "event": "barrier", "state_from": state, "state_to": next_state, "phase": phase, "left": results["left"].as_dict(), "right": results["right"].as_dict(), "barrier_skew_sec": skew, "deadline_sec": deadline, "failure_code": failure, "commands_emitted": False}
        trace["events"].append(event)
        trace["event_sequence"].append(phase)
        if failure is not None:
            for side in SIDES:
                adapter.cancel(side, phase)
                if safety_stop or phase in {"lift", "hold"}:
                    adapter.stop_unverified(side, failure)
            terminal_state = "locked_manual_intervention" if safety_stop or phase in {"lift", "hold"} else ("cancelled" if cancelled else "failed")
            trace.update({"terminal_state": terminal_state, "state": next_state, "failure_code": failure, "failed_phase": phase})
            return trace
        # A successful close result is not enough: both gripping confirmations
        # must be explicit before the plan is allowed to reach lift.
        if phase == "close" and not all(result.grasp_confirmed for result in results.values()):
            failure = "grasp_confirmation_missing"
            event["failure_code"], event["state_to"] = failure, "failed"
            for side in SIDES:
                adapter.cancel(side, phase)
            trace.update({"terminal_state": "failed", "state": "failed", "failure_code": failure, "failed_phase": "confirm"})
            return trace
        adapter.clock.now_sec = max(result.completed_at_sec for result in results.values())
        state = next_state
    trace.update({"terminal_state": "succeeded", "state": "holding_complete", "failure_code": None})
    return trace


def trace_json(trace: dict[str, Any]) -> str:
    """Stable JSON encoding helper; it intentionally does not write a file."""
    if trace.get("schema") != TRACE_SCHEMA or trace.get("commands_emitted") is not False:
        raise ValueError("dual_pen_trace_contract_invalid")
    return json.dumps(trace, sort_keys=True, ensure_ascii=False)
