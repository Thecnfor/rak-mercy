"""Deterministic, ROS-free replay for :mod:`dual_pen_transport_contract`."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .dual_pen_transport_contract import (
    DualPenTransportProfile,
    NavigationPose,
    SIDES,
    SimulationWorldBinding,
    build_dual_pen_transport_plan,
)


TRACE_SCHEMA = "dual_pen_transport_trace/v1"


@dataclass
class ManualClock:
    now_sec: float = 0.0


@dataclass(frozen=True)
class AdapterResult:
    accepted: bool
    feedback: str
    state: str
    code: str
    completed_at_sec: float
    grasp_confirmed: bool = False
    released_confirmed: bool = False
    collision_free: bool = True
    base_stationary: bool = True

    def as_dict(self) -> dict[str, Any]:
        return vars(self).copy()


@dataclass
class FakeDualPenTransportAdapter:
    """Fault-injectable boundary; records intents but never sends commands."""

    outcomes: dict[Any, Any] = field(default_factory=dict)
    clock: ManualClock = field(default_factory=ManualClock)
    calls: list[dict[str, Any]] = field(default_factory=list)

    def _spec(self, side: str | None, phase: str) -> dict[str, Any]:
        keys = ([f"{side}:{phase}", (side, phase)] if side else []) + [phase]
        value: Any = None
        found = False
        for key in keys:
            if key in self.outcomes:
                value, found = self.outcomes[key], True
                break
        if not found:
            return {"result": "succeeded", "grasp_confirmed": phase == "confirm_grasp", "released_confirmed": phase == "confirm_release"}
        if isinstance(value, dict) and side and side in value and not {"result", "accepted", "feedback", "delay_sec", "collision_free"}.intersection(value):
            value = value[side]
        if isinstance(value, str):
            return {"result": value}
        return dict(value) if isinstance(value, dict) else {"result": "failed"}

    def execute(self, phase: str, *, side: str | None, intent: dict[str, Any], deadline_sec: float) -> AdapterResult:
        spec = self._spec(side, phase)
        accepted = bool(spec.get("accepted", True))
        result = str(spec.get("result", "succeeded")) if accepted else "rejected"
        delay = max(0.0, float(spec.get("delay_sec", 0.0)))
        completed = self.clock.now_sec + delay
        if result == "succeeded" and completed > deadline_sec:
            result, code = "timed_out", "deadline_exceeded"
        else:
            code = "ok" if result == "succeeded" else str(spec.get("code", result))
        output = AdapterResult(
            accepted, str(spec.get("feedback", "reached")), result, code, completed,
            bool(spec.get("grasp_confirmed", phase == "close" and result == "succeeded")),
            bool(spec.get("released_confirmed", phase == "confirm_release" and result == "succeeded")),
            bool(spec.get("collision_free", True)),
            not bool(spec.get("base_moved", False)),
        )
        self.calls.append({"phase": phase, "side": side, "intent": intent, "deadline_sec": deadline_sec, "result": output.as_dict(), "commands_emitted": False})
        return output

    def cancel_all(self, reason: str) -> None:
        self.calls.append({"phase": "cancel", "reason": reason, "commands_emitted": False})

    def stop_unverified(self, reason: str) -> None:
        self.calls.append({"phase": "stop_unverified", "reason": reason, "commands_emitted": False})


def _failure(results: list[AdapterResult], phase: str, max_skew_sec: float) -> tuple[str | None, float]:
    if any(not result.accepted for result in results):
        return f"{phase}_goal_rejected", 0.0
    if any(not result.collision_free for result in results):
        return f"{phase}_collision_detected", 0.0
    bad = next((result for result in results if result.state != "succeeded"), None)
    if bad:
        return f"{phase}_{bad.code}", 0.0
    if any(result.feedback in {"missing", "lost", "none"} for result in results):
        return f"{phase}_feedback_missing", 0.0
    skew = max(result.completed_at_sec for result in results) - min(result.completed_at_sec for result in results)
    return ("barrier_skew_exceeded" if skew > max_skew_sec else None), skew


def run_dual_pen_transport_simulation(
    perception_payload: dict[str, Any], *, now_stamp_ns: int,
    profile: DualPenTransportProfile,
    yellow_work_pose: NavigationPose,
    table_2_drop_targets: dict[str, Any],
    simulation_world: SimulationWorldBinding,
    adapter: FakeDualPenTransportAdapter | None = None,
    cancel_at_phase: str | None = None,
) -> dict[str, Any]:
    """Replay the full task; every abnormal condition stops downstream work."""
    adapter = adapter or FakeDualPenTransportAdapter()
    plan = build_dual_pen_transport_plan(
        perception_payload, now_stamp_ns=now_stamp_ns, profile=profile,
        yellow_work_pose=yellow_work_pose,
        table_2_drop_targets=table_2_drop_targets, simulation_world=simulation_world,
    )
    trace: dict[str, Any] = {"schema": TRACE_SCHEMA, "mode": "offline_isaac_simulation", "commands_emitted": False, "plan": plan, "events": [], "terminal_state": "rejected"}
    if plan["state"] != "simulation_plan_ready":
        trace.update({"state": "rejected", "failure_code": plan["reason"]})
        return trace
    holding = False
    for index, step in enumerate(plan["steps"], start=1):
        phase = str(step["phase"])
        if cancel_at_phase == phase:
            adapter.cancel_all("operator_cancelled")
            trace.update({"terminal_state": "cancelled", "state": "cancelled", "failure_code": "operator_cancelled", "failed_phase": phase})
            return trace
        timeout = float(step.get("deadline_sec", profile.phase_timeout_sec))
        deadline = adapter.clock.now_sec + timeout
        if step["kind"] in {"perception_gate", "base_lock_gate"}:
            results = [adapter.execute(phase, side=None, intent=step, deadline_sec=deadline)]
        elif step["kind"] == "navigation_gate":
            results = [adapter.execute(phase, side=None, intent=step, deadline_sec=deadline)]
        else:
            results = [adapter.execute(phase, side=side, intent=step.get(side, {}), deadline_sec=deadline) for side in SIDES]
        failure, skew = _failure(results, phase, profile.max_barrier_skew_sec)
        if step.get("base_lock_required") is True and any(not result.base_stationary for result in results):
            failure = "base_moved_while_base_lock_required"
        if phase == "confirm_grasp" and not all(result.grasp_confirmed for result in results):
            failure = "grasp_confirmation_missing"
        if phase == "confirm_release" and not all(result.released_confirmed for result in results):
            failure = "release_confirmation_missing"
        event = {"sequence": index, "phase": phase, "results": [result.as_dict() for result in results], "barrier_skew_sec": skew, "failure_code": failure, "commands_emitted": False}
        trace["events"].append(event)
        if failure:
            adapter.cancel_all(failure)
            safety_critical = holding or phase in {"lift", "verify_base_still_at_yellow", "place_pregrasp", "place_approach", "release", "confirm_release"}
            if safety_critical:
                adapter.stop_unverified(failure)
            trace.update({"terminal_state": "locked_manual_intervention" if safety_critical else "failed", "state": "failed", "failure_code": failure, "failed_phase": phase})
            return trace
        adapter.clock.now_sec = max(result.completed_at_sec for result in results)
        if phase == "confirm_grasp":
            holding = True
        elif phase == "confirm_release":
            holding = False
    trace.update({"terminal_state": "succeeded", "state": "complete", "failure_code": None})
    return trace
