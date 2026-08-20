"""Deterministic, ROS-free replay for :mod:`dual_pen_transport_contract`."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite
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
    translation_dx_m: float = 0.0
    translation_dy_m: float = 0.0
    linear_x_mps: float = 0.0
    linear_y_mps: float = 0.0
    angular_z_rad_s: float = 0.0
    yaw_error_rad: float = 0.0
    zero_velocity_confirmations: int = 0
    tf_available: bool = True
    odom_available: bool = True
    turn_safe_pose_confirmed: bool = False
    both_grasps_confirmed: bool = False
    collision_sweep_validated: bool = False

    def as_dict(self) -> dict[str, Any]:
        return vars(self).copy()


@dataclass
class FakeDualPenTransportAdapter:
    """Fault-injectable boundary; records intents but never sends commands."""

    outcomes: dict[Any, Any] = field(default_factory=dict)
    clock: ManualClock = field(default_factory=ManualClock)
    calls: list[dict[str, Any]] = field(default_factory=list)

    def _spec(self, side: str | None, phase: str) -> dict[str, Any]:
        defaults: dict[str, Any] = {"result": "succeeded", "grasp_confirmed": phase == "confirm_grasp", "released_confirmed": phase == "confirm_release"}
        if phase == "rotate_in_place_to_table_2":
            defaults.update({"angular_z_rad_s": 0.08, "yaw_error_rad": 0.01, "zero_velocity_confirmations": 5, "turn_safe_pose_confirmed": True, "both_grasps_confirmed": True, "collision_sweep_validated": True})
        keys = ([f"{side}:{phase}", (side, phase)] if side else []) + [phase]
        value: Any = None
        found = False
        for key in keys:
            if key in self.outcomes:
                value, found = self.outcomes[key], True
                break
        if not found:
            return defaults
        if isinstance(value, dict) and side and side in value and not {"result", "accepted", "feedback", "delay_sec", "collision_free", "translation_dx_m", "translation_dy_m"}.intersection(value):
            value = value[side]
        if isinstance(value, str):
            return {**defaults, "result": value}
        return {**defaults, **dict(value)} if isinstance(value, dict) else {**defaults, "result": "failed"}

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
            float(spec.get("translation_dx_m", 0.0)),
            float(spec.get("translation_dy_m", 0.0)),
            float(spec.get("linear_x_mps", 0.0)),
            float(spec.get("linear_y_mps", 0.0)),
            float(spec.get("angular_z_rad_s", 0.0)),
            float(spec.get("yaw_error_rad", 0.0)),
            int(spec.get("zero_velocity_confirmations", 0)),
            bool(spec.get("tf_available", True)),
            bool(spec.get("odom_available", True)),
            bool(spec.get("turn_safe_pose_confirmed", False)),
            bool(spec.get("both_grasps_confirmed", False)),
            bool(spec.get("collision_sweep_validated", False)),
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
    table_2_orientation_pose: NavigationPose,
    table_2_drop_targets: dict[str, Any],
    simulation_world: SimulationWorldBinding,
    turn_safety_contract: dict[str, Any],
    adapter: FakeDualPenTransportAdapter | None = None,
    cancel_at_phase: str | None = None,
) -> dict[str, Any]:
    """Replay the full task; every abnormal condition stops downstream work."""
    adapter = adapter or FakeDualPenTransportAdapter()
    plan = build_dual_pen_transport_plan(
        perception_payload, now_stamp_ns=now_stamp_ns, profile=profile,
        yellow_work_pose=yellow_work_pose, table_2_orientation_pose=table_2_orientation_pose,
        table_2_drop_targets=table_2_drop_targets, simulation_world=simulation_world,
        turn_safety_contract=turn_safety_contract,
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
        if step["kind"] in {"perception_gate", "rotation_intent"}:
            results = [adapter.execute(phase, side=None, intent=step, deadline_sec=deadline)]
        elif step["kind"] == "navigation_gate":
            results = [adapter.execute(phase, side=None, intent=step, deadline_sec=deadline)]
        else:
            results = [adapter.execute(phase, side=side, intent=step.get(side, {}), deadline_sec=deadline) for side in SIDES]
        failure, skew = _failure(results, phase, profile.max_barrier_skew_sec)
        if step.get("translation_lock_required") is True:
            if any(not all(isfinite(value) for value in (result.translation_dx_m, result.translation_dy_m)) for result in results):
                failure = "translation_lock_feedback_nonfinite"
            elif any(not result.tf_available for result in results):
                failure = "translation_lock_tf_missing"
            elif any(not result.odom_available for result in results):
                failure = "translation_lock_odom_missing"
            elif any(abs(result.translation_dx_m) > float(step["translation_x_tolerance_m"]) for result in results):
                failure = "translation_lock_x_drift_exceeded"
            elif any(abs(result.translation_dy_m) > float(step["translation_y_tolerance_m"]) for result in results):
                failure = "translation_lock_y_drift_exceeded"
        if phase == "rotate_in_place_to_table_2" and failure is None:
            turn = results[0]
            if not all(isfinite(value) for value in (turn.linear_x_mps, turn.linear_y_mps, turn.angular_z_rad_s, turn.yaw_error_rad)):
                failure = "rotate_in_place_feedback_nonfinite"
            elif not turn.turn_safe_pose_confirmed:
                failure = "turn_safe_pose_not_confirmed"
            elif not holding or not turn.both_grasps_confirmed:
                failure = "both_grasps_not_confirmed_for_turn"
            elif not turn.collision_sweep_validated:
                failure = "collision_sweep_not_validated_for_turn"
            elif abs(turn.linear_x_mps) > 1e-9 or abs(turn.linear_y_mps) > 1e-9:
                failure = "rotate_in_place_linear_velocity_nonzero"
            elif abs(turn.angular_z_rad_s) > float(step["max_abs_angular_z_rad_s"]):
                failure = "rotate_in_place_angular_speed_exceeded"
            elif abs(turn.yaw_error_rad) > float(step["yaw_tolerance_rad"]):
                failure = "rotate_in_place_yaw_tolerance_not_met"
            elif turn.zero_velocity_confirmations < int(step["zero_velocity_confirmations_required"]):
                failure = "rotate_in_place_zero_confirmations_insufficient"
        if phase == "confirm_grasp" and not all(result.grasp_confirmed for result in results):
            failure = "grasp_confirmation_missing"
        if phase == "confirm_release" and not all(result.released_confirmed for result in results):
            failure = "release_confirmation_missing"
        event = {"sequence": index, "phase": phase, "results": [result.as_dict() for result in results], "barrier_skew_sec": skew, "failure_code": failure, "commands_emitted": False}
        trace["events"].append(event)
        if failure:
            adapter.cancel_all(failure)
            motion_integrity_failure = failure.startswith("translation_lock_") or failure in {
                "rotate_in_place_linear_velocity_nonzero", "rotate_in_place_angular_speed_exceeded",
                "rotate_in_place_yaw_tolerance_not_met", "rotate_in_place_zero_confirmations_insufficient",
                "turn_safe_pose_not_confirmed", "both_grasps_not_confirmed_for_turn",
                "collision_sweep_not_validated_for_turn",
            }
            safety_critical = holding or motion_integrity_failure or phase in {"lift", "rotate_in_place_to_table_2", "place_pregrasp", "place_approach", "release", "confirm_release"}
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
