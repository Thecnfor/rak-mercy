"""ROS-free end-to-end pen-pick simulation with deterministic fault injection.

The fake adapters model the future Nav2, dual-arm, and gripper boundaries but
cannot open a device, import a vendor SDK, or send a ROS command. Their JSON
trace is the acceptance artifact for replacing only the adapter boundary later.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .pen_pick_dry_run_contract import PickPlanLimits, build_dry_run_plan


TERMINAL_RESULTS = {"succeeded", "rejected", "timed_out", "unreachable", "failed", "cancelled"}
TRACE_VERSION = "pen_pick_trace/v1"


@dataclass(frozen=True)
class AdapterResult:
    state: str
    code: str
    detail: str = ""

    @property
    def succeeded(self) -> bool:
        return self.state == "succeeded"

    def as_dict(self) -> dict[str, str]:
        return {"state": self.state, "code": self.code, "detail": self.detail}


def _outcome(value: str, *, default_failure: str = "failed") -> AdapterResult:
    if not isinstance(value, str) or value not in TERMINAL_RESULTS:
        return AdapterResult("failed", "invalid_fake_outcome", repr(value))
    return AdapterResult(value, "ok" if value == "succeeded" else value, "" if value == "succeeded" else default_failure)


def _invalid_timeout(timeout_sec: float) -> AdapterResult | None:
    return None if isinstance(timeout_sec, (int, float)) and timeout_sec > 0.0 else AdapterResult("failed", "timeout_invalid", repr(timeout_sec))


@dataclass
class FakeNav2Adapter:
    """Fake standard NavigateToPose boundary; no rclpy ActionClient exists here."""

    outcome: str = "succeeded"
    calls: list[dict[str, Any]] = field(default_factory=list)

    def navigate(self, goal: dict[str, Any], timeout_sec: float, *, cancelled: bool = False) -> AdapterResult:
        self.calls.append({"goal": goal, "timeout_sec": timeout_sec, "cancelled": cancelled})
        invalid_timeout = _invalid_timeout(timeout_sec)
        if invalid_timeout is not None:
            return invalid_timeout
        return AdapterResult("cancelled", "cancelled", "cancelled_before_nav_goal") if cancelled else _outcome(self.outcome, default_failure="navigation_not_completed")


@dataclass
class FakeMercuryDualArmAdapter:
    """Fake Turing dual-six-axis arm contract with stage-level fault injection."""

    outcomes: dict[str, str] = field(default_factory=dict)
    calls: list[dict[str, Any]] = field(default_factory=list)

    def move(self, stage: str, pose: dict[str, Any], timeout_sec: float, *, selected_arm: str, cancelled: bool = False) -> AdapterResult:
        self.calls.append({"stage": stage, "pose": pose, "timeout_sec": timeout_sec, "selected_arm": selected_arm, "cancelled": cancelled})
        invalid_timeout = _invalid_timeout(timeout_sec)
        if invalid_timeout is not None:
            return invalid_timeout
        if selected_arm not in {"left", "right"}:
            return AdapterResult("failed", "selected_arm_invalid", selected_arm)
        if cancelled:
            return AdapterResult("cancelled", "cancelled", f"cancelled_before_{stage}")
        return _outcome(self.outcomes.get(stage, "succeeded"), default_failure=f"{stage}_not_completed")


@dataclass
class FakeGripperAdapter:
    """Fake gripper contract; close has explicit terminal feedback/result."""

    outcome: str = "succeeded"
    calls: list[dict[str, Any]] = field(default_factory=list)

    def close(self, timeout_sec: float, *, cancelled: bool = False) -> AdapterResult:
        self.calls.append({"timeout_sec": timeout_sec, "cancelled": cancelled})
        invalid_timeout = _invalid_timeout(timeout_sec)
        if invalid_timeout is not None:
            return invalid_timeout
        return AdapterResult("cancelled", "cancelled", "cancelled_before_close") if cancelled else _outcome(self.outcome, default_failure="gripper_close_not_confirmed")


@dataclass(frozen=True)
class SimulationTimeouts:
    navigation_sec: float = 30.0
    arm_motion_sec: float = 8.0
    gripper_sec: float = 3.0


def _event(index: int, step: str, adapter: str, timeout_sec: float, result: AdapterResult) -> dict[str, Any]:
    return {"sequence": index, "step": step, "adapter": adapter, "timeout_sec": timeout_sec, "result": result.as_dict()}


def run_pick_simulation(
    candidate_payload: dict[str, Any], *, now_stamp_ns: int,
    navigation_goal: dict[str, Any] | None = None, limits: PickPlanLimits = PickPlanLimits(),
    nav: FakeNav2Adapter | None = None, arm: FakeMercuryDualArmAdapter | None = None,
    gripper: FakeGripperAdapter | None = None, timeouts: SimulationTimeouts = SimulationTimeouts(),
    site_profile_validated: bool = True, enable_execution: bool = True,
    operator_approved: bool = True, cancel_at_step: str | None = None,
    include_navigation_gate: bool = False, selected_arm: str = "left",
) -> dict[str, Any]:
    """Run every pick phase in order; first rejection/cancel stops the trace."""
    nav, arm, gripper = nav or FakeNav2Adapter(), arm or FakeMercuryDualArmAdapter(), gripper or FakeGripperAdapter()
    trace: dict[str, Any] = {"trace_version": TRACE_VERSION, "mode": "offline_simulation", "hardware_commands_emitted": False, "events": []}
    plan = build_dry_run_plan(candidate_payload, now_stamp_ns=now_stamp_ns, limits=limits, site_profile_validated=site_profile_validated, enable_execution=enable_execution, operator_approved=operator_approved, include_navigation_gate=include_navigation_gate)
    trace["plan_state"] = plan["state"]
    if plan["state"] != "dry_run_ready":
        trace.update({"terminal_state": "failed", "reason": plan["reason"], "plan": plan})
        return trace
    for required, value in (("site_profile_not_validated", site_profile_validated), ("enable_execution_false", enable_execution), ("operator_approval_missing", operator_approved)):
        if not value:
            trace.update({"terminal_state": "failed", "reason": required, "plan": plan})
            return trace
    trace["target_id"] = plan["target_id"]
    step_map = {item["name"]: item for item in plan["steps"]}
    operations = ([] if not include_navigation_gate else [
        ("verify_navigation_arrival", "nav2", timeouts.navigation_sec),
    ]) + [
        ("pre_grasp", "dual_arm", timeouts.arm_motion_sec),
        ("approach", "dual_arm", timeouts.arm_motion_sec),
        ("grasp", "dual_arm", timeouts.arm_motion_sec),
        ("close_gripper", "gripper", timeouts.gripper_sec),
        ("lift", "dual_arm", timeouts.arm_motion_sec),
        ("safe_retreat", "dual_arm", timeouts.arm_motion_sec),
    ]
    for index, (step, adapter, timeout_sec) in enumerate(operations, start=1):
        cancelled = cancel_at_step == step
        if adapter == "nav2":
            result = nav.navigate(navigation_goal or {"frame_id": "map", "semantic": "validated_pregrasp_base_pose"}, timeout_sec, cancelled=cancelled)
        elif adapter == "gripper":
            result = gripper.close(timeout_sec, cancelled=cancelled)
        else:
            result = arm.move(step, step_map[step]["pose"], timeout_sec, selected_arm=selected_arm, cancelled=cancelled)
        trace["events"].append(_event(index, step, adapter, timeout_sec, result))
        if not result.succeeded:
            trace.update({"terminal_state": "cancelled" if result.state == "cancelled" else "failed", "reason": result.code, "failed_step": step, "plan": plan})
            return trace
    trace.update({"terminal_state": "succeeded", "reason": "ok", "plan": plan})
    return trace


def write_simulation_trace(trace: dict[str, Any], output_path: str | Path, *, temp_root: str | Path = r"E:\a_robot\temp\deyes") -> Path:
    """Write a trace only under the external temporary evidence directory."""
    root = Path(temp_root).resolve()
    output = Path(output_path).resolve()
    if root not in output.parents:
        raise ValueError("trace_output_must_be_under_temp_deyes")
    if trace.get("trace_version") != TRACE_VERSION:
        raise ValueError("trace_version_invalid")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(trace, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return output
