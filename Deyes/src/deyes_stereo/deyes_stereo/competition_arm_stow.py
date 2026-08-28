"""Fail-closed two-arm preparation for the hash-bound Isaac stow evidence."""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Any, Callable

from .competition_clearance_evidence import evaluate_profile_clearance


PORTS = {"left": "/dev/left_arm", "right": "/dev/right_arm"}


@dataclass(frozen=True)
class ArmStowPlan:
    order: tuple[str, str]
    power_on_deg: dict[str, tuple[float, ...]]
    stow_deg: dict[str, tuple[float, ...]]
    initial_tolerance_deg: float = 1.0
    final_tolerance_deg: float = 1.0
    speed: int = 5
    timeout_sec: float = 12.0


def load_stow_plan(profile_path: Path) -> ArmStowPlan:
    admission = evaluate_profile_clearance(profile_path)
    if not admission.accepted or not admission.evidence_path:
        raise RuntimeError(f"clearance_evidence_not_admitted:{admission.reason}")
    evidence = json.loads(Path(admission.evidence_path).read_text(encoding="utf-8"))
    initial = evidence["initial_pose"]
    order_name = str(initial["selected_order"])
    order = ("left", "right") if order_name == "left_then_right" else ("right", "left")
    return ArmStowPlan(
        order=order,
        power_on_deg={side: _rad_to_deg(initial[f"power_on_{side}_rad"]) for side in PORTS},
        stow_deg={side: _rad_to_deg(initial[f"stow_{side}_rad"]) for side in PORTS},
    )


def execute_stow_plan(
    plan: ArmStowPlan,
    *,
    mercury_factory: Callable[[str], Any],
    serial_owner_scan: Callable[[str], list[int]],
    lock_acquire: Callable[[str], Any],
    lock_release: Callable[[Any], None],
    clock: Callable[[], float] = time.monotonic,
    sleeper: Callable[[float], None] = time.sleep,
) -> dict[str, Any]:
    trace: list[dict[str, Any]] = []
    commands_emitted = False
    for side in plan.order:
        port = PORTS[side]
        owners = serial_owner_scan(port)
        if owners:
            raise RuntimeError(f"serial_port_owned:{side}:{owners}")
        handle = lock_acquire(side)
        robot = None
        try:
            robot = mercury_factory(port)
            _require_status(robot)
            current = _angles(robot)
            _require_close(current, plan.power_on_deg[side], plan.initial_tolerance_deg, f"initial_pose_mismatch:{side}")
            if robot.is_power_on() != 1:
                commands_emitted = True
                robot.power_on()
                sleeper(1.0)
                if robot.is_power_on() != 1:
                    raise RuntimeError(f"power_on_feedback_failed:{side}")
            commands_emitted = True
            robot.send_angles(list(plan.stow_deg[side]), plan.speed)
            final = _wait_angles(robot, plan.stow_deg[side], plan, clock=clock, sleeper=sleeper)
            _require_status(robot)
            trace.append({"side": side, "port": port, "initial_deg": list(current), "stow_deg": list(final)})
        except Exception:
            if robot is not None and commands_emitted:
                stop = getattr(robot, "stop", None)
                if callable(stop):
                    stop()
            raise
        finally:
            lock_release(handle)
    return {
        "schema": "competition_arm_stow_result/v1",
        "success": True,
        "order": list(plan.order),
        "trace": trace,
        "commands_emitted": commands_emitted,
    }


def _rad_to_deg(values: Any) -> tuple[float, ...]:
    result = tuple(math.degrees(float(value)) for value in values)
    if len(result) != 6 or not all(math.isfinite(value) for value in result):
        raise RuntimeError("stow_joint_vector_invalid")
    return result


def _angles(robot: Any) -> tuple[float, ...]:
    values = robot.get_angles()
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"joint_feedback_invalid:{values}") from exc
    if len(result) != 6 or not all(math.isfinite(value) for value in result):
        raise RuntimeError(f"joint_feedback_invalid:{values}")
    return result


def _require_status(robot: Any) -> None:
    status = robot.get_robot_status()
    if not isinstance(status, list) or any(int(value) != 0 for value in status):
        raise RuntimeError(f"robot_status_not_ok:{status}")


def _require_close(actual: tuple[float, ...], expected: tuple[float, ...], tolerance: float, reason: str) -> None:
    if max(abs(a - b) for a, b in zip(actual, expected)) > tolerance:
        raise RuntimeError(f"{reason}:expected={expected}:actual={actual}")


def _wait_angles(robot: Any, expected: tuple[float, ...], plan: ArmStowPlan, *, clock, sleeper) -> tuple[float, ...]:
    deadline = clock() + plan.timeout_sec
    last: tuple[float, ...] | None = None
    while clock() < deadline:
        last = _angles(robot)
        _require_status(robot)
        if max(abs(a - b) for a, b in zip(last, expected)) <= plan.final_tolerance_deg:
            return last
        sleeper(0.1)
    raise RuntimeError(f"stow_feedback_timeout:expected={expected}:actual={last}")
