"""Fail-closed contracts for one-pen Isaac motion.

This module deliberately does not contain IK.  Isaac motion is permitted only
when an external, reviewed IK/planning step supplies six joint targets for
each arm phase and the target remains bound to one observation and scene.
"""
from __future__ import annotations

import math
from typing import Any, Mapping

SCHEMA = "isaac_single_pen_plan/v1"
PHASES = ("pregrasp", "approach", "close", "lift", "return", "release")


def stage_feedback_converged(*, phase: str, target_arm_rad: Any = None, observed_arm_rad: Any = None,
                             target_gripper: float | None = None, observed_gripper: float | None = None,
                             feedback_age_sec: float, timeout_sec: float = 1.0,
                             arm_tolerance_rad: float = 0.03, gripper_tolerance: float = 0.02) -> tuple[bool, str]:
    """Barrier predicate: a stage may advance only after fresh settled feedback."""
    if phase not in PHASES:
        return False, "unknown_phase"
    if not math.isfinite(float(feedback_age_sec)) or feedback_age_sec < 0 or feedback_age_sec > timeout_sec:
        return False, "feedback_stale_or_timeout"
    if phase in {"close", "release"}:
        if target_gripper is None or observed_gripper is None or abs(float(target_gripper) - float(observed_gripper)) > gripper_tolerance:
            return False, "gripper_not_converged"
        return True, "ok"
    if not _finite_vector(target_arm_rad, 6) or not _finite_vector(observed_arm_rad, 6):
        return False, "arm_feedback_invalid"
    if max(abs(float(a) - float(b)) for a, b in zip(target_arm_rad, observed_arm_rad)) > arm_tolerance_rad:
        return False, "arm_not_converged"
    return True, "ok"


def _finite_vector(value: Any, size: int) -> bool:
    try:
        values = tuple(float(item) for item in value)
    except (TypeError, ValueError):
        return False
    return len(values) == size and all(math.isfinite(item) for item in values)


def validate_single_pen_candidate(candidate: Mapping[str, Any], *, expected_scene_sha256: str, now_stamp_ns: int, max_age_ns: int = 500_000_000) -> tuple[bool, str]:
    """Validate exactly one Isaac candidate without upgrading it to physical evidence."""
    if candidate.get("source") != "isaac_sim" or candidate.get("simulation_validated") is not True:
        return False, "candidate_not_isaac_simulation_validated"
    if candidate.get("physical_validated") is not False or candidate.get("physical_execution_eligible") is not False:
        return False, "physical_claim_forbidden"
    if candidate.get("candidate_count") != 1 or len(candidate.get("candidates", ())) != 1:
        return False, "exactly_one_candidate_required"
    scene = candidate.get("scene_sha256") or candidate.get("simulation", {}).get("scene_sha256")
    if scene != expected_scene_sha256:
        return False, "scene_sha256_mismatch"
    try:
        stamp = int(candidate.get("stamp_ns", int(candidate.get("stamp_sec", 0)) * 1_000_000_000 + int(candidate.get("stamp_nanosec", 0))))
    except (TypeError, ValueError):
        return False, "candidate_stamp_invalid"
    if stamp <= 0 or now_stamp_ns < stamp or now_stamp_ns - stamp > max_age_ns:
        return False, "candidate_stale"
    item = candidate["candidates"][0]
    if not isinstance(item, Mapping) or not item.get("target_id") or item.get("target_frame") != "base_link":
        return False, "candidate_identity_invalid"
    if not _finite_vector(item.get("grasp_point_base_m"), 3):
        return False, "grasp_point_invalid"
    return True, "ok"


def build_single_pen_motion_plan(candidate: Mapping[str, Any], *, scene_sha256: str, now_stamp_ns: int, ik_targets: Mapping[str, Any], max_age_ns: int = 500_000_000) -> dict[str, Any]:
    """Bind externally supplied IK targets to one candidate and scene."""
    valid, reason = validate_single_pen_candidate(candidate, expected_scene_sha256=scene_sha256, now_stamp_ns=now_stamp_ns, max_age_ns=max_age_ns)
    if not valid:
        return {"schema": SCHEMA, "state": "rejected", "reason": reason, "commands_emitted": False}
    missing = [phase for phase in PHASES if phase not in ik_targets]
    if missing:
        return {"schema": SCHEMA, "state": "rejected", "reason": "ik_targets_missing:" + ",".join(missing), "commands_emitted": False}
    for phase in PHASES:
        target = ik_targets[phase]
        if phase in {"close", "release"}:
            if not isinstance(target, Mapping) or not _finite_vector(target.get("right_gripper"), 1):
                return {"schema": SCHEMA, "state": "rejected", "reason": f"ik_gripper_target_invalid:{phase}", "commands_emitted": False}
        elif not isinstance(target, Mapping) or not _finite_vector(target.get("right_arm_rad"), 6):
            return {"schema": SCHEMA, "state": "rejected", "reason": f"ik_right_arm_target_invalid:{phase}", "commands_emitted": False}
    item = candidate["candidates"][0]
    stamp = int(candidate.get("stamp_ns", int(candidate.get("stamp_sec", 0)) * 1_000_000_000 + int(candidate.get("stamp_nanosec", 0))))
    return {"schema": SCHEMA, "state": "ready", "source": "isaac_sim", "simulation_only": True,
            "commands_emitted": False, "target_id": str(item["target_id"]), "stamp_ns": stamp,
            "scene_sha256": scene_sha256, "steps": [{"phase": phase, **dict(ik_targets[phase])} for phase in PHASES]}


def evaluate_pen_lift(*, before_z_m: float | None, after_z_m: float | None, threshold_m: float = 0.03) -> tuple[bool, str]:
    """Require the observed pen rigid body to rise, not merely a close result."""
    if before_z_m is None or after_z_m is None or not math.isfinite(before_z_m) or not math.isfinite(after_z_m):
        return False, "pen_pose_observation_missing"
    if after_z_m - before_z_m < float(threshold_m):
        return False, "pen_lift_threshold_not_met"
    return True, "ok"
