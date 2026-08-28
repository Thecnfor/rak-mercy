"""Hash-bound Isaac clearance evidence admission for the 650 mm venue profile.

This module is deliberately ROS- and hardware-free.  Production executors use
it before importing ``pymycobot`` so a hand-edited YAML boolean can never open
the serial port.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping


SCHEMA = "isaac_clearance_evidence/v1"
TARGET_SCOPE = "fixed_target_400_10_only"
EXPECTED_TARGET_XY_MM = (400.0, 10.0)
EXPECTED_ORIENTATION_DEG = (179.99, -12.0, 0.0)
EXPECTED_PICK_Z_MM = (235.0, 180.0, 140.0, 135.0, 180.0, 235.0)
EXPECTED_PLACE_Z_MM = (200.0, 165.0, 200.0, 260.0)
EXPECTED_TRANSPORT_POSE = (300.0, 10.0, 260.0, *EXPECTED_ORIENTATION_DEG)
EXPECTED_SOURCE_COLLISION_PRIMS = 55
EXPECTED_ACTIVE_COLLISION_PRIMS = 53
EXPECTED_REMOVED_COLLISION_PRIMS = (
    "/World/Pens/table_1_pen_3/CollisionAndFallbackVisual",
    "/World/Pens/table_1_pen_4/CollisionAndFallbackVisual",
)
MIN_ARM_CONSERVATIVE_MM = 10.0
MIN_FINGERTIP_RAW_MM = 2.0
MIN_NAV_CONSERVATIVE_MM = 50.0
MAX_REPEAT_DELTA_MM = 2.0


@dataclass(frozen=True)
class ClearanceAdmission:
    accepted: bool
    reason: str
    evidence_path: str | None = None
    evidence_sha256: str | None = None
    motion_contract_sha256: str | None = None
    conservative_clearance_mm: float = 0.0


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(value: Mapping[str, Any]) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def motion_contract(profile: Mapping[str, Any]) -> dict[str, Any]:
    motion = profile.get("motion") if isinstance(profile.get("motion"), Mapping) else {}
    transport = profile.get("transport") if isinstance(profile.get("transport"), Mapping) else {}
    return {
        "schema": "competition_motion_contract/v1",
        "target_scope": TARGET_SCOPE,
        "target_xy_mm": list(EXPECTED_TARGET_XY_MM),
        "table_height_mm": float(profile.get("table_height_m", float("nan"))) * 1000.0,
        "orientation_deg": [float(v) for v in profile.get("orientation_deg", ())],
        "pick_z_mm": [
            float(motion.get("high_z_mm", float("nan"))),
            float(motion.get("pregrasp_z_mm", float("nan"))),
            float(motion.get("approach_z_mm", float("nan"))),
            float(motion.get("contact_z_mm", float("nan"))),
            *[float(v) for v in motion.get("lift_z_mm", ())],
        ],
        "place_z_mm": [
            float(motion.get("place_pre_z_mm", float("nan"))),
            float(motion.get("release_z_mm", float("nan"))),
            *[float(v) for v in motion.get("retreat_z_mm", ())],
        ],
        "transport_pose_mm_deg": [float(v) for v in transport.get("pose_mm_deg", ())],
        "max_speed": int(motion.get("max_speed", -1)),
        "terminal_max_speed": int(motion.get("terminal_max_speed", -1)),
    }


def validate_motion_contract(contract: Mapping[str, Any]) -> tuple[bool, str]:
    checks = (
        (contract.get("schema") == "competition_motion_contract/v1", "motion_schema_mismatch"),
        (contract.get("target_scope") == TARGET_SCOPE, "target_scope_mismatch"),
        (_close_vector(contract.get("target_xy_mm"), EXPECTED_TARGET_XY_MM), "target_xy_mismatch"),
        (_close(float(contract.get("table_height_mm", float("nan"))), 650.0), "table_height_mismatch"),
        (_close_vector(contract.get("orientation_deg"), EXPECTED_ORIENTATION_DEG), "orientation_mismatch"),
        (_close_vector(contract.get("pick_z_mm"), EXPECTED_PICK_Z_MM), "pick_z_sequence_mismatch"),
        (_close_vector(contract.get("place_z_mm"), EXPECTED_PLACE_Z_MM), "place_z_sequence_mismatch"),
        (_close_vector(contract.get("transport_pose_mm_deg"), EXPECTED_TRANSPORT_POSE), "transport_pose_mismatch"),
        (contract.get("max_speed") == 8, "max_speed_mismatch"),
        (contract.get("terminal_max_speed") == 5, "terminal_speed_mismatch"),
    )
    for accepted, reason in checks:
        if not accepted:
            return False, reason
    return True, "ok"


def evaluate_profile_clearance(profile_path: Path) -> ClearanceAdmission:
    try:
        import yaml

        profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
        if not isinstance(profile, Mapping):
            return ClearanceAdmission(False, "venue_profile_not_mapping")
        contract = motion_contract(profile)
        valid, reason = validate_motion_contract(contract)
        if not valid:
            return ClearanceAdmission(False, reason)
        contract_sha = canonical_sha256(contract)
        transport = profile.get("transport")
        if not isinstance(transport, Mapping):
            return ClearanceAdmission(False, "transport_profile_missing", motion_contract_sha256=contract_sha)
        evidence_ref = transport.get("clearance_evidence_manifest")
        expected_sha = str(transport.get("clearance_evidence_sha256") or "").lower()
        if not evidence_ref or len(expected_sha) != 64:
            return ClearanceAdmission(False, "clearance_evidence_not_pinned", motion_contract_sha256=contract_sha)
        evidence_path = Path(str(evidence_ref))
        if not evidence_path.is_absolute():
            evidence_path = profile_path.parent / evidence_path
        evidence_path = evidence_path.resolve()
        if not evidence_path.is_file():
            return ClearanceAdmission(False, "clearance_evidence_missing", str(evidence_path), motion_contract_sha256=contract_sha)
        actual_sha = sha256_file(evidence_path)
        if actual_sha != expected_sha:
            return ClearanceAdmission(False, "clearance_evidence_sha256_mismatch", str(evidence_path), actual_sha, contract_sha)
        evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
        valid, reason, clearance = validate_clearance_evidence(evidence, expected_motion_sha256=contract_sha)
        if not valid:
            return ClearanceAdmission(False, reason, str(evidence_path), actual_sha, contract_sha)
        declared = float(transport.get("tcp_vertical_clearance_conservative_mm", 0.0))
        if not _close(declared, clearance):
            return ClearanceAdmission(False, "profile_clearance_does_not_match_evidence", str(evidence_path), actual_sha, contract_sha)
        booleans = (
            transport.get("transport_validated") is True,
            transport.get("collision_clearance_validated") is True,
            transport.get("kinematics_validated") is True,
            transport.get("joint_limits_passed") is True,
        )
        if not all(booleans):
            return ClearanceAdmission(False, "profile_validation_flags_closed", str(evidence_path), actual_sha, contract_sha)
        return ClearanceAdmission(True, "ok", str(evidence_path), actual_sha, contract_sha, clearance)
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError, AttributeError) as exc:
        return ClearanceAdmission(False, f"clearance_evidence_invalid:{type(exc).__name__}")


def validate_clearance_evidence(
    evidence: Mapping[str, Any], *, expected_motion_sha256: str
) -> tuple[bool, str, float]:
    if evidence.get("schema") != SCHEMA:
        return False, "clearance_evidence_schema_mismatch", 0.0
    if evidence.get("passed") is not True or evidence.get("target_scope") != TARGET_SCOPE:
        return False, "clearance_evidence_not_passed_or_wrong_scope", 0.0
    if evidence.get("motion_contract_sha256") != expected_motion_sha256:
        return False, "motion_contract_sha256_mismatch", 0.0
    assets = evidence.get("assets")
    if not isinstance(assets, Mapping):
        return False, "asset_evidence_missing", 0.0
    if assets.get("all_required_collisions_enabled") is not True:
        return False, "required_collision_disabled", 0.0
    for key in ("scene_usd_sha256", "source_scene_usd_sha256", "robot_usd_sha256", "scene_config_sha256"):
        if len(str(assets.get(key) or "")) != 64:
            return False, f"asset_sha_missing:{key}", 0.0
    simulation = evidence.get("simulation")
    if not isinstance(simulation, Mapping) or not _close(float(simulation.get("physics_hz", 0.0)), 60.0):
        return False, "physics_rate_mismatch", 0.0
    if simulation.get("synthetic_attachment") is not False or simulation.get("rigid_body_disabled") is not False or simulation.get("teleport_used") is not False:
        return False, "synthetic_or_teleport_method_forbidden", 0.0
    initial = evidence.get("initial_pose")
    if not isinstance(initial, Mapping) or initial.get("both_arms_auto_stowed") is not True:
        return False, "both_arm_stow_not_verified", 0.0
    if initial.get("selected_order") not in ("left_then_right", "right_then_left"):
        return False, "stow_order_invalid", 0.0
    for name in ("power_on_left_rad", "power_on_right_rad", "stow_left_rad", "stow_right_rad"):
        if not _finite_vector(initial.get(name), 6):
            return False, f"stow_vector_invalid:{name}", 0.0
    runs = evidence.get("runs")
    if not isinstance(runs, list) or len(runs) != 2:
        return False, "exactly_two_acceptance_runs_required", 0.0
    minima: list[float] = []
    for index, run in enumerate(runs):
        valid, reason, clearance = _validate_run(run)
        if not valid:
            return False, f"run_{index + 1}:{reason}", 0.0
        minima.append(clearance)
    if abs(minima[0] - minima[1]) > MAX_REPEAT_DELTA_MM:
        return False, "repeat_clearance_delta_exceeds_2mm", 0.0
    declared = float(evidence.get("conservative_clearance_mm", float("nan")))
    clearance = min(minima)
    if not _close(declared, clearance) or clearance < MIN_ARM_CONSERVATIVE_MM:
        return False, "conservative_clearance_summary_invalid", 0.0
    return True, "ok", clearance


def _validate_run(run: Any) -> tuple[bool, str, float]:
    if not isinstance(run, Mapping) or run.get("passed") is not True:
        return False, "run_not_passed", 0.0
    required_true = (
        "nav_table_1_reached", "nav_table_2_reached", "ik_fk_passed",
        "joint_feedback_passed", "stage_timeouts_passed", "dynamic_contact_grasp",
        "pen_placed_on_table_2",
    )
    for key in required_true:
        if run.get(key) is not True:
            return False, f"{key}_not_true", 0.0
    contacts = run.get("forbidden_contacts")
    if contacts != []:
        return False, "forbidden_contact_observed", 0.0
    try:
        arm_raw = float(run["minimum_arm_raw_mm"])
        arm_conservative = float(run["minimum_arm_conservative_mm"])
        finger_raw = float(run["minimum_fingertip_table_raw_mm"])
        nav_raw = float(run["minimum_navigation_raw_mm"])
        nav_conservative = float(run["minimum_navigation_conservative_mm"])
        lift = float(run["pen_lift_mm"])
    except (KeyError, TypeError, ValueError):
        return False, "clearance_metrics_invalid", 0.0
    if not all(math.isfinite(v) for v in (arm_raw, arm_conservative, finger_raw, nav_raw, nav_conservative, lift)):
        return False, "clearance_metrics_nonfinite", 0.0
    if arm_raw < 18.0 or arm_conservative < MIN_ARM_CONSERVATIVE_MM:
        return False, "arm_clearance_below_threshold", 0.0
    if finger_raw < MIN_FINGERTIP_RAW_MM:
        return False, "fingertip_clearance_below_threshold", 0.0
    if nav_raw < 58.0 or nav_conservative < MIN_NAV_CONSERVATIVE_MM:
        return False, "navigation_clearance_below_threshold", 0.0
    if lift < 30.0:
        return False, "pen_lift_below_30mm", 0.0
    return True, "ok", arm_conservative


def _close(a: float, b: float, tolerance: float = 1e-6) -> bool:
    return math.isfinite(a) and math.isfinite(b) and abs(a - b) <= tolerance


def _close_vector(value: Any, expected: tuple[float, ...]) -> bool:
    try:
        values = tuple(float(v) for v in value)
    except (TypeError, ValueError):
        return False
    return len(values) == len(expected) and all(_close(a, b) for a, b in zip(values, expected))


def _finite_vector(value: Any, size: int) -> bool:
    try:
        values = tuple(float(v) for v in value)
    except (TypeError, ValueError):
        return False
    return len(values) == size and all(math.isfinite(v) for v in values)
