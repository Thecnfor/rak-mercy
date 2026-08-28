import copy
import hashlib
import json
from pathlib import Path

import yaml

from deyes_stereo.competition_clearance_evidence import (
    TARGET_SCOPE,
    canonical_sha256,
    evaluate_profile_clearance,
    motion_contract,
    validate_clearance_evidence,
)


ROOT = Path(__file__).resolve().parents[1]


def _profile():
    return yaml.safe_load((ROOT / "config/stereo/competition_venue_65cm.yaml").read_text())


def _run(clearance=12.0):
    return {
        "passed": True,
        "nav_table_1_reached": True,
        "nav_table_2_reached": True,
        "ik_fk_passed": True,
        "joint_feedback_passed": True,
        "stage_timeouts_passed": True,
        "dynamic_contact_grasp": True,
        "pen_placed_on_table_2": True,
        "forbidden_contacts": [],
        "minimum_arm_raw_mm": clearance + 8.0,
        "minimum_arm_conservative_mm": clearance,
        "minimum_fingertip_table_raw_mm": 2.5,
        "minimum_navigation_raw_mm": 60.0,
        "minimum_navigation_conservative_mm": 52.0,
        "pen_lift_mm": 31.0,
    }


def _evidence(profile=None):
    profile = profile or _profile()
    contract_sha = canonical_sha256(motion_contract(profile))
    return {
        "schema": "isaac_clearance_evidence/v1",
        "passed": True,
        "target_scope": TARGET_SCOPE,
        "motion_contract_sha256": contract_sha,
        "assets": {
            "collision_prim_count": 55,
            "all_required_collisions_enabled": True,
            "scene_usd_sha256": "1" * 64,
            "robot_usd_sha256": "2" * 64,
            "scene_config_sha256": "3" * 64,
        },
        "simulation": {
            "physics_hz": 60.0,
            "synthetic_attachment": False,
            "rigid_body_disabled": False,
            "teleport_used": False,
        },
        "initial_pose": {
            "both_arms_auto_stowed": True,
            "selected_order": "left_then_right",
            "power_on_left_rad": [0.0] * 6,
            "power_on_right_rad": [0.0] * 6,
            "stow_left_rad": [0.1] * 6,
            "stow_right_rad": [-0.1] * 6,
        },
        "runs": [_run(12.0), _run(11.0)],
        "conservative_clearance_mm": 11.0,
    }


def test_valid_evidence_and_tamper_fail_closed(tmp_path):
    profile = _profile()
    evidence = _evidence(profile)
    ok, reason, clearance = validate_clearance_evidence(
        evidence, expected_motion_sha256=canonical_sha256(motion_contract(profile))
    )
    assert (ok, reason, clearance) == (True, "ok", 11.0)
    for mutation, expected in (
        (("assets", "collision_prim_count", 54), "collision_prim_count_mismatch"),
        (("assets", "all_required_collisions_enabled", False), "required_collision_disabled"),
        (("simulation", "synthetic_attachment", True), "synthetic_or_teleport_method_forbidden"),
        (("runs", 0, "forbidden_contacts", ["arm:table"]), "run_1:forbidden_contact_observed"),
        (("runs", 0, "minimum_arm_raw_mm", 17.9), "run_1:arm_clearance_below_threshold"),
        (("runs", 0, "minimum_fingertip_table_raw_mm", 1.9), "run_1:fingertip_clearance_below_threshold"),
        (("runs", 0, "minimum_navigation_raw_mm", 57.9), "run_1:navigation_clearance_below_threshold"),
    ):
        bad = copy.deepcopy(evidence)
        if mutation[0] == "runs":
            bad[mutation[0]][mutation[1]][mutation[2]] = mutation[3]
        else:
            bad[mutation[0]][mutation[1]] = mutation[2]
        assert validate_clearance_evidence(
            bad, expected_motion_sha256=evidence["motion_contract_sha256"]
        )[1] == expected


def test_profile_requires_pinned_file_hash_and_matching_motion(tmp_path):
    profile = _profile()
    evidence = _evidence(profile)
    evidence_path = tmp_path / "competition_isaac_clearance_evidence.json"
    evidence_path.write_text(json.dumps(evidence, sort_keys=True) + "\n")
    profile["transport"].update({
        "transport_validated": True,
        "collision_clearance_validated": True,
        "tcp_vertical_clearance_conservative_mm": 11.0,
        "clearance_evidence_manifest": evidence_path.name,
        "clearance_evidence_sha256": hashlib.sha256(evidence_path.read_bytes()).hexdigest(),
    })
    profile_path = tmp_path / "profile.yaml"
    profile_path.write_text(yaml.safe_dump(profile, sort_keys=False))
    assert evaluate_profile_clearance(profile_path).accepted

    evidence_path.write_text(evidence_path.read_text() + " ")
    assert evaluate_profile_clearance(profile_path).reason == "clearance_evidence_sha256_mismatch"


def test_default_profile_remains_fail_closed_without_real_evidence():
    result = evaluate_profile_clearance(ROOT / "config/stereo/competition_venue_65cm.yaml")
    assert result.accepted is False
    assert result.reason == "clearance_evidence_not_pinned"
