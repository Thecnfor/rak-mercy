from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "deyes_stereo"))

from deyes_stereo.competition_showcase_contract import (  # noqa: E402
    build_transaction_result,
    build_showcase_target,
    decide_pick_continuation,
)


def test_runtime_perception_failure_builds_truthful_fixed_marker_showcase_target() -> None:
    target = build_showcase_target("target_timeout")

    assert target == {
        "schema": "competition_showcase_target/v1",
        "sensor_target_available": False,
        "synthetic_target": True,
        "degraded": True,
        "degraded_reason": "target_timeout",
        "target_xy_mm": [400.0, 10.0],
        "right_arm_sdk_target_m": [0.4, 0.01, 0.135],
        "orientation_deg": [179.99, -12.0, 0.0],
        "commands_emitted": False,
    }
    assert "trusted_for_venue_execution" not in target
    assert "force_fixed_target" not in target


def test_object_not_verified_continues_only_as_truthful_showcase() -> None:
    result = {
        "schema": "competition_grasp_verification/v1",
        "success": False,
        "navigation_permitted": False,
        "motion_completed": True,
        "transport_pose_reached": True,
        "hardware_ok": True,
        "object_grasp_verified": False,
        "verification_failure_class": "object_absent",
        "reason": "grasp_not_verified",
    }

    assert decide_pick_continuation(result, showcase_enabled=True) == {
        "action": "continue_showcase",
        "competition_success": False,
        "object_grasp_verified": False,
        "degraded_reason": "grasp_not_verified",
    }
    assert result["navigation_permitted"] is False


def test_verified_pick_preserves_normal_competition_success() -> None:
    result = {
        "schema": "competition_grasp_verification/v1",
        "success": True,
        "navigation_permitted": True,
        "motion_completed": True,
        "transport_pose_reached": True,
        "hardware_ok": True,
        "object_grasp_verified": True,
        "verification_failure_class": None,
        "reason": "ok",
    }

    decision = decide_pick_continuation(result, showcase_enabled=True)
    assert decision["action"] == "continue_verified"
    assert decision["competition_success"] is True
    assert decision["degraded_reason"] is None


def test_strict_mode_and_hardware_faults_never_continue() -> None:
    unverified = {
        "schema": "competition_grasp_verification/v1",
        "success": False,
        "navigation_permitted": False,
        "motion_completed": True,
        "transport_pose_reached": True,
        "hardware_ok": True,
        "object_grasp_verified": False,
        "verification_failure_class": "perception",
        "reason": "roi_feedback_timeout",
    }
    assert decide_pick_continuation(unverified, showcase_enabled=False)["action"] == "stop"

    hardware_fault = {**unverified, "hardware_ok": False, "reason": "serial_feedback_failed"}
    assert decide_pick_continuation(hardware_fault, showcase_enabled=True)["action"] == "stop"


def test_completed_empty_showcase_has_dual_truthful_status() -> None:
    result = build_transaction_result(
        transaction_id="tx-1",
        showcase_mode=True,
        competition_success=False,
        showcase_complete=True,
        object_grasp_verified=False,
        target_source="fixed_marker_showcase",
        pick_motion_completed=True,
        transport_pose_reached=True,
        goal4_completed=True,
        place_motion_completed=True,
        degraded_reasons=["target_timeout", "grasp_not_verified"],
        hard_stop_reason=None,
    )

    assert result["schema"] == "competition_transaction_result/v1"
    assert result["competition_success"] is False
    assert result["showcase_complete"] is True
    assert result["object_grasp_verified"] is False
    assert result["commands_emitted"] is True
    assert result["hard_stop_reason"] is None
