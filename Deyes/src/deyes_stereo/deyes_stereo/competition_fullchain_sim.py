"""ROS-free, fail-closed Mercury X1 650 mm competition-chain simulation.

The module owns orchestration only.  A single adapter seam keeps Isaac/ROS and
deterministic fixtures interchangeable while preserving the same ordered
observable phases.  The bundled adapter is explicitly synthetic and can never
upgrade physical calibration or execution claims.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Protocol

from .competition_grasp_verification import GraspVerifier


PHASES = (
    "navigate_table_1",
    "stabilize_table_1",
    "stereo_snapshot",
    "cuda_depth_camera_info",
    "table_plane_audit",
    "single_pen_detection",
    "project_target",
    "validate_ik",
    "pick",
    "verify_grasp",
    "navigate_table_2",
    "place",
    "retreat",
)

FAIL_CLOSED_FAULTS = (
    "navigation_table_1_failed",
    "stale_frame",
    "zero_pen",
    "multiple_pens",
    "invalid_depth",
    "table_plane_conflict",
    "projector_unavailable",
    "target_out_of_bounds",
    "ik_failed",
    "grasp_verification_failed",
    "navigation_table_2_failed",
)


class CompetitionAdapter(Protocol):
    """One deep simulation seam: perform an observable competition phase."""

    synthetic_inputs: bool

    def perform(
        self, phase: str, context: Mapping[str, Any], *, force_fixed_target: bool
    ) -> Mapping[str, Any]: ...


@dataclass(frozen=True)
class VenueTruth:
    table_height_m: float = 0.650
    head_angles_deg: tuple[float, float] = (-54.93, 2.63)
    tool_orientation_deg: tuple[float, float, float] = (179.99, -12.0, 0.0)
    contact_z_m: float = 0.135
    transport_pose_mm_deg: tuple[float, ...] = (
        300.0,
        10.0,
        260.0,
        179.99,
        -12.0,
        0.0,
    )


def _failure(reason: str) -> dict[str, Any]:
    return {"success": False, "reason": reason}


class CompetitionFullChain:
    """Run exactly one competition attempt; any failed phase terminates it."""

    def __init__(
        self,
        adapter: CompetitionAdapter,
        *,
        force_fixed_target: bool = False,
        truth: VenueTruth = VenueTruth(),
    ) -> None:
        self._adapter = adapter
        self._force_fixed_target = bool(force_fixed_target)
        self._truth = truth

    def run(self) -> dict[str, Any]:
        context: dict[str, Any] = {
            "schema": "competition_fullchain_sim/v1",
            "scene": {
                "table_height_m": self._truth.table_height_m,
                "pen_count": 1,
                "override_only": True,
            },
            "venue_truth": {
                "head_angles_deg": list(self._truth.head_angles_deg),
                "tool_orientation_deg": list(self._truth.tool_orientation_deg),
                "contact_z_m": self._truth.contact_z_m,
                "transport_pose_mm_deg": list(self._truth.transport_pose_mm_deg),
            },
        }
        events: list[dict[str, Any]] = []
        for phase in PHASES:
            outcome = dict(
                self._adapter.perform(
                    phase, context, force_fixed_target=self._force_fixed_target
                )
            )
            success = outcome.pop("success", False) is True
            reason = str(outcome.pop("reason", "ok" if success else "phase_failed"))
            if phase == "verify_grasp" and success:
                verification = outcome.get("grasp_verification")
                if (
                    not isinstance(verification, Mapping)
                    or verification.get("success") is not True
                    or verification.get("navigation_permitted") is not True
                ):
                    success = False
                    reason = "grasp_not_verified"
            event = {
                "phase": phase,
                "attempt": 1,
                "status": "passed" if success else "failed",
                "reason": reason,
                "evidence": outcome,
            }
            events.append(event)
            if success:
                context.update(outcome)
                continue
            return {
                **context,
                "state": "failed",
                "reason": reason,
                "failed_phase": phase,
                "events": events,
                "retry_count": 0,
                "synthetic_inputs": bool(self._adapter.synthetic_inputs),
                "physical_validated": False,
                "hardware_commands_emitted": False,
            }
        return {
            **context,
            "state": "completed",
            "reason": "ok",
            "events": events,
            "retry_count": 0,
            "synthetic_inputs": bool(self._adapter.synthetic_inputs),
            "physical_validated": False,
            "hardware_commands_emitted": False,
        }


class SimulatedCompetitionAdapter:
    """Deterministic fixture adapter with auditable, simulation-only evidence."""

    synthetic_inputs = True

    _FAULT_PHASE = {
        "navigation_table_1_failed": "navigate_table_1",
        "stale_frame": "stereo_snapshot",
        "invalid_depth": "cuda_depth_camera_info",
        "table_plane_conflict": "table_plane_audit",
        "zero_pen": "single_pen_detection",
        "multiple_pens": "single_pen_detection",
        "projector_unavailable": "project_target",
        "target_out_of_bounds": "project_target",
        "ik_failed": "validate_ik",
        "grasp_verification_failed": "verify_grasp",
        "navigation_table_2_failed": "navigate_table_2",
    }

    def __init__(self, *, fault: str | None = None) -> None:
        self.fault = fault
        self.calls: list[str] = []
        self._verifier = GraspVerifier(empty_closed_feedback=10.0)

    def perform(
        self, phase: str, context: Mapping[str, Any], *, force_fixed_target: bool
    ) -> Mapping[str, Any]:
        self.calls.append(phase)
        if self._FAULT_PHASE.get(self.fault) == phase:
            if self.fault == "projector_unavailable" and force_fixed_target:
                return self._forced_target()
            return self._fault_result()
        handlers = {
            "navigate_table_1": lambda: {"navigation_table_1": {"action": "/navigate_to_pose", "result": "succeeded"}},
            "stabilize_table_1": lambda: {"stability": {"duration_sec": 0.5, "linear_speed_mps": 0.0, "angular_speed_radps": 0.0}},
            "stereo_snapshot": lambda: {"snapshot": {"stamp_ns": 1_787_842_935_000_000_000, "source": "synthetic_fixture", "rgb_synthetic": True, "stereo_pair_exact": True, "fresh": True}},
            "cuda_depth_camera_info": lambda: {"perception": {"depth_backend": "synthetic_metric_depth_fixture", "camera_info_corrected": True, "stamp_match": True, "depth_valid": True}},
            "table_plane_audit": self._plane_audit,
            "single_pen_detection": lambda: {"detection": {"backend": "synthetic_yolo_contract_fixture", "candidate_count": 1, "selection": "pen_feature_midpoint", "not_visual_measurement": True}},
            "project_target": self._projected_target,
            "validate_ik": lambda: {"trajectory": {"transport": {"pose_mm_deg": [300.0, 10.0, 260.0, 179.99, -12.0, 0.0], "six_axis": True, "position_residual_mm": 0.001, "joint_limits_passed": True, "minimum_clearance_mm": 50.0}}},
            "pick": lambda: {"pick_trace": {"z_sequence_mm": [235.0, 180.0, 140.0, 135.0, 180.0, 235.0], "orientation_deg": [179.99, -12.0, 0.0], "gripper_open": 70, "gripper_closed": 0, "feedback_converged": True}},
            "verify_grasp": self._verified_grasp,
            "navigate_table_2": lambda: {"navigation_table_2": {"action": "/navigate_to_pose", "result": "succeeded", "grasp_gate_observed": True}},
            "place": lambda: {"place_trace": {"z_sequence_mm": [200.0, 165.0], "gripper_release": 70, "feedback_converged": True}},
            "retreat": lambda: {"retreat_trace": {"z_sequence_mm": [200.0, 260.0], "feedback_converged": True}},
        }
        evidence = dict(handlers[phase]())
        return {"success": True, "reason": "ok", **evidence}

    def _fault_result(self) -> Mapping[str, Any]:
        reasons = {
            "navigation_table_1_failed": "navigation_table_1_failed",
            "stale_frame": "snapshot_stale",
            "invalid_depth": "depth_invalid",
            "table_plane_conflict": "table_height_deviation_exceeds_25mm",
            "zero_pen": "expected_one_pen_got_0",
            "multiple_pens": "expected_one_pen_got_2",
            "projector_unavailable": "projector_unavailable",
            "target_out_of_bounds": "target_outside_workspace",
            "ik_failed": "ik_residual_or_limits_or_clearance_failed",
            "grasp_verification_failed": "grasp_not_verified",
            "navigation_table_2_failed": "navigation_table_2_failed",
        }
        return _failure(reasons.get(self.fault, "injected_failure"))

    def _plane_audit(self) -> Mapping[str, Any]:
        if self.fault == "plane_missing":
            return {
                "plane_audit": {
                    "state": "fixed_height_unverified",
                    "warning": "plane_missing_using_fixed_650mm",
                    "expected_distance_m": 0.469428925,
                    "measured_distance_m": None,
                }
            }
        if self.fault == "plane_low_quality":
            return {
                "plane_audit": {
                    "state": "fixed_height_unverified",
                    "warning": "plane_low_quality_using_fixed_650mm",
                    "expected_distance_m": 0.469428925,
                    "measured_distance_m": 0.471,
                    "residual_rms_m": 0.030,
                }
            }
        return {
            "plane_audit": {
                "state": "fixed_height_verified",
                "expected_distance_m": 0.469428925,
                "measured_distance_m": 0.469428925,
                "residual_rms_m": 0.002,
            }
        }

    @staticmethod
    def _projected_target() -> Mapping[str, Any]:
        return {
            "target": {
                "right_arm_sdk_target_m": [0.400, 0.010, 0.135],
                "selection_source": "pen_feature_midpoint",
                "force_fixed_target": False,
                "simulation_validated": True,
                "physical_validated": False,
            }
        }

    @staticmethod
    def _forced_target() -> Mapping[str, Any]:
        return {
            "success": True,
            "reason": "explicit_force_fixed_target",
            "target": {
                "right_arm_sdk_target_m": [0.400, 0.010, 0.135],
                "selection_source": "forced_fixed_xy",
                "force_fixed_target": True,
                "simulation_validated": True,
                "physical_validated": False,
            },
        }

    def _verified_grasp(self) -> Mapping[str, Any]:
        return {
            "grasp_verification": self._verifier.verify(
                pen_height_over_table_m=0.031,
                original_roi_has_pen=[True, True, True],
                gripper_feedback=10.0,
            )
        }
