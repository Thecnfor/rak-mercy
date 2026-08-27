"""Pure, latched post-lift grasp verification contract."""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Sequence


@dataclass
class GraspVerifier:
    empty_closed_feedback: float
    latched: bool = False
    result: dict | None = field(default=None, init=False)

    def verify(self, *, pen_height_over_table_m: float | None,
               original_roi_has_pen: Sequence[bool], gripper_feedback: float) -> dict:
        if self.latched:
            return {**self.result, "single_attempt_latched": True}  # type: ignore[arg-type]
        self.latched = True
        condition_a = pen_height_over_table_m is not None and float(pen_height_over_table_m) >= .030
        condition_b = (len(original_roi_has_pen) >= 3 and
                       not any(bool(v) for v in original_roi_has_pen[-3:]) and
                       float(gripper_feedback) - float(self.empty_closed_feedback) >= 5.0)
        success = bool(condition_a or condition_b)
        self.result = {"schema": "competition_grasp_verification/v1", "success": success,
                       "condition_a_lifted_30mm": condition_a,
                       "condition_b_roi_clear_3_and_feedback_delta_5": condition_b,
                       "gripper_feedback_delta": float(gripper_feedback)-float(self.empty_closed_feedback),
                       "navigation_permitted": success, "reason": "ok" if success else "grasp_not_verified",
                       "single_attempt_latched": True}
        return dict(self.result)
