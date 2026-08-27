"""Deterministic mock IK used when ikpy is not available.

The action server picks this at startup if ``ikpy`` cannot be imported
(e.g. before calibration packages the URDF + scipy), or when
``solver_type=mock`` is set in the launch file. This lets the rest of
the pipeline — TF2 publication, ExecuteCartesianStage wiring — be
exercised end-to-end without a real arm.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional

from .ikpy_solver import IkResult


@dataclass
class MockSolver:
    arm_side: str
    observation_pose_deg: Iterable[float]
    x_clip: tuple[float, float] = (-0.6, 0.6)
    y_clip: tuple[float, float] = (-0.6, 0.6)
    z_clip: tuple[float, float] = (0.02, 1.10)

    def solve(
        self,
        pose_base: list[float],
        current_joint_deg: Optional[list[float]] = None,
        *,
        max_iter: int = 60,
        tol_m: float = 1e-3,
    ) -> IkResult:
        if len(pose_base) != 6:
            return IkResult(False, [], failure_code="POSE_SHAPE_INVALID")
        x, y, z, _rx, _ry, _rz = pose_base
        if not (self.x_clip[0] <= x <= self.x_clip[1]
                and self.y_clip[0] <= y <= self.y_clip[1]
                and self.z_clip[0] <= z <= self.z_clip[1]):
            return IkResult(False, list(self.observation_pose_deg),
                            failure_code="MOCK_OUT_OF_BOX")
        # Mock: keep the joint at observation pose but slightly lean toward
        # the target so RViz visualization reflects the goal.
        seed = list(current_joint_deg) if current_joint_deg else list(self.observation_pose_deg)
        if len(seed) < 7:
            seed = list(self.observation_pose_deg) + [0.0] * (7 - len(seed))
        return IkResult(success=True, joint_deg=seed[:7])