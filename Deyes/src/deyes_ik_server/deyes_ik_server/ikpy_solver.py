"""ikpy-backed 7-DOF IK solver for Mercury X1.

We intentionally do NOT derive Jacobians by hand. ikpy's numerical
Levenberg-Marquardt implementation reads URDF joint axes/limits/origins,
gives us back joint angles in radians, and lets us seed with the current
joint pose so we converge in a handful of iterations.

Design notes:
  * ``IkpySolver7DOF`` wraps a single arm chain (right or left).
  * Initial position is taken from the last solved pose, falling back to
    the joint mid-range. This avoids joint jumps between consecutive
    ExecuteCartesianStage calls.
  * Joint limits are read straight from the URDF; ikpy clamps each
    iterate and ``clamp=True`` keeps the final result inside the limits.
  * We expose a structured failure reason so the action server can put
    a stable code in ``failure_code``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


# numpy is heavy; import lazily so the mock solver / urdf loader can be used
# on a Mac dev box without ROS or scientific packages installed.
def _np():
    import numpy as _np  # noqa: WPS433 — local import by design
    return _np


from .urdf_loader import arm_joint_names, resolve_urdf

try:  # ikpy is heavy and optional during offline unit tests
    import ikpy.chain
    _HAS_IKPY = True
except Exception as _exc:  # pragma: no cover - exercised only when ikpy missing
    _HAS_IKPY = False
    _IKPY_IMPORT_ERROR = _exc


def _ensure_ikpy() -> None:
    if not _HAS_IKPY:
        raise RuntimeError(
            "ikpy is not installed. Run `pip install --user ikpy scipy` "
            f"on the venue robot (Python 3.8). Original import error: {_IKPY_IMPORT_ERROR!r}"
        )


# Vendor URDF encodes the right-arm chain as link_body -> joint1_R -> ... -> link7_R.
# ikpy's Chain.from_urdf_file parses the whole robot, so we filter by the
# arm's joint names via ``active_links_mask``.
def _build_active_links_mask(urdf_path: str, side: str) -> list[bool]:
    """Return a per-link mask that activates only this arm's joints."""
    _ensure_ikpy()
    chain = ikpy.chain.Chain.from_urdf_file(urdf_path, last_link_vector=None)
    joint_names = arm_joint_names(side)
    mask: list[bool] = []
    for link in chain.links:
        # ikpy names inactive links after their parent joint; skip the base
        if link.name in joint_names:
            mask.append(True)
        elif link.name in {"base_link", "base_footprint", "link_body"}:
            mask.append(False)
        else:
            mask.append(False)
    if sum(mask) != 7:
        # Fall back to "first N links are arm joints". Better than crashing
        # the field if the URDF is updated upstream.
        return [False] + [True] * 7 + [False] * (len(chain.links) - 8)
    return mask


@dataclass
class IkResult:
    success: bool
    joint_deg: list[float]
    failure_code: str = ""
    iterations: int = 0
    residual_m: float = 0.0


class IkpySolver7DOF:
    """Numerical IK for one 7-DOF Mercury X1 arm."""

    def __init__(self, arm_side: str, urdf_path: Optional[str] = None) -> None:
        self.arm_side = (arm_side or "").strip().lower()
        if self.arm_side not in ("left", "right"):
            raise ValueError(f"arm_side must be 'left' or 'right', got {arm_side!r}")
        self.urdf_path = urdf_path or resolve_urdf()
        _ensure_ikpy()
        mask = _build_active_links_mask(self.urdf_path, self.arm_side)
        self._chain = ikpy.chain.Chain.from_urdf_file(
            self.urdf_path,
            active_links_mask=mask,
            last_link_vector=None,
        )
        self._n_active = sum(mask)
        # Cache the last joint solution so consecutive ExecuteCartesianStage
        # calls can seed the optimizer with the arm's current pose.
        self._last_joint_deg: Optional[list[float]] = None

    # ---- public API -------------------------------------------------

    def solve(
        self,
        pose_base: list[float],
        current_joint_deg: Optional[list[float]] = None,
        *,
        max_iter: int = 60,
        tol_m: float = 1e-3,
    ) -> IkResult:
        """Solve IK for one Cartesian goal.

        Args:
          pose_base: ``[x_m, y_m, z_m, rx_deg, ry_deg, rz_deg]`` (XYZ extrinsic Euler).
          current_joint_deg: optional 7-vector seeding the solver (degrees).
        """
        if len(pose_base) != 6:
            return IkResult(False, [], failure_code="POSE_SHAPE_INVALID")
        target_position = _np().asarray(pose_base[0:3], dtype=float)
        if not _np().all(_np().isfinite(target_position)):
            return IkResult(False, [], failure_code="POSE_NONFINITE")

        target_orientation = self._euler_deg_to_quat(pose_base[3:6])

        initial = self._compose_initial_state(current_joint_deg)
        try:
            joint_full = self._chain.inverse_kinematics(
                target_position=target_position,
                target_orientation=target_orientation,
                initial_position=initial,
                max_iter=max_iter,
            )
        except ValueError as exc:
            return IkResult(False, [], failure_code=f"IK_VALUE_ERROR:{exc}")
        except Exception as exc:  # pragma: no cover - defensive
            return IkResult(False, [], failure_code=f"IK_RUNTIME_ERROR:{exc}")

        # ikpy returns one angle per chain link. The first link is the base
        # (inactive in our mask) so the 7 arm joints sit at indices 1..7.
        active = list(joint_full[1 : 1 + self._n_active])
        # ikpy returns radians; Action contract and pymycobot expect degrees.
        joint_deg = [math.degrees(float(v)) for v in active]

        # Residual check: did we actually reach the target?
        np = _np()
        fk = self._chain.forward_kinematics(joint_full)
        pos_err = float(np.linalg.norm(np.asarray(fk[:3, 3]) - target_position))
        if pos_err > tol_m:
            return IkResult(False, joint_deg,
                            failure_code=f"FK_RESIDUAL_{pos_err:.4f}_M",
                            residual_m=pos_err)

        self._last_joint_deg = joint_deg
        return IkResult(success=True, joint_deg=joint_deg, residual_m=pos_err)

    # ---- helpers ---------------------------------------------------

    def _compose_initial_state(self, current_joint_deg: Optional[list[float]]):
        """Build the (chain.length,) initial vector ikpy expects.

        Index 0 is the (fixed) base link, indices 1..7 are the 7 arm joints.
        ikpy uses radian angles throughout.
        """
        state = _np().zeros(self._chain.length)
        seed = current_joint_deg or self._last_joint_deg
        if seed is None:
            # Reasonable observation pose for Mercury X1 right arm.
            seed = [4.479, 94.999, 4.5, -84.29, 76.824, 92.6, 0.0]
        for i in range(self._n_active):
            state[1 + i] = math.radians(float(seed[i]))
        return state

    @staticmethod
    def _euler_deg_to_quat(euler_xyz_deg: list[float]):
        from scipy.spatial.transform import Rotation as R

        np = _np()
        if not np.all(np.isfinite(euler_xyz_deg)):
            return np.array([0.0, 0.0, 0.0, 1.0])
        return R.from_euler("xyz", euler_xyz_deg, degrees=True).as_quat()  # [x,y,z,w]

    # Convenience: vendor-probed observation pose in degrees.
    OBSERVATION_POSE_RIGHT_DEG = [4.479, 94.999, 4.5, -84.29, 76.824, 92.6, 0.0]