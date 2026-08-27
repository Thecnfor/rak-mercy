"""CLI smoke test for IkpySolver7DOF.

Usage (after `colcon build`):

    ros2 run deyes_ik_server ikpy_solver_smoketest \
        --arm-side right \
        --pose 0.30 -0.10 0.18 90 0 90

It loads the URDF, solves the requested pose, prints the joint angles
in degrees, then re-runs forward kinematics to report residual error.
"""

from __future__ import annotations

import argparse
import math
import sys

from .ikpy_solver import IkpySolver7DOF


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--arm-side", default="right", choices=["left", "right"])
    p.add_argument("--urdf", default=None)
    p.add_argument("--pose", nargs=6, type=float, required=True,
                   metavar=("X_M", "Y_M", "Z_M", "RX_DEG", "RY_DEG", "RZ_DEG"))
    p.add_argument("--seed", nargs=6, type=float, default=None,
                   help="optional 6-element joint seed in degrees")
    args = p.parse_args()

    try:
        solver = IkpySolver7DOF(args.arm_side, urdf_path=args.urdf)
    except Exception as exc:
        print(f"[smoketest] failed to build solver: {exc}", file=sys.stderr)
        return 2

    seed = args.seed
    res = solver.solve(args.pose, seed)
    print(f"[smoketest] success={res.success} code={res.failure_code} "
          f"residual_m={res.residual_m:.4f}")
    deg = res.joint_deg
    print(f"[smoketest] joint_deg={[round(v, 2) for v in deg]}")
    return 0 if res.success else 1


if __name__ == "__main__":
    sys.exit(main())