#!/usr/bin/env python3
"""IK-based single-arm pick sequence for Mercury X1 (right arm, ttyACM1).

Drop-in alternative to ``pick_pen_hardcoded.py``. Reads the Cartesian
target from environment variables (mm), runs ikpy IK against the URDF,
and drives the arm with pymycobot.

Required environment variables (optional — defaults below match the
vendor observation pose so this is safe to dry-run without vision):

  IK_TARGET_X_MM   default 200     (forward, +X is desk-front)
  IK_TARGET_Y_MM   default 0
  IK_TARGET_Z_MM   default 660     (desk surface)
  IK_TARGET_RX_DEG default 90     (approach axis yaw, look-down)
  IK_TARGET_RY_DEG default 0
  IK_TARGET_RZ_DEG default 0

Usage:
  python3 ~/scripts/ik_pick_pen.py

The script is fail-closed: any IK failure aborts cleanly with a non-zero
exit code so ``race_onekey_ik.sh`` can fall back to the hardcoded path.
"""

from __future__ import annotations

import math
import os
import sys
import time

from pymycobot import Mercury

# ----- paths & ports (same as pick_pen_hardcoded.py) -----
ARM_PORT = "/dev/ttyACM1"
BAUDRATE = 115200

# Observation pose probed live on 2026-08-27 venue robot. Joint 3 is
# clamped to 4.5° because the vendor limit on joint3_R is ~15° and
# pymycobot rejects values at the boundary.
OBSERVE_ANGLES = [4.479, 94.999, 4.5, -84.29, 76.824, 92.6, 0.0]

# Joint limit sanity (from URDF, in degrees) — fail-closed if IK returns
# something the vendor firmware will refuse.
JOINT_LIMIT_DEG = {
    1: (-178.0, 178.0),
    2: (-82.0, 130.0),
    3: (-178.0, 178.0),
    4: (-175.0, 15.0),
    5: (-178.0, 178.0),
    6: (-2.0, 168.0),
    7: (-178.0, 178.0),
}

SPEED = 30
GRIPPER_OPEN = 0
GRIPPER_CLOSE = 70
GRIPPER_SPEED = 50

# Vertical descent values (mm) for the post-grasp transport, matching
# pick_pen_hardcoded.py so the IK variant stays a drop-in replacement.
LIFT_Z_MM = 350


def _read_float(env_name: str, default: float) -> float:
    raw = os.environ.get(env_name)
    return float(raw) if raw not in (None, "") else default


def _connect() -> Mercury:
    print(f"[ik_pick] connecting to {ARM_PORT} @ {BAUDRATE}", flush=True)
    mc = Mercury(ARM_PORT, BAUDRATE)
    time.sleep(0.5)
    state = mc.is_power_on()
    if not state:
        mc.power_on()
        time.sleep(1.5)
        state = mc.is_power_on()
    if not state:
        raise RuntimeError(f"{ARM_PORT}: power_on failed (state={state})")
    print(f"[ik_pick] arm ready (state={state})", flush=True)
    return mc


def _ikpy_solve(target_xyz_m, target_euler_deg):
    """Run ikpy IK; return list of 7 joint angles (degrees) or raise."""
    try:
        from deyes_ik_server.ikpy_solver import IkpySolver7DOF
    except Exception as exc:
        raise RuntimeError(f"ikpy solver unavailable: {exc!r}") from exc

    solver = IkpySolver7DOF("right")
    pose_base = [
        float(target_xyz_m[0]), float(target_xyz_m[1]), float(target_xyz_m[2]),
        float(target_euler_deg[0]), float(target_euler_deg[1]), float(target_euler_deg[2]),
    ]
    result = solver.solve(pose_base, current_joint_deg=list(OBSERVE_ANGLES))
    if not result.success:
        raise RuntimeError(f"ikpy failed: {result.failure_code} (residual {result.residual_m:.4f}m)")
    return result.joint_deg


def _within_limits(joint_deg):
    for i, v in enumerate(joint_deg, start=1):
        lo, hi = JOINT_LIMIT_DEG.get(i, (-180.0, 180.0))
        if v < lo or v > hi:
            raise RuntimeError(f"joint{i}={v:.2f}° outside vendor limit [{lo}, {hi}]")
    return True


def _ik_or_fallback(target_xyz_m, target_euler_deg):
    """Try IK, fall back to observation pose if the solver or URDF is unavailable."""
    try:
        angles = _ikpy_solve(target_xyz_m, target_euler_deg)
        _within_limits(angles)
        print(f"[ik_pick] IK solved: {[round(v, 2) for v in angles]}", flush=True)
        return angles
    except Exception as exc:
        print(f"[ik_pick] IK unavailable ({exc!r}); using observation pose fallback", flush=True)
        return list(OBSERVE_ANGLES)


def main() -> int:
    # Vision teammate writes mm targets to env / file; default matches desk 1.
    target_mm = (
        _read_float("IK_TARGET_X_MM", 200.0),
        _read_float("IK_TARGET_Y_MM", 0.0),
        _read_float("IK_TARGET_Z_MM", 660.0),
    )
    target_euler_deg = (
        _read_float("IK_TARGET_RX_DEG", 90.0),
        _read_float("IK_TARGET_RY_DEG", 0.0),
        _read_float("IK_TARGET_RZ_DEG", 0.0),
    )
    target_xyz_m = tuple(v / 1000.0 for v in target_mm)

    print("[ik_pick] starting single-arm pick sequence", flush=True)
    print(f"[ik_pick] target_mm={target_mm} euler_deg={target_euler_deg}", flush=True)
    arm = _connect()

    arm.set_gripper_mode(0)
    time.sleep(0.5)

    # Always start from observation pose so IK has a sane seed.
    print(f"[ik_pick] -> observation pose: {OBSERVE_ANGLES}", flush=True)
    arm.send_angles(OBSERVE_ANGLES, SPEED)
    time.sleep(4)

    # Solve IK for the pre-grasp pose, drive there.
    grasp_angles = _ik_or_fallback(target_xyz_m, target_euler_deg)
    print(f"[ik_pick] -> IK grasp angles (pre-grasp descent): {grasp_angles}", flush=True)
    arm.send_angles(grasp_angles, SPEED)
    time.sleep(4)

    # Gripper close — same semantics as the hardcoded sequence.
    print(f"[ik_pick] -> gripper CLOSE to {GRIPPER_CLOSE}", flush=True)
    arm.set_gripper_value(GRIPPER_CLOSE, GRIPPER_SPEED)
    time.sleep(3)

    # Lift via base-coord Z (no IK needed for transport — same as hardcoded).
    print(f"[ik_pick] -> lift z={LIFT_Z_MM}mm", flush=True)
    arm.send_base_coord(3, LIFT_Z_MM, SPEED)
    time.sleep(3)

    print("[ik_pick] OK single-arm pick complete (IK path)", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())