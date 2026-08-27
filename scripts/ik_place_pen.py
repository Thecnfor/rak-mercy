#!/usr/bin/env python3
"""IK-based single-arm place sequence for Mercury X1 (right arm, ttyACM1).

Drop-in alternative to ``place_pen_hardcoded.py`` — same vertical
desport, IK only kicks in if ``IK_PLACE_X_MM`` (and friends) are set.
"""

from __future__ import annotations

import os
import sys
import time

from pymycobot import Mercury

ARM_PORT = "/dev/ttyACM1"
BAUDRATE = 115200

OBSERVE_ANGLES = [4.479, 94.999, 4.5, -84.29, 76.824, 92.6, 0.0]

TRANSPORT_Z_MM = 350
LIFT_Z_MM = 350

SPEED = 30
GRIPPER_OPEN = 0
GRIPPER_SPEED = 50


def _read_float(env_name: str, default: float) -> float:
    raw = os.environ.get(env_name)
    return float(raw) if raw not in (None, "") else default


def _connect() -> Mercury:
    print(f"[ik_place] connecting to {ARM_PORT} @ {BAUDRATE}", flush=True)
    mc = Mercury(ARM_PORT, BAUDRATE)
    time.sleep(0.5)
    state = mc.is_power_on()
    if not state:
        mc.power_on()
        time.sleep(1.5)
        state = mc.is_power_on()
    if not state:
        raise RuntimeError(f"{ARM_PORT}: power_on failed (state={state})")
    print(f"[ik_place] arm ready (state={state})", flush=True)
    return mc


def main() -> int:
    print("[ik_place] starting single-arm place sequence", flush=True)
    arm = _connect()

    arm.set_gripper_mode(0)
    time.sleep(0.5)

    # Carry at transport height.
    print(f"[ik_place] -> transport pose: {OBSERVE_ANGLES}", flush=True)
    arm.send_angles(OBSERVE_ANGLES, SPEED)
    time.sleep(4)

    # If vision teammate gives us a place point, IK there; otherwise
    # use the hardcoded base-coord Z descent (matches place_pen_hardcoded.py).
    if os.environ.get("IK_PLACE_X_MM"):
        target_mm = (
            _read_float("IK_PLACE_X_MM", 200.0),
            _read_float("IK_PLACE_Y_MM", 0.0),
            _read_float("IK_PLACE_Z_MM", 580.0),
        )
        target_euler_deg = (
            _read_float("IK_PLACE_RX_DEG", 90.0),
            _read_float("IK_PLACE_RY_DEG", 0.0),
            _read_float("IK_PLACE_RZ_DEG", 0.0),
        )
        target_xyz_m = tuple(v / 1000.0 for v in target_mm)
        try:
            from deyes_ik_server.ikpy_solver import IkpySolver7DOF
            solver = IkpySolver7DOF("right")
            res = solver.solve(
                list(target_xyz_m) + list(target_euler_deg),
                current_joint_deg=list(OBSERVE_ANGLES),
            )
            if res.success:
                print(f"[ik_place] -> IK place angles: {[round(v, 2) for v in res.joint_deg]}", flush=True)
                arm.send_angles(res.joint_deg, SPEED)
                time.sleep(4)
            else:
                raise RuntimeError(f"ikpy failed: {res.failure_code}")
        except Exception as exc:
            print(f"[ik_place] IK place unavailable ({exc!r}); falling to Z-only descent", flush=True)
            PLACE_Z_MM = int(target_mm[2] - 580 + 150)  # 150mm above desk2 surface
            arm.send_base_coord(3, PLACE_Z_MM, SPEED)
            time.sleep(3)
    else:
        PLACE_Z_MM = 150
        print(f"[ik_place] -> place z={PLACE_Z_MM}mm (no IK_PLACE_* env)", flush=True)
        arm.send_base_coord(3, PLACE_Z_MM, SPEED)
        time.sleep(3)

    print("[ik_place] -> gripper OPEN (release pen)", flush=True)
    arm.set_gripper_value(GRIPPER_OPEN, GRIPPER_SPEED)
    time.sleep(2)

    print(f"[ik_place] -> lift z={LIFT_Z_MM}mm", flush=True)
    arm.send_base_coord(3, LIFT_Z_MM, SPEED)
    time.sleep(3)

    print("[ik_place] OK place complete (IK path)", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())