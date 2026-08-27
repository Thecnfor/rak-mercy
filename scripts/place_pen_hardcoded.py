#!/usr/bin/env python3
"""Hardcoded single-arm place sequence for Mercury X1 (this venue robot).

Same ttyACM discovery as pick_pen_hardcoded.py — only ttyACM1 is a
working Mercury arm on this robot.

Drop into: nav has already brought the chassis to goal_4, robot facing desk 2.
Sequence: transport height -> descend to place height -> open gripper
(release pen) -> lift.

Usage:
  python3 ~/scripts/place_pen_hardcoded.py
"""
from __future__ import annotations

import sys
import time

from pymycobot import Mercury

ARM_PORT = "/dev/ttyACM1"
BAUDRATE = 115200

# Same observation pose as pick — probed live on this robot
OBSERVE_ANGLES = [4.479, 94.999, 5.009, -84.29, 76.824, 92.6]

# Vertical positions (mm)
TRANSPORT_Z_MM = 350   # carrying height above desk
PLACE_Z_MM = 150       # release point (just above desk surface)
LIFT_Z_MM = 350        # retract after release

SPEED = 30
GRIPPER_OPEN = 0
GRIPPER_SPEED = 50


def connect(port: str) -> Mercury:
    print(f"[place] connecting to {port} @ {BAUDRATE}", flush=True)
    mc = Mercury(port, BAUDRATE)
    time.sleep(0.5)
    state = mc.is_power_on()
    if not state:
        print(f"[place] {port}: not on (state={state}), calling power_on()",
              flush=True)
        mc.power_on()
        time.sleep(1.5)
        state = mc.is_power_on()
    if not state:
        raise RuntimeError(f"{port}: power_on failed (state={state})")
    print(f"[place] {port}: arm ready (state={state})", flush=True)
    return mc


def main() -> int:
    print("[place] starting single-arm place sequence", flush=True)
    arm = connect(ARM_PORT)

    arm.set_gripper_mode(0)
    time.sleep(0.5)

    # Move to transport height (carrying pen from desk 1)
    print(f"[place] -> transport pose: {OBSERVE_ANGLES}", flush=True)
    arm.send_angles(OBSERVE_ANGLES, SPEED)
    time.sleep(4)

    # Lower to place height (just above desk 2 surface)
    print(f"[place] -> place z={PLACE_Z_MM}mm", flush=True)
    arm.send_base_coord(3, PLACE_Z_MM, SPEED)
    time.sleep(3)

    # Open gripper to release pen
    print("[place] -> gripper OPEN (release pen)", flush=True)
    arm.set_gripper_value(GRIPPER_OPEN, GRIPPER_SPEED)
    time.sleep(2)

    # Lift arm back to transport height
    print(f"[place] -> lift z={LIFT_Z_MM}mm", flush=True)
    arm.send_base_coord(3, LIFT_Z_MM, SPEED)
    time.sleep(3)

    print("[place] OK place complete", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())