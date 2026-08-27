#!/usr/bin/env python3
"""Hardcoded single-arm pick sequence for Mercury X1 (this venue robot).

On-site probe (2026-08-27 venue robot 192.168.43.60):
  - ttyACM0: opens but is NOT a Mercury arm (no protocol response)
  - ttyACM1: POWER_ON OK, 6 reported joints — the working arm
  - ttyACM2: chassis (wheeltec_controller symlink)
  - ttyACM3: permission denied (probably camera or radar)

So we use ONLY ttyACM1. Single-arm pick (no dual-arm hug).

Drop into: nav has already brought the chassis to goal_3, robot facing desk 1.
Sequence: open gripper -> descend to pre-grasp -> descend to grasp ->
close gripper -> lift.

Joint angles below were probed live on this robot:
  read on first power_on: [4.479, 94.999, 5.009, -84.29, 76.824, 92.6]
These are a safe observation pose. Tune interactively if needed.

Usage:
  python3 ~/scripts/pick_pen_hardcoded.py
"""
from __future__ import annotations

import sys
import time

from pymycobot import Mercury

# Serial port — verified on 2026-08-27 venue robot
ARM_PORT = "/dev/ttyACM1"
BAUDRATE = 115200

# Joint angles (degrees): safe observation pose — probed live from arm
OBSERVE_ANGLES = [4.479, 94.999, 5.009, -84.29, 76.824, 92.6]

# Vertical descent values (mm): Mercury uses send_base_coord(3, z, speed)
PREGRASP_Z_MM = 200   # 20 cm above pen
GRASP_Z_MM = 100      # at pen level (desk 660 - half pen 70 - clearance 10 = 580)
LIFT_Z_MM = 350       # 35 cm above pen after grasp (transport height)

SPEED = 30
GRIPPER_OPEN = 0
GRIPPER_CLOSE = 70    # 0=open, 100=closed; 70 leaves room for variable pen sizes
GRIPPER_SPEED = 50


def connect(port: str) -> Mercury:
    print(f"[pick] connecting to {port} @ {BAUDRATE}", flush=True)
    mc = Mercury(port, BAUDRATE)
    time.sleep(0.5)
    # Mercury.is_power_on() returns int (0/1) NOT Python bool.
    # Use truthy check `not state` instead of `is not True`.
    state = mc.is_power_on()
    if not state:
        print(f"[pick] {port}: not on (state={state}), calling power_on()",
              flush=True)
        mc.power_on()
        time.sleep(1.5)
        state = mc.is_power_on()
    if not state:
        raise RuntimeError(f"{port}: power_on failed (state={state})")
    print(f"[pick] {port}: arm ready (state={state})", flush=True)
    return mc


def main() -> int:
    print("[pick] starting single-arm pick sequence", flush=True)
    arm = connect(ARM_PORT)

    # Set adaptive gripper mode
    arm.set_gripper_mode(0)
    time.sleep(0.5)

    # Move arm to observation pose
    print(f"[pick] -> observation pose: {OBSERVE_ANGLES}", flush=True)
    arm.send_angles(OBSERVE_ANGLES, SPEED)
    time.sleep(4)  # give arm time to settle

    # Open gripper
    print("[pick] -> gripper OPEN", flush=True)
    arm.set_gripper_value(GRIPPER_OPEN, GRIPPER_SPEED)
    time.sleep(2)

    # Descend to pre-grasp
    print(f"[pick] -> pre-grasp z={PREGRASP_Z_MM}mm", flush=True)
    arm.send_base_coord(3, PREGRASP_Z_MM, SPEED)
    time.sleep(3)

    # Descend to grasp height
    print(f"[pick] -> grasp z={GRASP_Z_MM}mm", flush=True)
    arm.send_base_coord(3, GRASP_Z_MM, SPEED)
    time.sleep(3)

    # Close gripper (single-arm pick)
    print(f"[pick] -> gripper CLOSE to {GRIPPER_CLOSE}", flush=True)
    arm.set_gripper_value(GRIPPER_CLOSE, GRIPPER_SPEED)
    time.sleep(3)

    # Lift
    print(f"[pick] -> lift z={LIFT_Z_MM}mm", flush=True)
    arm.send_base_coord(3, LIFT_Z_MM, SPEED)
    time.sleep(3)

    print("[pick] OK single-arm pick complete", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())