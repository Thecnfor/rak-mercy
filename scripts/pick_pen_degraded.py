#!/usr/bin/env python3
"""Fixed-table degraded pick using venue-probed Mercury BaseCoords."""

from __future__ import annotations

import argparse
import json
import os
import time

# Venue-tuned on 2026-08-27 after the pen was moved into the right-arm zone.
DEFAULT_PICK = (400.0, 10.0, 45.0, 179.99, -12.0, 0.0)
TRANSPORT_ANGLES = (20.0, -10.0, -130.0, -80.0, 73.0, 90.0)
PORT = os.environ.get("DEGRADED_ARM_PORT", "/dev/right_arm")
BAUD = 115200
MOVE_SPEED = 8
GRIPPER_SPEED = 20
GRIPPER_OPEN = 70
GRIPPER_CLOSED = 0


def _target() -> tuple[float, ...]:
    names = ("X", "Y", "Z", "RX", "RY", "RZ")
    return tuple(float(os.environ.get(f"DEGRADED_PICK_{name}", value))
                 for name, value in zip(names, DEFAULT_PICK))


def _wait_pose(robot, expected, timeout=8.0, xyz_tol=4.0, angle_tol=2.0):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = robot.get_base_coords()
        if last and len(last) >= 6:
            xyz_ok = max(abs(float(last[i]) - expected[i]) for i in range(3)) <= xyz_tol
            rpy_ok = max(abs(float(last[i]) - expected[i]) for i in range(3, 6)) <= angle_tol
            if xyz_ok and rpy_ok:
                return list(last)
        time.sleep(0.25)
    raise RuntimeError(f"pose_not_reached expected={expected} actual={last}")


def _status_ok(robot):
    status = robot.get_robot_status()
    if not isinstance(status, list) or any(int(v) != 0 for v in status):
        raise RuntimeError(f"robot_status_not_ok:{status}")
    return status


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    pick = _target()
    pregrasp = (pick[0], pick[1], 90.0, pick[3], pick[4], pick[5])
    z50 = (pick[0], pick[1], 50.0, pick[3], pick[4], pick[5])
    print(json.dumps({"mode": "degraded_fixed_table", "pick": pick,
                      "pregrasp": pregrasp, "transport": TRANSPORT_ANGLES}))
    if args.dry_run:
        return 0

    from pymycobot import Mercury
    arm = Mercury(PORT, BAUD)
    if not arm.is_power_on():
        arm.power_on(); time.sleep(1.5)
    _status_ok(arm)
    arm.set_gripper_mode(0)
    arm.set_gripper_value(GRIPPER_OPEN, GRIPPER_SPEED); time.sleep(1.0)

    arm.send_base_coords(list(pregrasp), MOVE_SPEED)
    _wait_pose(arm, pregrasp)
    arm.send_base_coords(list(z50), 6)
    _wait_pose(arm, z50)
    jog = arm.jog_base_increment_coord(3, pick[2] - 50.0, 4, _async=False)
    if jog != 0:
        raise RuntimeError(f"grasp_z_jog_failed:{jog}")
    reached = _wait_pose(arm, pick, timeout=4.0, xyz_tol=3.0)
    _status_ok(arm)

    arm.set_gripper_value(GRIPPER_CLOSED, GRIPPER_SPEED); time.sleep(2.0)
    arm.send_base_coords(list(pregrasp), 8)
    _wait_pose(arm, pregrasp)
    arm.send_angles(list(TRANSPORT_ANGLES), 8, _async=False)
    _status_ok(arm)
    print(json.dumps({"success": True, "grasp_pose": reached,
                      "gripper_command": GRIPPER_CLOSED}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
