#!/usr/bin/env python3
"""Fixed-table degraded release after navigation has turned to table two."""

from __future__ import annotations

import argparse
import json
import os
import time

DEFAULT_PLACE = (400.0, 10.0, 75.0, 179.99, -12.0, 0.0)
TRANSPORT_ANGLES = (20.0, -10.0, -130.0, -80.0, 73.0, 90.0)
PORT = os.environ.get("DEGRADED_ARM_PORT", "/dev/right_arm")
GRIPPER_OPEN = 70


def _target() -> tuple[float, ...]:
    names = ("X", "Y", "Z", "RX", "RY", "RZ")
    return tuple(float(os.environ.get(f"DEGRADED_PLACE_{name}", value))
                 for name, value in zip(names, DEFAULT_PLACE))


def _wait_pose(robot, expected, timeout=8.0, tol=5.0):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = robot.get_base_coords()
        if last and max(abs(float(last[i]) - expected[i]) for i in range(3)) <= tol:
            return list(last)
        time.sleep(0.25)
    raise RuntimeError(f"pose_not_reached expected={expected} actual={last}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    place = _target()
    preplace = (place[0], place[1], 110.0, place[3], place[4], place[5])
    print(json.dumps({"mode": "degraded_fixed_table", "place": place,
                      "preplace": preplace, "transport": TRANSPORT_ANGLES}))
    if args.dry_run:
        return 0

    from pymycobot import Mercury
    arm = Mercury(PORT, 115200)
    if not arm.is_power_on():
        arm.power_on(); time.sleep(1.5)
    status = arm.get_robot_status()
    if not isinstance(status, list) or any(int(v) != 0 for v in status):
        raise RuntimeError(f"robot_status_not_ok:{status}")
    arm.send_base_coords(list(preplace), 8); _wait_pose(arm, preplace)
    arm.send_base_coords(list(place), 6); reached = _wait_pose(arm, place)
    arm.set_gripper_value(GRIPPER_OPEN, 20); time.sleep(2.0)
    arm.send_base_coords(list(preplace), 8); _wait_pose(arm, preplace)
    arm.send_angles(list(TRANSPORT_ANGLES), 8, _async=False)
    print(json.dumps({"success": True, "release_pose": reached,
                      "gripper_command": GRIPPER_OPEN}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
