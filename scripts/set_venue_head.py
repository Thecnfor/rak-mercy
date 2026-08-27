#!/usr/bin/env python3
"""Restore the camera head pose used by the 2026-08-27 venue calibration."""

import argparse
import json
import os
import time


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--joint-11", type=float, default=-54.93)
    parser.add_argument("--joint-12", type=float, default=2.63)
    args = parser.parse_args()
    target = {"joint_11_deg": args.joint_11, "joint_12_deg": args.joint_12}
    print(json.dumps(target))
    if args.dry_run:
        return 0
    from pymycobot import Mercury

    robot = Mercury(os.environ.get("DEGRADED_ARM_PORT", "/dev/right_arm"), 115200)
    if not robot.is_power_on():
        robot.power_on()
        time.sleep(1.5)
    robot.send_angle(11, args.joint_11, 8)
    robot.send_angle(12, args.joint_12, 8)
    time.sleep(2.0)
    if hasattr(robot, "get_angle"):
        actual = [robot.get_angle(11), robot.get_angle(12)]
    else:
        angles = robot.get_angles()
        actual = [angles[10], angles[11]] if angles and len(angles) >= 12 else [None, None]
    if any(value is None for value in actual):
        raise RuntimeError(f"head_feedback_missing:{actual}")
    if abs(float(actual[0]) - args.joint_11) > 0.5 or abs(float(actual[1]) - args.joint_12) > 0.5:
        raise RuntimeError(f"head_pose_mismatch target={target} actual={actual}")
    print(json.dumps({"success": True, "actual_deg": actual}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
