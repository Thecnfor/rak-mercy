#!/usr/bin/env python3
"""Send a single move_base goal without the adapter.

Why this exists:
  - send_mission.py goes through pick_navigation_adapter_ros1.py, which has
    observed to falsely report "amcl_or_odom_stale_after_success" even
    when nav actually succeeded. The adapter also has a completion latch
    that blocks subsequent missions.
  - For the race-day flow we want each goal to be sent DIRECTLY to
    move_base via SimpleActionClient, with a single retry, blocking
    until SUCCEEDED.

Usage:
  python3 ~/scripts/send_one_goal.py goal1_start
  python3 ~/scripts/send_one_goal.py goal3_right
  python3 ~/scripts/send_one_goal.py goal4_back

Exit codes:
  0 = SUCCEEDED within timeout
  1 = any other terminal state (ABORTED, REJECTED, LOST)
  2 = timeout
"""

from __future__ import annotations

import math
import sys
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


GOALS: dict[str, tuple[float, float, float, float, int]] = {
    # name -> (x, y, z_quat, w_quat, timeout_sec)
    "goal1_start":  (0.030306, -0.033085, -0.00600194, 0.999982, 30),
    "goal3_right":  (2.732700, -1.939300,  0.00652463, 0.999972, 90),
    "goal4_back":   (2.640200, -1.969990, -0.70577400, 0.708434, 90),
}


def send_one(target: str) -> int:
    if target not in GOALS:
        print(f"unknown target {target!r}; valid: {list(GOALS)}", flush=True)
        return 1

    x, y, z, w, timeout = GOALS[target]
    rospy.init_node(f"send_one_goal_{target}", anonymous=True)

    print(f"[goal] waiting for /move_base ...", flush=True)
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    if not client.wait_for_server(rospy.Duration(20.0)):
        print(f"[goal] /move_base not available after 20s", flush=True)
        return 2

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w

    print(f"[goal] sending {target} ({x:.3f}, {y:.3f})", flush=True)
    client.send_goal(goal)

    if not client.wait_for_result(rospy.Duration(timeout)):
        print(f"[goal] TIMEOUT after {timeout}s", flush=True)
        client.cancel_goal()
        return 2

    state = client.get_state()
    state_name = {
        GoalStatus.PENDING: "PENDING",
        GoalStatus.ACTIVE: "ACTIVE",
        GoalStatus.PREEMPTED: "PREEMPTED",
        GoalStatus.SUCCEEDED: "SUCCEEDED",
        GoalStatus.ABORTED: "ABORTED",
        GoalStatus.REJECTED: "REJECTED",
        GoalStatus.PREEMPTING: "PREEMPTING",
        GoalStatus.RECALLING: "RECALLING",
        GoalStatus.RECALLED: "RECALLED",
        GoalStatus.LOST: "LOST",
    }.get(state, f"state={state}")

    if state == GoalStatus.SUCCEEDED:
        print(f"[goal] {target} SUCCEEDED", flush=True)
        return 0

    print(f"[goal] {target} FAILED ({state_name})", flush=True)
    return 1


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: send_one_goal.py <target_id>", flush=True)
        return 1
    target = sys.argv[1]
    try:
        return send_one(target)
    except rospy.ROSInterruptException:
        return 1


if __name__ == "__main__":
    sys.exit(main())