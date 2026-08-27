#!/usr/bin/env python3
"""Publish initial pose to AMCL so it seeds particles near goal1_start.

Why this exists:
  - We deleted the hardcoded static_transform_publisher (was forcing
    map -> base_link at a fixed point, which lied about reality).
  - /global_localization spreads particles over the WHOLE map; if the
    robot starts at a known position (goal1_start) this is wasteful
    and takes 30-60 seconds to converge.
  - This script publishes /initialpose = goal1_start pose. AMCL seeds
    its particle cloud NEAR that pose, and converges in 5-10 seconds
    using the lidar scan as confirmation.

Crucially: this is a HINT, not a hardcoded TF.
  - AMCL still runs the particle filter on top of the hint.
  - If the robot is actually somewhere else, AMCL can recover by
    spreading particles during the next update_min_d / update_min_a
    cycle.
  - RViz will show the robot at goal1 only AFTER AMCL has converged,
    which means lidar data matched the map there. That is the
    correct, lidar-grounded localization you wanted.

Use site YAML's goal1_start pose as the initial hint. If the YAML
has multiple targets, prefer goal1_start explicitly; fall back to
the first entry if goal1_start is missing.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Any

import rospy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped


DEFAULT_GOAL = "goal1_start"
INITIAL_POSE_TOPIC = "/initialpose"


def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    """Convert 2D yaw to a quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _load_goal_pose(site_path: Path, target_id: str) -> tuple[float, float, float]:
    """Read the byte-exact pose for target_id from the site YAML.

    Returns (x, y, yaw_rad). Raises SystemExit on failure so the
    caller sees a clear error in the race log.
    """
    with site_path.open("r", encoding="utf-8") as fh:
        profile = yaml.safe_load(fh)
    if not isinstance(profile, dict) or profile.get("schema") != "pick_navigation_site/v1":
        raise SystemExit(f"site_profile_schema_invalid: {site_path}")
    targets = profile.get("allowed_targets") or []
    if not isinstance(targets, list) or not targets:
        raise SystemExit("site_profile_allowlist_empty")

    chosen: dict[str, Any] | None = None
    for entry in targets:
        if isinstance(entry, dict) and entry.get("target_id") == target_id:
            chosen = entry
            break
    if chosen is None:
        # Fall back to first target with a pose
        for entry in targets:
            if isinstance(entry, dict) and isinstance(entry.get("pose"), dict):
                chosen = entry
                rospy.logwarn(
                    "goal '%s' not in site YAML, falling back to first target '%s'",
                    target_id, entry.get("target_id"))
                break
    if chosen is None:
        raise SystemExit(f"no target with pose found in {site_path}")

    pose = chosen["pose"]
    if pose.get("frame_id") != "map":
        raise SystemExit(
            f"target '{chosen.get('target_id')}' pose frame_id must be 'map',"
            f" got {pose.get('frame_id')!r}")
    x = float(pose["x"])
    y = float(pose["y"])
    yaw = float(pose.get("yaw_rad", 0.0))
    return x, y, yaw


def publish_initial_pose(x: float, y: float, yaw: float) -> None:
    """Publish /initialpose once, with a generous covariance so AMCL
    treats it as a hint (not a hard lock).
    """
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0.0
    qx, qy, qz, qw = _yaw_to_quat(yaw)
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    # Covariance: 0.5m std on x/y, ~30 deg on yaw — wide enough that
    # AMCL can still reject the hint if lidar disagrees.
    cov = [0.0] * 36
    cov[0] = 0.5
    cov[7] = 0.5
    cov[35] = 0.5
    msg.pose.covariance = cov

    pub = rospy.Publisher(INITIAL_POSE_TOPIC, PoseWithCovarianceStamped,
                          queue_size=1, latch=True)
    rospy.sleep(1.0)  # let subscriber (AMCL) connect
    pub.publish(msg)
    rospy.loginfo(
        "published /initialpose at (%.4f, %.4f, yaw=%.4f rad) — "
        "AMCL will refine via lidar", x, y, yaw)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--site-yaml",
        default="/home/elephant/scripts/pick_navigation.site.yaml",
        help="Path to the pick_navigation_site/v1 allowlist YAML.")
    parser.add_argument("--target-id", default=DEFAULT_GOAL,
                        help="target_id to use as the initial pose hint.")
    parser.add_argument("--x", type=float, default=None,
                        help="Override X (meters). Skip site YAML.")
    parser.add_argument("--y", type=float, default=None,
                        help="Override Y (meters). Skip site YAML.")
    parser.add_argument("--yaw", type=float, default=None,
                        help="Override yaw (radians). Skip site YAML.")
    args = parser.parse_args()

    rospy.init_node("set_initial_pose", anonymous=True)

    if args.x is not None and args.y is not None:
        yaw = args.yaw if args.yaw is not None else 0.0
        x, y = args.x, args.y
        rospy.loginfo("using CLI overrides: (%.4f, %.4f, yaw=%.4f)", x, y, yaw)
    else:
        site_path = Path(args.site_yaml).expanduser()
        if not site_path.exists():
            raise SystemExit(f"site_yaml not found: {site_path}")
        x, y, yaw = _load_goal_pose(site_path, args.target_id)
        rospy.loginfo("using site YAML target '%s': (%.4f, %.4f, yaw=%.4f)",
                      args.target_id, x, y, yaw)

    publish_initial_pose(x, y, yaw)
    return 0


if __name__ == "__main__":
    sys.exit(main())