#!/usr/bin/env python3
"""Publish a ``pick_navigation`` mission onto ``/x1/pick/nav_mission`` and
print the adapter's evidence reply.

Why a Python publisher instead of ``rostopic pub``:
* the mission payload is JSON; shell quoting of nested ``{}`` and ``"``
  breaks every other dry-run (yesterday T5 died on JSON escape).
* the pose must be a *byte-exact* match against the site YAML allowlist
  (the adapter rejects anything else with ``mission_pose_not_exact_site_allowlist_match``).
  Reading the YAML here and reusing the same Python dict guarantees that.

Fail-closed contract (mirrors ``pick_navigation_adapter_ros1.py``):
* the script never retries.  If the adapter rejects, we print the reason
  and exit non-zero so the operator can decide.
* ``mission_id`` defaults to ``race-<UTC seconds>`` so it is always unique;
  override with ``--mission-id`` for reproducibility.
* ``nav_epoch`` defaults to 1; bump it when re-issuing the same target.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

import rospy
import yaml
from std_msgs.msg import String

MISSION_TOPIC = "/x1/pick/nav_mission"
EVIDENCE_TOPIC = "/x1/pick/navigation_evidence"


def _load_allowlist(site_path: Path) -> list[dict[str, Any]]:
    with site_path.open("r", encoding="utf-8") as fh:
        profile = yaml.safe_load(fh)
    if not isinstance(profile, dict) or profile.get("schema") != "pick_navigation_site/v1":
        raise SystemExit(f"site_profile_schema_invalid: {site_path}")
    targets = profile.get("allowed_targets") or []
    if not isinstance(targets, list) or not targets:
        raise SystemExit("site_profile_allowlist_empty (no targets in YAML)")
    return targets


def _select_target(targets: list[dict[str, Any]], requested: str | None) -> dict[str, Any]:
    if requested:
        for t in targets:
            if t.get("target_id") == requested:
                return t
        available = ", ".join(t.get("target_id", "?") for t in targets)
        raise SystemExit(f"mission_target_not_allowlisted: '{requested}' not in [{available}]")
    if len(targets) == 1:
        return targets[0]
    available = ", ".join(t.get("target_id", "?") for t in targets)
    raise SystemExit(f"multiple targets available; pass --target-id one of [{available}]")


def _build_mission(target: dict[str, Any], mission_id: str, nav_epoch: int) -> dict[str, Any]:
    pose = dict(target["pose"])  # shallow copy keeps dict identity stable for adapter equality
    if pose.get("frame_id") != "map":
        raise SystemExit(f"target pose frame_id must be 'map', got {pose.get('frame_id')!r}")
    return {
        "mission_id": mission_id,
        "nav_epoch": nav_epoch,
        "target_id": target["target_id"],
        "pose": pose,
    }


def _wait_for_evidence(timeout_sec: float) -> dict[str, Any] | None:
    """Block until one evidence reply arrives or the timeout fires.

    Subscribes BEFORE we publish so the subscriber connection is wired
    before the adapter receives the mission; this avoids the
    "publisher dropped because subscriber not yet connected" race that
    otherwise surfaces as a spurious ``no_evidence`` reply.
    """
    box: dict[str, Any] = {}

    def _cb(msg: String) -> None:
        try:
            box["payload"] = json.loads(msg.data)
        except ValueError:
            box["payload"] = {"_invalid_json": True, "_raw": msg.data}

    sub = rospy.Subscriber(EVIDENCE_TOPIC, String, _cb, queue_size=10)
    # Give the master a beat to propagate the new subscriber registration.
    rospy.sleep(0.5)
    deadline = time.monotonic() + timeout_sec
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if "payload" in box:
            return box["payload"]
        rate.sleep()
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--site-yaml",
        default="/home/elephant/temp/deyes/pick_navigation.site.yaml",
        help="Path to the pick_navigation_site/v1 allowlist YAML.",
    )
    parser.add_argument("--target-id", default=None, help="Force a specific allowlist target.")
    parser.add_argument("--mission-id", default=None, help="Override mission_id (default: race-<UTC>).")
    parser.add_argument("--nav-epoch", type=int, default=1, help="Bump when re-issuing the same target.")
    parser.add_argument("--timeout", type=float, default=10.0, help="Seconds to wait for evidence.")
    args = parser.parse_args()

    site_path = Path(args.site_yaml).expanduser()
    if not site_path.exists():
        raise SystemExit(f"site_profile_load_failed: {site_path} not found")

    targets = _load_allowlist(site_path)
    target = _select_target(targets, args.target_id)
    mission_id = args.mission_id or f"race-{int(time.time())}"
    mission = _build_mission(target, mission_id, args.nav_epoch)

    rospy.init_node("send_mission", anonymous=True)
    pub = rospy.Publisher(MISSION_TOPIC, String, queue_size=10, latch=False)
    # Block briefly so the publisher is registered before the first message.
    rospy.sleep(0.5)

    payload = json.dumps(mission, separators=(",", ":"))
    rospy.loginfo("publishing mission: %s", payload)
    pub.publish(String(data=payload))
    # Re-publish once after a short delay in case the first message raced
    # with the adapter subscriber's TCPROS handshake.
    rospy.sleep(0.5)
    pub.publish(String(data=payload))

    evidence = _wait_for_evidence(args.timeout)
    if evidence is None:
        print(json.dumps({
            "status": "no_evidence",
            "mission_id": mission_id,
            "hint": "adapter did not reply within timeout; is T2 (adapter) running?",
        }, indent=2))
        return 2

    print(json.dumps({"status": "evidence", "mission": mission, "evidence": evidence}, indent=2))
    return 0 if evidence.get("result") == "succeeded" else 1


if __name__ == "__main__":
    sys.exit(main())
