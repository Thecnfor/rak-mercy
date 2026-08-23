#!/usr/bin/env python3
"""Race-day health monitor for the T1..T6 dry-run windows.

Refreshes a compact status panel every second so the operator can see at a
glance whether the full pipeline (ROS1 nav -> adapter -> ros1_bridge ->
Deyes -> mission publisher) is alive.  Mirrors the 6-terminal manual that
shipped with ``start_competition_pipeline.sh``.

Design notes
------------
* Pure ROS 1 (the bridge surfaces enough ROS 2 traffic for T4 monitoring;
  we additionally shell out to ``ros2 node list`` once per refresh as a
  sanity check on the Deyes side).
* No automatic remediation: this script *only* prints.  Failures must be
  diagnosed by the operator (race day: react-on-sight, never auto-retry).
* Health thresholds are intentionally conservative; tune ``STALE_*`` and
  ``EXPECTED_TOPIC_COUNT`` below if the venue's hardware differs.
"""

from __future__ import annotations

import json
import subprocess
import time
from collections import deque
from typing import Any

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

# ---------------------------------------------------------------------------
# Tunable thresholds (seconds without a message before "STALE" is declared)
# ---------------------------------------------------------------------------
STALE_SCAN_SEC = 2.0         # /scan is normally 10 Hz
STALE_AMCL_SEC = 2.0         # /amcl_pose roughly matches move_base ticks
STALE_EVIDENCE_SEC = 30.0    # one evidence per mission, so allow long gaps
STALE_MISSION_SEC = 60.0     # missions are infrequent during dry-run
EXPECTED_BRIDGE_TOPICS = 1   # how many bidirectional bridge pairs we expect
EXPECTED_DEYES_NODES_MIN = 8 # deyes_bringup launches 11 nodes (some optional)

EVIDENCE_TOPIC = "/x1/pick/navigation_evidence"
MISSION_TOPIC = "/x1/pick/nav_mission"


class _FreshnessTracker:
    """Track last-seen timestamp and emit "STALE" when it goes quiet."""

    def __init__(self, name: str, stale_sec: float) -> None:
        self.name = name
        self.stale_sec = stale_sec
        self.last_msg: Any = None
        self.last_seen: float | None = None
        self.count = 0

    def update(self, msg: Any) -> None:
        self.last_msg = msg
        self.last_seen = time.monotonic()
        self.count += 1

    def status(self) -> tuple[str, str]:
        if self.last_seen is None:
            return ("WAITING", "no message yet")
        age = time.monotonic() - self.last_seen
        if age > self.stale_sec:
            return ("STALE", f"{age:.1f}s since last")
        return ("OK", f"n={self.count} {age:.1f}s ago")


def _ros2_node_count() -> tuple[int | None, str]:
    """Best-effort count of ROS 2 nodes; may fail if RMW is misconfigured."""
    try:
        out = subprocess.check_output(
            ["bash", "-lc",
             "source /opt/ros/galactic/setup.bash 2>/dev/null && "
             "source $HOME/deyes_physical_ws_8375517/install/setup.bash 2>/dev/null && "
             "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
             "export ROS_DOMAIN_ID=0 && "
             "ros2 daemon stop >/dev/null 2>&1; "
             "ros2 node list 2>/dev/null"],
            stderr=subprocess.STDOUT, timeout=8.0,
        ).decode().strip()
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired, FileNotFoundError) as exc:
        return None, f"ros2 unavailable: {type(exc).__name__}"
    nodes = [n for n in out.splitlines() if n.strip()]
    return len(nodes), f"{len(nodes)} nodes"


def _bridge_topic_count() -> tuple[int | None, str]:
    """Rough proxy: how many /x1/* topics the bridge is forwarding."""
    try:
        out = subprocess.check_output(
            ["bash", "-lc",
             "source /opt/ros/noetic/setup.bash 2>/dev/null && "
             "source /opt/ros/galactic/setup.bash 2>/dev/null && "
             "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
             "export ROS_DOMAIN_ID=0 && "
             "rostopic list 2>/dev/null | grep -c '^/x1/'"],
            stderr=subprocess.STDOUT, timeout=8.0,
        ).decode().strip()
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired, FileNotFoundError) as exc:
        return None, f"rostopic unavailable: {type(exc).__name__}"
    try:
        return int(out.splitlines()[-1] if out else "0"), f"{out.split()} /x1/* topics"
    except ValueError:
        return None, f"unexpected output: {out!r}"


def main() -> None:
    rospy.init_node("race_monitor", anonymous=True)
    scan = _FreshnessTracker("/scan", STALE_SCAN_SEC)
    amcl = _FreshnessTracker("/amcl_pose", STALE_AMCL_SEC)
    evidence = _FreshnessTracker(EVIDENCE_TOPIC, STALE_EVIDENCE_SEC)
    mission = _FreshnessTracker(MISSION_TOPIC, STALE_MISSION_SEC)

    last_evidence_payload: dict[str, Any] | None = None
    last_mission_payload: dict[str, Any] | None = None
    history: deque[dict[str, Any]] = deque(maxlen=8)

    rospy.Subscriber("/scan", LaserScan, scan.update, queue_size=1)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl.update, queue_size=1)

    def _sanitize(value: Any) -> Any:
        # Defense-in-depth against terminal-injection: ROS String values
        # originate from JSON on the wire and may carry raw control bytes
        # (e.g. 0x1b ESC) that the terminal would otherwise interpret.
        if isinstance(value, str):
            return "".join(c if c.isprintable() or c in "\n\t" else "?" for c in value)
        return value

    def _on_evidence(msg: String) -> None:
        nonlocal last_evidence_payload
        evidence.update(msg)
        try:
            last_evidence_payload = json.loads(msg.data)
            history.append({
                "ts": time.time(), "kind": "evidence",
                "result": _sanitize(last_evidence_payload.get("result")),
                "reason": _sanitize(last_evidence_payload.get("reason")),
                "mission_id": _sanitize(last_evidence_payload.get("mission_id")),
            })
        except ValueError:
            last_evidence_payload = {"_invalid_json": True, "_raw": msg.data[:120]}

    def _on_mission(msg: String) -> None:
        nonlocal last_mission_payload
        mission.update(msg)
        try:
            last_mission_payload = json.loads(msg.data)
            history.append({
                "ts": time.time(), "kind": "mission",
                "target_id": _sanitize(last_mission_payload.get("target_id")),
                "mission_id": _sanitize(last_mission_payload.get("mission_id")),
            })
        except ValueError:
            last_mission_payload = {"_invalid_json": True, "_raw": msg.data[:120]}

    rospy.Subscriber(EVIDENCE_TOPIC, String, _on_evidence, queue_size=1)
    rospy.Subscriber(MISSION_TOPIC, String, _on_mission, queue_size=1)

    started = time.monotonic()
    print("race_monitor started (Ctrl-C to quit)", flush=True)
    while not rospy.is_shutdown():
        ros2_count, ros2_detail = _ros2_node_count()
        bridge_count, bridge_detail = _bridge_topic_count()

        t1_main = amcl.status()
        t1_sensor = scan.status()
        t2 = evidence.status()
        t3_status = "OK" if bridge_count and bridge_count >= EXPECTED_BRIDGE_TOPICS else "WARN"
        t4_status = "OK" if ros2_count and ros2_count >= EXPECTED_DEYES_NODES_MIN else "WARN"

        uptime = time.monotonic() - started
        print("\033[2J\033[H", end="")  # clear screen, home cursor
        print(f"=== RACE MONITOR  uptime={uptime:.1f}s  now={time.strftime('%H:%M:%S')} ===")
        print(f"T1 nav       amcl={t1_main[0]:7s} ({t1_main[1]:30s})  scan={t1_sensor[0]:7s} ({t1_sensor[1]})")
        print(f"T2 adapter   evidence={t2[0]:7s} ({t2[1]})")
        print(f"T3 bridge    {t3_status:7s} ({bridge_detail})")
        print(f"T4 Deyes     {t4_status:7s} ({ros2_detail})")
        print(f"T5 mission   mission={mission.status()[0]:7s} ({mission.status()[1]})")
        print(f"T6 monitor   OK        (this script, pid {__import__('os').getpid()})")

        if last_evidence_payload:
            short = {k: last_evidence_payload.get(k) for k in ("mission_id", "result", "reason")}
            print(f"  last evidence : {json.dumps(short, ensure_ascii=False)}")
        if last_mission_payload:
            short = {k: last_mission_payload.get(k) for k in ("mission_id", "target_id", "nav_epoch")}
            print(f"  last mission  : {json.dumps(short, ensure_ascii=False)}")

        if history:
            print("  recent events :")
            for ev in list(history)[-6:]:
                print(f"    {time.strftime('%H:%M:%S', time.localtime(ev['ts']))} {ev}")

        rospy.sleep(1.0)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
