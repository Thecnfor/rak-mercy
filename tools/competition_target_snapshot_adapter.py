#!/usr/bin/env python3
"""Snapshot one executable competition target from a ROS 2 String topic.

This adapter never synthesizes a target. Placeholder/waiting messages fail closed.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/x1/competition/pick_target")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as exc:
        raise SystemExit(f"ROS 2 Python unavailable: {exc}") from exc

    rclpy.init(args=None)
    node = Node("competition_target_snapshot_adapter")
    payload: dict[str, object] | None = None
    error: str | None = None

    def receive(message: String) -> None:
        nonlocal payload, error
        try:
            candidate = json.loads(message.data)
            if not isinstance(candidate, dict):
                raise ValueError("target payload must be an object")
            status = str(candidate.get("status", "")).lower()
            if status.startswith("waiting") or candidate.get("reason") == "waiting_for_exact_stamp_projector_adapter":
                raise ValueError("target node is still a non-executable waiting placeholder")
            if candidate.get("schema") != "competition_pick_target/v1":
                raise ValueError("target schema must be competition_pick_target/v1")
            if candidate.get("valid") is not True:
                raise ValueError("target is not valid")
            # Trust/explicit fixed-target policy is evaluated by the runner, which
            # has the operator's FORCE_FIXED_TARGET setting. The adapter only snapshots.
            payload = candidate
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            error = str(exc)

    node.create_subscription(String, args.topic, receive, 10)
    deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
    try:
        while payload is None and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node(); rclpy.shutdown()
    if payload is None:
        print(error or "timed out waiting for competition target", file=sys.stderr)
        return 2
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
