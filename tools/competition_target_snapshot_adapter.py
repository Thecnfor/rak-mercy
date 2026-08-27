#!/usr/bin/env python3
"""Snapshot one live target or build the same target from an offline fixture.

Exit codes: 0 accepted snapshot, 2 timeout, 3 terminal fail-closed target,
4 malformed input/configuration.  No mode synthesizes a target.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
from typing import Any, Sequence


def _target_builder():
    try:
        from deyes_stereo.competition_pick_target_node import build_target_from_fixture
    except ModuleNotFoundError:
        # Repository checkout convenience; deployed runs import the colcon install.
        package = Path(__file__).resolve().parents[1] / "Deyes" / "src" / "deyes_stereo"
        if package.is_dir():
            sys.path.insert(0, str(package))
        from deyes_stereo.competition_pick_target_node import build_target_from_fixture
    return build_target_from_fixture


def validate_target(candidate: Any) -> tuple[dict[str, Any] | None, str | None, int | None]:
    """Return payload/error/exit-code; ``None`` code means keep waiting."""
    if not isinstance(candidate, dict):
        return None, "target payload must be an object", 4
    status = str(candidate.get("status", "")).lower()
    if status.startswith("waiting") or candidate.get("reason") == "waiting_for_exact_stamp_projector_adapter":
        return None, "target node is still a non-executable waiting placeholder", None
    if candidate.get("schema") != "competition_pick_target/v1":
        return None, "target schema must be competition_pick_target/v1", 4
    if candidate.get("valid") is not True:
        return None, "target rejected: " + str(candidate.get("reason", "unknown")), 3
    target = candidate.get("right_arm_sdk_target_m")
    orientation = candidate.get("orientation_deg")
    pixel = candidate.get("pixel_uv")
    geometry_ok = (
        candidate.get("commands_emitted") is False
        and isinstance(target, list)
        and len(target) == 3
        and all(isinstance(value, (int, float)) and math.isfinite(value) for value in target)
        and math.isclose(float(target[2]), 0.135, abs_tol=1e-12)
        and orientation == [179.99, -12.0, 0.0]
        and isinstance(pixel, list)
        and len(pixel) == 2
        and all(isinstance(value, (int, float)) and math.isfinite(value) for value in pixel)
    )
    if not geometry_ok:
        return None, "target geometry/command-state contract invalid", 3
    if candidate.get("trusted_for_venue_execution") is True:
        return candidate, None, 0
    fixed = (
        candidate.get("selection_source") == "fixed_xy_fallback"
        and candidate.get("degraded") is True
        and candidate.get("force_fixed_target") is True
        and candidate.get("execution_allowed") is True
        and candidate.get("right_arm_sdk_target_m") == [0.4, 0.01, 0.135]
        and "[400,10]mm" in str(candidate.get("manual_action_required", ""))
    )
    if fixed:
        return candidate, None, 0
    return None, "untrusted random/projected XY is not snapshot-eligible", 3


def _write(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def run_fixture(args: argparse.Namespace) -> int:
    if args.force_fixed_target and os.environ.get("FORCE_FIXED_TARGET") != "1":
        print(
            "fixture/configuration error: --force-fixed-target requires FORCE_FIXED_TARGET=1",
            file=sys.stderr,
        )
        return 4
    try:
        result = _target_builder()(
            args.fixture,
            args.venue_profile,
            args.projector,
            allow_bbox_center=args.allow_bbox_center,
            force_fixed_target=args.force_fixed_target,
        )
    except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
        print(f"fixture/configuration error: {exc}", file=sys.stderr)
        return 4
    _write(args.output, result)
    payload, error, code = validate_target(result)
    if payload is not None:
        return 0
    print(error or "fixture target rejected", file=sys.stderr)
    return 3 if code is None else code


def run_topic(args: argparse.Namespace) -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as exc:
        print(f"ROS 2 Python unavailable: {exc}", file=sys.stderr)
        return 4

    rclpy.init(args=None)
    node = Node("competition_target_snapshot_adapter")
    payload: dict[str, Any] | None = None
    error: str | None = None
    terminal_code: int | None = None

    def receive(message: String) -> None:
        nonlocal payload, error, terminal_code
        try:
            candidate = json.loads(message.data)
        except json.JSONDecodeError as exc:
            error, terminal_code = f"target JSON malformed: {exc}", 4
            return
        accepted, validation_error, code = validate_target(candidate)
        if accepted is not None:
            payload, terminal_code = accepted, 0
        elif code is not None:
            error, terminal_code = validation_error, code
        else:
            error = validation_error

    node.create_subscription(String, args.topic, receive, 10)
    deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
    try:
        while (
            payload is None
            and terminal_code is None
            and node.get_clock().now().nanoseconds < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if payload is None:
        if terminal_code is not None:
            print(error or "terminal competition target rejection", file=sys.stderr)
            return terminal_code
        print(error or "timed out waiting for competition target", file=sys.stderr)
        return 2
    _write(args.output, payload)
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--fixture", type=Path)
    mode.add_argument("--topic", default="/x1/competition/pick_target")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--projector", type=Path)
    parser.add_argument("--venue-profile", type=Path)
    parser.add_argument("--allow-bbox-center", action="store_true")
    parser.add_argument("--force-fixed-target", action="store_true")
    args = parser.parse_args(argv)
    if args.fixture is not None:
        if args.projector is None or args.venue_profile is None:
            parser.error("--fixture requires --projector and --venue-profile")
        return run_fixture(args)
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    return run_topic(args)


if __name__ == "__main__":
    raise SystemExit(main())
