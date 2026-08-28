#!/usr/bin/env python3
"""Automatically stow both arms only when hash-bound Isaac evidence admits it."""
from __future__ import annotations

import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess

from deyes_stereo.competition_arm_stow import execute_stow_plan, load_stow_plan


def _owners(port: str) -> list[int]:
    completed = subprocess.run(["lsof", "-t", "--", port], capture_output=True, text=True, check=False)
    return [int(value) for value in completed.stdout.split() if value.isdigit()]


def _lock(side: str):
    path = Path(f"/tmp/deyes_competition_{side}_arm.lock")
    handle = path.open("a+")
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        handle.close()
        raise RuntimeError(f"arm_lock_unavailable:{side}")
    return handle


def _unlock(handle) -> None:
    fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
    handle.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--venue-profile", type=Path, required=True)
    parser.add_argument("--result-json", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    commands_emitted = False
    try:
        plan = load_stow_plan(args.venue_profile)
        if args.dry_run:
            result = {"schema": "competition_arm_stow_result/v1", "success": True,
                      "dry_run": True, "order": list(plan.order), "commands_emitted": False}
        else:
            from pymycobot import Mercury

            result = execute_stow_plan(
                plan,
                mercury_factory=lambda port: Mercury(port, 115200),
                serial_owner_scan=_owners,
                lock_acquire=_lock,
                lock_release=_unlock,
            )
            commands_emitted = result["commands_emitted"] is True
    except (ImportError, OSError, RuntimeError, ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
        result = {"schema": "competition_arm_stow_result/v1", "success": False,
                  "reason": str(exc), "commands_emitted": commands_emitted}
    args.result_json.parent.mkdir(parents=True, exist_ok=True)
    args.result_json.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, sort_keys=True))
    return 0 if result.get("success") else 2


if __name__ == "__main__":
    raise SystemExit(main())
