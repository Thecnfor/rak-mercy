#!/usr/bin/env python3
"""Validate two raw Isaac runs and unlock the venue profile only on a real pass."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/deyes_stereo"))

from deyes_stereo.competition_clearance_evidence import (  # noqa: E402
    SCHEMA,
    TARGET_SCOPE,
    canonical_sha256,
    motion_contract,
    sha256_file,
    validate_clearance_evidence,
)


EXIT_ASSET = 20
EXIT_COLLISION = 21
EXIT_CONTACT = 22
EXIT_CLEARANCE = 23
EXIT_FLOW = 24


def classify_failure(reason: str) -> int:
    if "asset" in reason or "collision_prim" in reason or "required_collision" in reason:
        return EXIT_ASSET
    if "forbidden_contact" in reason or "synthetic" in reason or "teleport" in reason:
        return EXIT_CONTACT
    if "clearance" in reason or "fingertip" in reason or "repeat" in reason:
        return EXIT_CLEARANCE
    if "collision" in reason:
        return EXIT_COLLISION
    return EXIT_FLOW


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--raw-run", type=Path, action="append", required=True)
    parser.add_argument("--output-evidence", type=Path, required=True)
    parser.add_argument("--output-profile", type=Path, required=True)
    args = parser.parse_args()
    if len(args.raw_run) != 2:
        parser.error("exactly two --raw-run files are required")
    profile = yaml.safe_load(args.profile.read_text(encoding="utf-8"))
    contract_sha = canonical_sha256(motion_contract(profile))
    raw = [json.loads(path.read_text(encoding="utf-8")) for path in args.raw_run]
    first = raw[0]
    evidence = {
        "schema": SCHEMA,
        "passed": all(item.get("passed") is True for item in raw),
        "target_scope": TARGET_SCOPE,
        "motion_contract_sha256": contract_sha,
        "assets": first.get("assets"),
        "simulation": first.get("simulation"),
        "initial_pose": first.get("initial_pose"),
        "runs": [item.get("run") for item in raw],
        "conservative_clearance_mm": min(
            float(item.get("run", {}).get("minimum_arm_conservative_mm", 0.0)) for item in raw
        ),
        "raw_run_sha256": [sha256_file(path) for path in args.raw_run],
    }
    valid, reason, clearance = validate_clearance_evidence(
        evidence, expected_motion_sha256=contract_sha
    )
    evidence["passed"] = valid
    evidence["reason"] = reason
    args.output_evidence.parent.mkdir(parents=True, exist_ok=True)
    args.output_evidence.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if not valid:
        print(json.dumps({"passed": False, "reason": reason, "profile_updated": False}))
        return classify_failure(reason)
    profile["transport"].update({
        "transport_validated": True,
        "collision_clearance_validated": True,
        "validation_reason": "hash_bound_isaac_real_clearance_acceptance",
        "tcp_vertical_clearance_conservative_mm": clearance,
        "clearance_evidence_manifest": args.output_evidence.name,
        "clearance_evidence_sha256": hashlib.sha256(args.output_evidence.read_bytes()).hexdigest(),
    })
    args.output_profile.parent.mkdir(parents=True, exist_ok=True)
    args.output_profile.write_text(yaml.safe_dump(profile, sort_keys=False), encoding="utf-8")
    print(json.dumps({"passed": True, "reason": "ok", "profile_updated": True,
                      "conservative_clearance_mm": clearance}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
