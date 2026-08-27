#!/usr/bin/env python3
"""Write an auditable tier-C success trace and fail-closed fault matrix."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

from deyes_stereo.competition_fullchain_sim import (
    CompetitionFullChain,
    FAIL_CLOSED_FAULTS,
    SimulatedCompetitionAdapter,
)


def run_matrix() -> dict[str, object]:
    nominal = CompetitionFullChain(SimulatedCompetitionAdapter()).run()
    degraded = CompetitionFullChain(
        SimulatedCompetitionAdapter(fault="projector_unavailable"),
        force_fixed_target=True,
    ).run()
    warnings = {
        fault: CompetitionFullChain(SimulatedCompetitionAdapter(fault=fault)).run()
        for fault in ("plane_missing", "plane_low_quality")
    }
    failures = {
        fault: CompetitionFullChain(SimulatedCompetitionAdapter(fault=fault)).run()
        for fault in FAIL_CLOSED_FAULTS
    }
    accepted = (
        nominal["state"] == "completed"
        and degraded["state"] == "completed"
        and all(result["state"] == "completed" for result in warnings.values())
        and all(
            result["state"] == "failed"
            and result["retry_count"] == 0
            and result["physical_validated"] is False
            for result in failures.values()
        )
    )
    return {
        "schema": "competition_65cm_fixture_matrix/v1",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "classification": "C_synthetic_fixture",
        "accepted": accepted,
        "nominal": nominal,
        "explicit_fixed_xy_degraded": degraded,
        "plane_warning_cases": warnings,
        "fail_closed_cases": failures,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = run_matrix()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(args.output)
    return 0 if result["accepted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
