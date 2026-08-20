"""Physical point-correspondence solver for ``base_link_T_left_camera``.

Operators collect the same physical feature (for example six checkerboard
corners touched by a calibrated tool) in camera and base coordinates.  This is
an eye-to-hand calibration: it deliberately does not accept guessed offsets.
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .extrinsics_contract import matrix_to_quaternion, solve_base_from_camera


def build_document(payload: dict[str, Any], *, max_rms_m: float = 0.005, max_p95_m: float = 0.010) -> dict[str, Any]:
    samples = payload.get("correspondences") or []
    camera = np.asarray([item["camera_point_m"] for item in samples], dtype=np.float64)
    base = np.asarray([item["base_point_m"] for item in samples], dtype=np.float64)
    rotation, translation, residuals = solve_base_from_camera(camera, base)
    rms = float(np.sqrt(np.mean(np.square(residuals))))
    p95 = float(np.percentile(residuals, 95))
    required = ("calibration_id", "robot_id", "camera_pair_id", "stereo_calibration_id")
    missing = [key for key in required if not str(payload.get(key) or "").strip()]
    if missing:
        raise ValueError("missing_identity:" + ",".join(missing))
    return {
        "calibration_id": str(payload["calibration_id"]),
        "robot_id": str(payload["robot_id"]),
        "camera_pair_id": str(payload["camera_pair_id"]),
        "stereo_calibration_id": str(payload["stereo_calibration_id"]),
        "date": datetime.now(timezone.utc).date().isoformat(),
        "source": "physical_point_correspondences",
        "source_frame": "left_camera_optical_frame",
        "target_frame": "base_link",
        "translation_m": [float(v) for v in translation],
        "quaternion_xyzw": matrix_to_quaternion(rotation),
        "metrics": {
            "correspondence_count": int(len(samples)), "rms_m": rms, "p95_m": p95,
            "max_m": float(np.max(residuals)), "max_rms_m": max_rms_m, "max_p95_m": max_p95_m,
        },
        "validated": bool(rms <= max_rms_m and p95 <= max_p95_m),
        "operator_confirmation": bool(payload.get("operator_confirmation", False)),
        "samples_file": str(payload.get("samples_file") or "operator_supplied"),
    }


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="JSON physical point correspondences")
    parser.add_argument("--output", required=True, help="YAML output under temp/deyes/calibration")
    parser.add_argument("--max-rms-m", type=float, default=0.005)
    parser.add_argument("--max-p95-m", type=float, default=0.010)
    args = parser.parse_args(argv)
    input_path, output_path = Path(args.input).expanduser(), Path(args.output).expanduser()
    with input_path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise SystemExit("input JSON root must be an object")
    payload["samples_file"] = str(input_path)
    document = build_document(payload, max_rms_m=args.max_rms_m, max_p95_m=args.max_p95_m)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(document, handle, allow_unicode=True, sort_keys=False)
    print(json.dumps({"output": str(output_path), "validated": document["validated"], "metrics": document["metrics"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
