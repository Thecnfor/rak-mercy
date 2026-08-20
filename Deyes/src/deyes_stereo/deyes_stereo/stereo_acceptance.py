"""ROS-free contracts for the X1 stereo truth and endurance acceptance gates.

The module deliberately reports measured quality; it does not tune, filter, or
alter depth samples.  Its command line interface always requires an explicit
output directory outside the source checkout.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence


TRUTH_THRESHOLDS_M: dict[float, tuple[float, float]] = {
    0.30: (0.010, 0.020),
    0.50: (0.010, 0.020),
    0.80: (0.020, 0.030),
    1.00: (0.020, 0.030),
}
MIN_SAMPLES_PER_DISTANCE = 100
MIN_RUNTIME_DURATION_SEC = 600.0
RVIZ_MANUAL_CHECKS = (
    "flat_plane_has_no_obvious_warping_or_layering",
    "no_obvious_ghosting",
    "optical_axes_are_x_right_y_down_z_forward",
)


class AcceptanceInputError(ValueError):
    """The supplied report cannot safely be used for an acceptance decision."""


def _as_finite(value: Any, field: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise AcceptanceInputError(f"invalid_{field}") from exc
    if not math.isfinite(parsed):
        raise AcceptanceInputError(f"invalid_{field}")
    return parsed


def _as_bool(value: Any, field: str) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        normalised = value.strip().lower()
        if normalised in {"true", "1", "yes"}:
            return True
        if normalised in {"false", "0", "no"}:
            return False
    raise AcceptanceInputError(f"invalid_{field}")


def percentile95(values: Sequence[float]) -> float:
    """Nearest-rank P95, documented so reports are reproducible without numpy."""
    if not values:
        raise AcceptanceInputError("p95_requires_at_least_one_value")
    ordered = sorted(values)
    return ordered[math.ceil(0.95 * len(ordered)) - 1]


def _truth_key(value: float) -> float | None:
    for target in TRUTH_THRESHOLDS_M:
        if math.isclose(value, target, rel_tol=0.0, abs_tol=1e-6):
            return target
    return None


@dataclass(frozen=True)
class TruthSample:
    truth_m: float
    measured_m: float | None
    valid: bool
    plane_residual_m: float | None = None


def parse_truth_samples(records: Iterable[Mapping[str, Any]]) -> list[TruthSample]:
    samples: list[TruthSample] = []
    for index, record in enumerate(records, start=1):
        if not isinstance(record, Mapping):
            raise AcceptanceInputError(f"row_{index}_is_not_an_object")
        truth = _as_finite(record.get("truth_m"), f"row_{index}_truth_m")
        if _truth_key(truth) is None:
            raise AcceptanceInputError(f"row_{index}_truth_m_not_a_required_distance")
        valid = _as_bool(record.get("valid", True), f"row_{index}_valid")
        raw_measured = record.get("measured_m", record.get("depth_m", record.get("z_m")))
        measured = None
        if raw_measured not in (None, ""):
            measured = _as_finite(raw_measured, f"row_{index}_measured_m")
        if valid and measured is None:
            raise AcceptanceInputError(f"row_{index}_valid_sample_missing_measured_m")
        plane_residual = None
        if "plane_residual_m" in record and record["plane_residual_m"] not in (None, ""):
            plane_residual = _as_finite(record["plane_residual_m"], f"row_{index}_plane_residual_m")
        samples.append(TruthSample(truth, measured, valid, plane_residual))
    if not samples:
        raise AcceptanceInputError("no_samples")
    return samples


def _stats(values: Sequence[float]) -> dict[str, float]:
    if not values:
        return {}
    return {
        "bias_m": sum(values) / len(values),
        "mae_m": sum(abs(value) for value in values) / len(values),
        "rmse_m": math.sqrt(sum(value * value for value in values) / len(values)),
        "max_abs_error_m": max(abs(value) for value in values),
        "p95_abs_error_m": percentile95([abs(value) for value in values]),
        "repeatability_stddev_m": math.sqrt(sum((value - (sum(values) / len(values))) ** 2 for value in values) / len(values)),
    }


def evaluate_truth_samples(records: Iterable[Mapping[str, Any]]) -> dict[str, Any]:
    samples = parse_truth_samples(records)
    groups: list[dict[str, Any]] = []
    reasons: list[str] = []
    for target, (max_mae, max_p95) in TRUTH_THRESHOLDS_M.items():
        group = [sample for sample in samples if _truth_key(sample.truth_m) == target]
        valid = [sample for sample in group if sample.valid and sample.measured_m is not None]
        errors = [sample.measured_m - target for sample in valid if sample.measured_m is not None]
        plane_values = [abs(sample.plane_residual_m) for sample in valid if sample.plane_residual_m is not None]
        stats = _stats(errors)
        group_reasons: list[str] = []
        if len(group) < MIN_SAMPLES_PER_DISTANCE:
            group_reasons.append("insufficient_samples")
        if len(valid) < MIN_SAMPLES_PER_DISTANCE:
            group_reasons.append("insufficient_valid_measurements")
        if len(plane_values) != len(valid):
            group_reasons.append("plane_residual_missing_for_valid_measurements")
        if stats and stats["mae_m"] > max_mae:
            group_reasons.append("mae_exceeds_limit")
        if stats and stats["p95_abs_error_m"] > max_p95:
            group_reasons.append("p95_abs_error_exceeds_limit")
        if group_reasons:
            reasons.extend(f"{target:.2f}m:{reason}" for reason in group_reasons)
        group_report: dict[str, Any] = {
            "truth_m": target,
            "sample_count": len(group),
            "valid_count": len(valid),
            "valid_ratio": (len(valid) / len(group)) if group else 0.0,
            "limits_m": {"mae": max_mae, "p95_abs_error": max_p95},
            "validated": not group_reasons,
            "reasons": group_reasons,
            **stats,
        }
        if plane_values:
            group_report["plane_residual_m"] = {
                "count": len(plane_values),
                "mean_abs_m": sum(plane_values) / len(plane_values),
                "p95_abs_m": percentile95(plane_values),
                "max_abs_m": max(plane_values),
            }
        groups.append(group_report)
    return {
        "contract": "deyes.stereo_truth_acceptance.v1",
        "required_distances_m": list(TRUTH_THRESHOLDS_M),
        "minimum_samples_per_distance": MIN_SAMPLES_PER_DISTANCE,
        "groups": groups,
        "overall_validated": not reasons,
        "reasons": reasons,
    }


def evaluate_runtime_metrics(metrics: Mapping[str, Any]) -> dict[str, Any]:
    """Apply the fixed 10-minute runtime gate to machine-readable metrics."""
    required = (
        "duration_sec", "capture_failures", "pair_max_skew_ms", "left_image_hz",
        "right_image_hz", "depth_hz", "points_hz", "center_roi_coverage_min",
        "processing_overrun_sustained", "calibration_validated", "calibration_id",
        "rviz_manual_checks", "pair_diagnostics_observed", "depth_status_observed",
        "depth_coverage_observed", "points_status_observed", "pointcloud_status_always_validated",
        "pointcloud_calibration_identity_consistent",
    )
    missing = [name for name in required if name not in metrics]
    if missing:
        raise AcceptanceInputError("missing_runtime_fields:" + ",".join(missing))
    duration = _as_finite(metrics["duration_sec"], "duration_sec")
    failures = _as_finite(metrics["capture_failures"], "capture_failures")
    skew = _as_finite(metrics["pair_max_skew_ms"], "pair_max_skew_ms")
    left_hz = _as_finite(metrics["left_image_hz"], "left_image_hz")
    right_hz = _as_finite(metrics["right_image_hz"], "right_image_hz")
    depth_hz = _as_finite(metrics["depth_hz"], "depth_hz")
    points_hz = _as_finite(metrics["points_hz"], "points_hz")
    coverage = _as_finite(metrics["center_roi_coverage_min"], "center_roi_coverage_min")
    sustained = _as_bool(metrics["processing_overrun_sustained"], "processing_overrun_sustained")
    calibration_validated = _as_bool(metrics["calibration_validated"], "calibration_validated")
    pair_diagnostics_observed = _as_bool(metrics["pair_diagnostics_observed"], "pair_diagnostics_observed")
    depth_status_observed = _as_bool(metrics["depth_status_observed"], "depth_status_observed")
    depth_coverage_observed = _as_bool(metrics["depth_coverage_observed"], "depth_coverage_observed")
    points_status_observed = _as_bool(metrics["points_status_observed"], "points_status_observed")
    pointcloud_status_always_validated = _as_bool(
        metrics["pointcloud_status_always_validated"], "pointcloud_status_always_validated")
    pointcloud_identity_consistent = _as_bool(
        metrics["pointcloud_calibration_identity_consistent"], "pointcloud_calibration_identity_consistent")
    calibration_id = metrics["calibration_id"]
    if not isinstance(calibration_id, str):
        raise AcceptanceInputError("invalid_calibration_id")
    rviz_input = metrics["rviz_manual_checks"]
    if not isinstance(rviz_input, Mapping):
        raise AcceptanceInputError("invalid_rviz_manual_checks")
    rviz_checks = {name: _as_bool(rviz_input.get(name, False), f"rviz_{name}") for name in RVIZ_MANUAL_CHECKS}
    checks = {
        "duration_at_least_600s": duration >= MIN_RUNTIME_DURATION_SEC,
        "capture_failures_zero": failures == 0.0,
        "pair_diagnostics_observed": pair_diagnostics_observed,
        "every_static_pair_at_most_10ms": skew <= 10.0,
        "left_image_at_least_28hz": left_hz >= 28.0,
        "right_image_at_least_28hz": right_hz >= 28.0,
        "depth_at_least_12hz": depth_hz >= 12.0,
        "points_at_least_12hz": points_hz >= 12.0,
        "center_roi_coverage_at_least_85pct": coverage >= 0.85,
        "depth_status_observed": depth_status_observed,
        "depth_coverage_observed": depth_coverage_observed,
        "no_sustained_processing_overrun": not sustained,
        "physical_calibration_validated": calibration_validated and calibration_id.strip() not in {"", "unassigned"},
        "points_status_observed": points_status_observed,
        "pointcloud_status_always_validated": pointcloud_status_always_validated,
        "pointcloud_calibration_identity_consistent": pointcloud_identity_consistent,
        **{f"rviz_{name}": passed for name, passed in rviz_checks.items()},
    }
    return {
        "contract": "deyes.stereo_runtime_acceptance.v1",
        "metrics": dict(metrics),
        "checks": checks,
        "overall_validated": all(checks.values()),
        "reasons": [name for name, passed in checks.items() if not passed],
        "rviz_manual_checks": rviz_checks,
    }


def _load_records(path: Path) -> list[dict[str, Any]]:
    if path.suffix.lower() == ".csv":
        with path.open("r", encoding="utf-8-sig", newline="") as handle:
            return list(csv.DictReader(handle))
    with path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    records = payload.get("samples") if isinstance(payload, dict) else payload
    if not isinstance(records, list):
        raise AcceptanceInputError("truth_input_must_be_a_json_list_or_object_with_samples")
    return records


def _repo_root() -> Path:
    # .../Deyes/src/deyes_stereo/deyes_stereo/stereo_acceptance.py
    return Path(__file__).resolve().parents[4]


def _output_dir(path_value: str) -> Path:
    supplied = Path(path_value).expanduser()
    if not supplied.is_absolute():
        raise AcceptanceInputError("output_dir_must_be_an_absolute_temp_directory")
    path = supplied.resolve()
    try:
        path.relative_to(_repo_root())
    except ValueError:
        path.mkdir(parents=True, exist_ok=True)
        return path
    raise AcceptanceInputError("output_dir_must_be_outside_the_repository_use_temp")


def _markdown(report: Mapping[str, Any], title: str) -> str:
    lines = [f"# {title}", "", f"Generated: {datetime.now(timezone.utc).isoformat()}", "", f"Validated: `{report['overall_validated']}`", ""]
    if report.get("contract") == "deyes.stereo_truth_acceptance.v1":
        lines.extend(["| Truth (m) | Samples | Valid | MAE (m) | P95 abs (m) | Result |", "| --- | ---: | ---: | ---: | ---: | --- |"])
        for group in report["groups"]:
            lines.append(
                f"| {group['truth_m']:.2f} | {group['sample_count']} | {group['valid_count']} | "
                f"{group.get('mae_m', float('nan')):.6f} | {group.get('p95_abs_error_m', float('nan')):.6f} | "
                f"{'PASS' if group['validated'] else 'FAIL'} |"
            )
    else:
        lines.extend(["## Runtime checks", ""])
        lines.extend(f"- {'PASS' if passed else 'FAIL'}: `{name}`" for name, passed in report["checks"].items())
        lines.extend(["", "## RViz manual checks (operator required)", ""])
        lines.extend(f"- {'PASS' if passed else 'FAIL'}: `{item}`" for item, passed in report["rviz_manual_checks"].items())
    if report["reasons"]:
        lines.extend(["", "## Blocking reasons", ""])
        lines.extend(f"- `{reason}`" for reason in report["reasons"])
    return "\n".join(lines) + "\n"


def write_report(report: Mapping[str, Any], output_dir: Path, stem: str, title: str) -> tuple[Path, Path]:
    json_path = output_dir / f"{stem}.json"
    markdown_path = output_dir / f"{stem}.md"
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    markdown_path.write_text(_markdown(report, title), encoding="utf-8")
    return json_path, markdown_path


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="X1 stereo truth/runtime acceptance reporter")
    subparsers = parser.add_subparsers(dest="mode", required=True)
    truth = subparsers.add_parser("truth", help="evaluate CSV/JSON truth samples")
    truth.add_argument("--input", required=True)
    truth.add_argument("--output-dir", required=True, help="absolute temp directory outside this repository")
    runtime = subparsers.add_parser("runtime", help="evaluate a runtime metrics JSON object")
    runtime.add_argument("--input", required=True)
    runtime.add_argument("--output-dir", required=True, help="absolute temp directory outside this repository")
    args = parser.parse_args(argv)
    try:
        output_dir = _output_dir(args.output_dir)
        with Path(args.input).expanduser().open("r", encoding="utf-8") as handle:
            runtime_payload = json.load(handle) if args.mode == "runtime" else None
        if args.mode == "truth":
            report = evaluate_truth_samples(_load_records(Path(args.input).expanduser()))
            paths = write_report(report, output_dir, "truth_acceptance", "X1 Stereo Truth Acceptance")
        else:
            if not isinstance(runtime_payload, Mapping):
                raise AcceptanceInputError("runtime_input_must_be_a_json_object")
            report = evaluate_runtime_metrics(runtime_payload)
            paths = write_report(report, output_dir, "runtime_acceptance", "X1 Stereo Runtime Acceptance")
    except (OSError, json.JSONDecodeError, AcceptanceInputError) as exc:
        parser.error(str(exc))
    print(f"json: {paths[0]}\nmarkdown: {paths[1]}\nvalidated={report['overall_validated']}")
    return 0 if report["overall_validated"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
