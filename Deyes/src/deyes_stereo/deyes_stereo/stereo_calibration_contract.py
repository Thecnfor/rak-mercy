"""ROS-independent contract checks for the physical stereo calibration tool."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable, Sequence


BOARD_INNER_CORNERS = (9, 6)
CALIBRATION_SIZE = (640, 360)
MIN_SAMPLES = 40
MAX_SAMPLES = 60
MAX_PAIR_SKEW_MS = 10.0
MAX_REPROJ_RMS_PX = 0.50
MAX_EPIPOLAR_P95_PX = 0.50


@dataclass(frozen=True)
class CalibrationGate:
    validated: bool
    reasons: tuple[str, ...]


def validate_capture_arguments(
    *,
    square_size_m: float,
    requested_samples: int,
    width: int,
    height: int,
) -> tuple[str, ...]:
    """Return human-readable preflight failures without guessing physical data."""
    reasons: list[str] = []
    if not math.isfinite(square_size_m) or square_size_m <= 0.0:
        reasons.append("square_size_m_must_be_explicit_and_positive")
    if not MIN_SAMPLES <= requested_samples <= MAX_SAMPLES:
        reasons.append("requested_samples_must_be_between_40_and_60")
    if (width, height) != CALIBRATION_SIZE:
        reasons.append("resolution_must_be_exactly_640x360")
    return tuple(reasons)


def validation_gate(
    *,
    sample_count: int,
    resolution: Sequence[int],
    reproj_rms_px: float | None,
    epipolar_p95_px: float | None,
    source: str,
    left_right_confirmed: bool,
    baseline_sign_confirmed: bool,
    scale_confirmed: bool,
    coverage_complete: bool,
) -> CalibrationGate:
    """Centralized, deliberately conservative rule for a trusted YAML."""
    reasons: list[str] = []
    if not MIN_SAMPLES <= sample_count <= MAX_SAMPLES:
        reasons.append("valid_sample_count_not_in_40_to_60")
    if tuple(resolution) != CALIBRATION_SIZE:
        reasons.append("resolution_not_640x360")
    if source != "physical_checkerboard":
        reasons.append("source_is_not_physical_checkerboard")
    if reproj_rms_px is None or reproj_rms_px > MAX_REPROJ_RMS_PX:
        reasons.append("reproj_rms_exceeds_0_50_px")
    if epipolar_p95_px is None or epipolar_p95_px > MAX_EPIPOLAR_P95_PX:
        reasons.append("epipolar_p95_exceeds_0_50_px")
    if not coverage_complete:
        reasons.append("capture_coverage_incomplete")
    if not left_right_confirmed:
        reasons.append("operator_left_right_not_confirmed")
    if not baseline_sign_confirmed:
        reasons.append("operator_baseline_sign_not_confirmed")
    if not scale_confirmed:
        reasons.append("operator_scale_not_confirmed")
    return CalibrationGate(validated=not reasons, reasons=tuple(reasons))


def coverage_cells(corners: Iterable[Sequence[float]], width: int, height: int) -> set[tuple[int, int]]:
    """Map board centres to a 3x3 image grid for capture coverage auditing."""
    result: set[tuple[int, int]] = set()
    for centre in corners:
        x, y = float(centre[0]), float(centre[1])
        column = min(2, max(0, int(3.0 * x / width)))
        row = min(2, max(0, int(3.0 * y / height)))
        result.add((column, row))
    return result


def coverage_complete(cells: Iterable[tuple[int, int]]) -> bool:
    return set(cells) == {(x, y) for x in range(3) for y in range(3)}
