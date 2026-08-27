"""Pure validation helpers for the fixed-venue perception admission gate."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class GateResult:
    accepted: bool
    reason: str
    depth_m: float | None = None
    valid_pixels: int = 0
    coverage: float = 0.0
    bbox_xyxy: tuple[int, int, int, int] | None = None


def stamp_ns(payload: dict[str, Any]) -> int:
    return int(payload.get("stamp_sec", 0)) * 1_000_000_000 + int(
        payload.get("stamp_nanosec", 0)
    )


def evaluate_detection_depth(
    payload: dict[str, Any],
    depth: np.ndarray,
    *,
    min_depth_m: float = 0.20,
    max_depth_m: float = 1.00,
    min_valid_pixels: int = 8,
    min_coverage: float = 0.03,
) -> GateResult:
    """Accept exactly one pen whose bounding box contains usable metric depth."""
    detections = payload.get("detections")
    if not isinstance(detections, list):
        return GateResult(False, "detections_missing")
    if payload.get("auto_grasp_permitted") is not True:
        return GateResult(False, str(payload.get("rejection_reason") or "detector_rejected"))
    if len(detections) != 1 or int(payload.get("detection_count", len(detections))) != 1:
        return GateResult(False, f"expected_one_pen_got_{len(detections)}")
    if depth.ndim != 2 or depth.size == 0:
        return GateResult(False, "invalid_depth_image")

    bbox = detections[0].get("bbox_xyxy")
    if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
        return GateResult(False, "bbox_missing")
    height, width = depth.shape
    try:
        x0, y0, x1, y1 = (int(round(float(value))) for value in bbox)
    except (TypeError, ValueError):
        return GateResult(False, "bbox_invalid")
    x0, x1 = max(0, min(x0, width)), max(0, min(x1, width))
    y0, y1 = max(0, min(y0, height)), max(0, min(y1, height))
    if x1 <= x0 or y1 <= y0:
        return GateResult(False, "bbox_outside_image")

    # Use the middle 60% to reduce background leakage around a thin pen.
    inset_x = int((x1 - x0) * 0.20)
    inset_y = int((y1 - y0) * 0.20)
    roi = depth[y0 + inset_y : y1 - inset_y, x0 + inset_x : x1 - inset_x]
    if roi.size == 0:
        roi = depth[y0:y1, x0:x1]
    valid = np.isfinite(roi) & (roi >= min_depth_m) & (roi <= max_depth_m)
    count = int(np.count_nonzero(valid))
    coverage = count / float(roi.size)
    box = (x0, y0, x1, y1)
    if count < min_valid_pixels:
        return GateResult(False, "insufficient_valid_depth", valid_pixels=count, coverage=coverage, bbox_xyxy=box)
    if coverage < min_coverage:
        return GateResult(False, "depth_coverage_too_low", valid_pixels=count, coverage=coverage, bbox_xyxy=box)
    median = float(np.median(roi[valid]))
    return GateResult(True, "ok", median, count, coverage, box)
