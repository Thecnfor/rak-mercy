"""Deterministic classical-CV pen pixels and exact-stamp pairing (ROS-free)."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from .stamp_pairing import ExactStampPairCache


@dataclass(frozen=True)
class RectifiedImage:
    stamp_ns: int; frame_id: str; width: int; height: int; gray: np.ndarray


@dataclass(frozen=True)
class ExtractorParams:
    min_mask_pixels: int = 12
    max_mask_pixels: int = 800
    min_axis_length_px: float = 18.0
    min_aspect_ratio: float = 2.0
    min_pca_ratio: float = 2.0
    edge_margin_px: int = 12
    min_contrast: int = 8
    morphology_kernel_px: int = 5


def detection_stamp_ns(payload: dict[str, Any]) -> int:
    return int(payload.get("stamp_sec", 0) or 0) * 1_000_000_000 + int(payload.get("stamp_nanosec", 0) or 0)


def _one_pen(payload: dict[str, Any]) -> tuple[dict[str, Any] | None, str]:
    if payload.get("ambiguous") or payload.get("rejection_reason") == "ambiguous_multi_target":
        return None, "ambiguous_multi_target"
    detections = [item for item in payload.get("detections", []) if isinstance(item, dict) and (item.get("class_name") == "pen" or item.get("label") == "pen")]
    if len(detections) == 0:
        return None, "waiting_for_one_pen"
    if len(detections) != 1:
        return None, "ambiguous_multi_target"
    return detections[0], "ok"


def _frame_contract_reason(image: RectifiedImage, payload: dict[str, Any]) -> str | None:
    """Validate frame-scoped metadata before any per-box pixel operation."""
    if detection_stamp_ns(payload) != image.stamp_ns:
        return "detection_image_stamp_mismatch"
    if str(payload.get("frame_id") or "") != image.frame_id:
        return "detection_image_frame_mismatch"
    if int(payload.get("image_width", 0) or 0) != image.width or int(payload.get("image_height", 0) or 0) != image.height:
        return "detection_image_size_mismatch"
    return None


def _bbox_sort_key(detection: dict[str, Any], source_index: int) -> tuple[float, ...]:
    """Make a fallback identity order independent of detector list ordering."""
    bbox = detection.get("bbox_xyxy")
    if not isinstance(bbox, list) or len(bbox) != 4:
        return (float("inf"), float("inf"), float("inf"), float("inf"), float(source_index))
    try:
        x0, y0, x1, y1 = (float(value) for value in bbox)
        return ((x0 + x1) * .5, (y0 + y1) * .5, -float(detection.get("confidence", 0.0) or 0.0), x0, float(source_index))
    except (TypeError, ValueError):
        return (float("inf"), float("inf"), float("inf"), float("inf"), float(source_index))


def _normalise_pen_detections(payload: dict[str, Any]) -> list[dict[str, Any]]:
    """Return pen boxes with unique, deterministic per-frame identities.

    YOLO's target gate already emits stable ``det_index``/``target_id`` values.
    This defensive normalization also makes hand-authored or object-fusion
    payloads safe: duplicate indexes are reassigned in a geometry-stable order
    and duplicate IDs receive a deterministic ``__det_XX`` suffix.
    """
    payload_detections = payload.get("detections", [])
    if not isinstance(payload_detections, list):
        return []
    raw = [
        (source_index, dict(item))
        for source_index, item in enumerate(payload_detections)
        if isinstance(item, dict) and (item.get("class_name") == "pen" or item.get("label") == "pen")
    ]
    supplied_indexes: set[int] = set()
    indexed: list[tuple[int, dict[str, Any], int | None]] = []
    for source_index, detection in raw:
        try:
            index = int(detection["det_index"])
            if index < 0 or index in supplied_indexes:
                index = None
            else:
                supplied_indexes.add(index)
        except (KeyError, TypeError, ValueError):
            index = None
        indexed.append((source_index, detection, index))

    next_index = 0
    assigned: list[tuple[int, dict[str, Any], int]] = []
    for source_index, detection, index in sorted(indexed, key=lambda item: _bbox_sort_key(item[1], item[0])):
        if index is None:
            while next_index in supplied_indexes:
                next_index += 1
            index = next_index
            supplied_indexes.add(index)
            next_index += 1
        assigned.append((source_index, detection, index))

    used_ids: set[str] = set()
    normalised: list[dict[str, Any]] = []
    for _, detection, index in sorted(assigned, key=lambda item: item[2]):
        requested_id = str(detection.get("target_id") or "").strip()
        target_id = requested_id or f"target_{index:02d}"
        if target_id in used_ids:
            target_id = f"{target_id}__det_{index:02d}"
            suffix = 1
            while target_id in used_ids:
                target_id = f"{requested_id or 'target'}__det_{index:02d}_{suffix}"
                suffix += 1
        used_ids.add(target_id)
        detection["det_index"] = index
        detection["target_id"] = target_id
        normalised.append(detection)
    return normalised


def _component(gray: np.ndarray, bbox: list[float], params: ExtractorParams) -> tuple[np.ndarray | None, str]:
    h, w = gray.shape[:2]
    x0, y0, x1, y1 = [int(round(float(v))) for v in bbox]
    x0, y0, x1, y1 = max(0, x0), max(0, y0), min(w, x1), min(h, y1)
    if x1 - x0 < 4 or y1 - y0 < 4: return None, "invalid_bbox"
    roi = gray[y0:y1, x0:x1]
    blur = cv2.GaussianBlur(roi, (0, 0), 5.0)
    high = cv2.absdiff(roi, blur)
    _, contrast = cv2.threshold(high, params.min_contrast, 255, cv2.THRESH_BINARY)
    edges = cv2.Canny(roi, max(5, params.min_contrast), max(20, params.min_contrast * 3))
    mask = cv2.bitwise_or(contrast, cv2.dilate(edges, np.ones((3, 3), np.uint8)))
    k = max(3, int(params.morphology_kernel_px) | 1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((k, k), np.uint8))
    count, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
    roi_center = np.array([(x1 - x0 - 1) / 2, (y1 - y0 - 1) / 2])
    best: tuple[float, int] | None = None
    for index in range(1, count):
        area = int(stats[index, cv2.CC_STAT_AREA])
        if area < params.min_mask_pixels: continue
        pts = np.column_stack(np.where(labels == index))[:, ::-1].astype(np.float32)
        rect = cv2.minAreaRect(pts); a, b = rect[1]; length, width = max(a, b), max(1.0, min(a, b))
        aspect = length / width
        distance = float(np.linalg.norm(centers[index] - roi_center)) / max(1.0, np.linalg.norm(roi_center))
        score = aspect * min(area, 500) * max(0.0, 1.25 - distance)
        # Keep a weakly elongated component so downstream receives an explicit
        # axis_complete=false rather than a deceptively usable candidate.
        if aspect >= 1.1 and (best is None or score > best[0]): best = (score, index)
    if best is None: return None, "no_elongated_component"
    result = np.zeros_like(mask); result[labels == best[1]] = 255
    full = np.zeros_like(gray); full[y0:y1, x0:x1] = result
    return full, "ok"


def _extract_detection(image: RectifiedImage, detection: dict[str, Any], params: ExtractorParams) -> tuple[dict[str, Any] | None, str]:
    bbox = detection.get("bbox_xyxy")
    if not isinstance(bbox, list) or len(bbox) != 4: return None, "invalid_bbox"
    mask, reason = _component(image.gray, bbox, params)
    if mask is None: return None, reason
    ys, xs = np.where(mask > 0)
    if len(xs) < params.min_mask_pixels: return None, "mask_too_small"
    points = np.column_stack((xs, ys)).astype(np.float64); center = points.mean(axis=0)
    values, vectors = np.linalg.eigh(np.cov((points - center).T)); order = np.argsort(values)[::-1]
    major, minor, axis = float(values[order[0]]), float(values[order[1]]), vectors[:, order[0]]
    ratio = major / max(minor, 1e-9); projections = (points - center) @ axis
    endpoints = np.stack((center + axis * projections.min(), center + axis * projections.max()))
    axis_length = float(np.linalg.norm(endpoints[1] - endpoints[0])); aspect = axis_length / max(1.0, 2.0 * np.sqrt(max(minor, 1e-9)))
    near_edge = bool(np.any(endpoints[:, 0] <= params.edge_margin_px) or np.any(endpoints[:, 0] >= image.width - 1 - params.edge_margin_px) or np.any(endpoints[:, 1] <= params.edge_margin_px) or np.any(endpoints[:, 1] >= image.height - 1 - params.edge_margin_px))
    complete = axis_length >= params.min_axis_length_px and aspect >= params.min_aspect_ratio and ratio >= params.min_pca_ratio and not near_edge
    if len(points) > params.max_mask_pixels: points = points[np.linspace(0, len(points) - 1, params.max_mask_pixels, dtype=int)]
    target_id = str(detection.get("target_id") or "target_00")
    feature = {"label": "pen", "class_name": "pen", "id": target_id, "target_id": target_id, "det_index": int(detection.get("det_index", 0) or 0), "confidence": float(detection.get("confidence", 0.0) or 0.0), "bbox_xyxy": [float(v) for v in bbox], "mask_pixels_px": [[int(x), int(y)] for x, y in points], "axis_endpoints_px": [[round(float(v), 3) for v in point] for point in endpoints], "axis_complete": bool(complete), "quality": {"mask_pixel_count": int(len(xs)), "axis_length_px": round(axis_length, 3), "aspect_ratio": float(round(aspect, 3)), "pca_ratio": float(round(ratio, 3)), "near_image_edge": near_edge}}
    return feature, "ok" if complete else "axis_incomplete"


def extract_one_pen(image: RectifiedImage, payload: dict[str, Any], params: ExtractorParams = ExtractorParams()) -> tuple[dict[str, Any] | None, str]:
    """Compatibility API for call sites that intentionally require one pen."""
    detection, reason = _one_pen(payload)
    if detection is None: return None, reason
    frame_reason = _frame_contract_reason(image, payload)
    if frame_reason is not None: return None, frame_reason
    return _extract_detection(image, detection, params)


def extract_pen_features(
    image: RectifiedImage, payload: dict[str, Any], params: ExtractorParams = ExtractorParams(),
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], str | None]:
    """Extract every pen independently after one exact frame-contract gate.

    Returns ``(features, target_rejections, frame_rejection_reason)``.  A
    frame rejection suppresses *all* boxes; a box rejection only suppresses
    that target and never fabricates endpoints or an axis for it.
    """
    frame_reason = _frame_contract_reason(image, payload)
    if frame_reason is not None:
        return [], [], frame_reason
    if payload.get("ambiguous") or payload.get("rejection_reason") == "ambiguous_multi_target":
        return [], [], "ambiguous_multi_target"
    detections = _normalise_pen_detections(payload)
    if not detections:
        return [], [], "waiting_for_one_pen"
    features: list[dict[str, Any]] = []
    rejections: list[dict[str, Any]] = []
    for detection in detections:
        try:
            feature, reason = _extract_detection(image, detection, params)
        except (TypeError, ValueError, cv2.error) as exc:
            feature, reason = None, f"invalid_bbox:{exc}"
        if feature is not None:
            features.append(feature)
            continue
        rejection: dict[str, Any] = {
            "target_id": str(detection["target_id"]), "det_index": int(detection["det_index"]), "reason": reason,
        }
        bbox = detection.get("bbox_xyxy")
        if isinstance(bbox, list) and len(bbox) == 4:
            try:
                rejection["bbox_xyxy"] = [float(value) for value in bbox]
            except (TypeError, ValueError):
                pass
        rejections.append(rejection)
    return features, rejections, None


def build_feature_payload(
    image: RectifiedImage,
    feature: dict[str, Any] | None,
    reason: str,
) -> dict[str, Any]:
    """Build a frame-scoped result, including an explicit empty result."""
    return {
        "stamp_sec": image.stamp_ns // 1_000_000_000,
        "stamp_nanosec": image.stamp_ns % 1_000_000_000,
        "frame_id": image.frame_id,
        "source_frame": image.frame_id,
        "image_width": image.width,
        "image_height": image.height,
        "features": [] if feature is None else [feature],
        "axis_complete": bool(feature is not None and feature.get("axis_complete", False)),
        "rejection_reason": None if feature is not None else reason,
    }


def build_features_payload(
    image: RectifiedImage, features: list[dict[str, Any]], target_rejections: list[dict[str, Any]],
    frame_rejection_reason: str | None = None,
) -> dict[str, Any]:
    """Build the multi-target frame schema consumed by the depth/grasp node."""
    copied_features = list(features)
    copied_rejections = list(target_rejections)
    return {
        "stamp_sec": image.stamp_ns // 1_000_000_000,
        "stamp_nanosec": image.stamp_ns % 1_000_000_000,
        "frame_id": image.frame_id,
        "source_frame": image.frame_id,
        "image_width": image.width,
        "image_height": image.height,
        "features": copied_features,
        "axis_complete": bool(copied_features) and all(bool(feature.get("axis_complete", False)) for feature in copied_features),
        "detection_count": len(copied_features) + len(copied_rejections),
        "success_count": len(copied_features),
        "failure_count": len(copied_rejections),
        "axis_incomplete_count": sum(not bool(feature.get("axis_complete", False)) for feature in copied_features),
        "target_rejections": copied_rejections,
        "rejection_reason": frame_rejection_reason or (None if copied_features else "waiting_for_one_pen"),
    }


class PenFeatureJoiner:
    def __init__(self, capacity: int = 8, max_age_ns: int = 500_000_000) -> None:
        self._pairs = ExactStampPairCache(capacity, max_age_ns)
    def add_detection(self, payload: dict[str, Any], now_ns: int) -> tuple[dict[str, Any], RectifiedImage] | None:
        stamp = detection_stamp_ns(payload); pair = self._pairs.add_left(stamp, payload, now_ns)
        return None if pair is None else (pair[1], pair[2])
    def add_image(self, image: RectifiedImage, now_ns: int) -> tuple[dict[str, Any], RectifiedImage] | None:
        pair = self._pairs.add_right(image.stamp_ns, image, now_ns)
        return None if pair is None else (pair[1], pair[2])
    def expire(self, now_ns: int) -> None: self._pairs.expire(now_ns)
