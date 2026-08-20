"""ROS-free contracts shared by every YOLO detector backend."""

from __future__ import annotations

import json
from typing import Any, Iterable, Mapping


def parse_allowed_class_ids_json(text: str) -> tuple[frozenset[int], str | None]:
    """Parse the detector class allowlist without silently changing its meaning.

    The parameter is intentionally a JSON array rather than a ROS integer-array
    parameter so launch files and deployed YAML retain one portable representation.
    An empty array means that no class filter is applied.
    """
    if not isinstance(text, str):
        return frozenset(), "allowed_class_ids_json must be a JSON array string"
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as exc:
        return frozenset(), f"allowed_class_ids_json is invalid JSON: {exc.msg}"
    if not isinstance(payload, list):
        return frozenset(), "allowed_class_ids_json must be a JSON array"

    class_ids: list[int] = []
    for index, value in enumerate(payload):
        if isinstance(value, bool) or not isinstance(value, int):
            return frozenset(), (
                f"allowed_class_ids_json[{index}] must be a non-negative integer"
            )
        if value < 0:
            return frozenset(), (
                f"allowed_class_ids_json[{index}] must be a non-negative integer"
            )
        if value in class_ids:
            return frozenset(), f"allowed_class_ids_json contains duplicate class id {value}"
        class_ids.append(value)
    return frozenset(class_ids), None


def filter_detections_by_allowed_class_ids(
    detections: Iterable[Mapping[str, Any]], allowed_class_ids: frozenset[int]
) -> list[dict[str, Any]]:
    """Apply the one backend-independent detector-output class filter.

    ``allowed_class_ids == frozenset()`` is deliberately pass-through.  A malformed
    detection never becomes a valid detection when filtering is enabled.
    """
    copied = [dict(detection) for detection in detections]
    if not allowed_class_ids:
        return copied
    return [
        detection
        for detection in copied
        if isinstance(detection.get("class_id"), int)
        and not isinstance(detection.get("class_id"), bool)
        and int(detection["class_id"]) in allowed_class_ids
    ]
