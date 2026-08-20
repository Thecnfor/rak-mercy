"""ROS-free contracts shared by every YOLO detector backend."""

from __future__ import annotations

import json
import hashlib
import re
from dataclasses import dataclass
from typing import Any, Iterable, Mapping, Sequence


_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


@dataclass(frozen=True)
class TensorRTBinding:
    """The runtime facts required to validate this node's narrow TRT ABI."""

    index: int
    name: str
    is_input: bool
    shape: tuple[int, ...]
    dtype: str


@dataclass(frozen=True)
class TensorRTEngineContract:
    """Validated static TensorRT layout used by the YOLOv5 decoder."""

    input_index: int
    output_index: int
    input_shape: tuple[int, int, int, int]
    output_shape: tuple[int, int, int]


def normalize_sha256(value: str) -> str:
    """Return a normalized digest or raise rather than silently disabling the gate."""
    normalized = value.strip().lower()
    if not _SHA256_RE.fullmatch(normalized):
        raise ValueError("expected_model_sha256_must_be_a_64_character_lower_or_upper_hex_digest")
    return normalized


def verify_model_sha256(path: str, expected_sha256: str = "") -> str:
    """Hash a model and optionally enforce its configured identity.

    An empty expected digest preserves generic detector behavior while still
    publishing the observed digest.  A configured digest is always validated;
    missing files and mismatches fail closed with stable errors.
    """
    model_path = str(path).strip()
    if not model_path:
        raise ValueError("model_path_must_not_be_empty")
    digest = hashlib.sha256()
    try:
        with open(model_path, "rb") as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(block)
    except FileNotFoundError as exc:
        raise ValueError("model_file_missing") from exc
    actual = digest.hexdigest()
    expected_text = str(expected_sha256).strip()
    if expected_text:
        expected = normalize_sha256(expected_text)
        if actual != expected:
            raise ValueError(f"model_sha256_mismatch expected={expected} actual={actual}")
    return actual


def normalize_tensorrt_dtype_name(value: Any) -> str:
    """Normalize TensorRT enum spellings (for example ``DataType.HALF``)."""
    text = str(value).strip().lower()
    aliases = {
        "float": "float32",
        "float32": "float32",
        "fp32": "float32",
        "half": "float16",
        "float16": "float16",
        "fp16": "float16",
        "int32": "int32",
        "int8": "int8",
        "bool": "bool",
    }
    for suffix, normalized in aliases.items():
        if text == suffix or text.endswith(f".{suffix}"):
            return normalized
    return text


def validate_tensorrt_yolov5_contract(
    bindings: Sequence[TensorRTBinding], *, input_width: int, input_height: int, class_count: int
) -> TensorRTEngineContract:
    """Fail closed unless an engine exactly matches the decoder's static ABI.

    Only one explicit-batch NCHW FP32 input and one FP16/FP32
    ``[1, N, 5+C]`` output are safe for the in-process decoder.  In particular,
    this rejects YOLOv8/11/26 layouts, dynamic profiles and plugin engines with
    extra bindings instead of guessing how to execute or decode them.
    """
    if input_width <= 0 or input_height <= 0:
        raise ValueError("detector_input_dimensions_must_be_positive")
    if class_count <= 0:
        raise ValueError("expected_class_count_must_be_positive")
    inputs = [binding for binding in bindings if binding.is_input]
    outputs = [binding for binding in bindings if not binding.is_input]
    if len(inputs) != 1 or len(outputs) != 1 or len(bindings) != 2:
        raise ValueError("tensorrt_engine_must_have_exactly_one_input_and_one_output_binding")

    input_binding, output_binding = inputs[0], outputs[0]
    expected_input = (1, 3, input_height, input_width)
    if input_binding.shape != expected_input:
        raise ValueError(
            f"tensorrt_input_shape_must_be_{expected_input}_got_{input_binding.shape}"
        )
    if input_binding.dtype != "float32":
        raise ValueError("tensorrt_input_dtype_must_be_float32")
    if len(output_binding.shape) != 3 or output_binding.shape[0] != 1 or output_binding.shape[1] <= 0:
        raise ValueError("tensorrt_output_shape_must_be_static_[1,N,5+C]")
    expected_channels = 5 + class_count
    if output_binding.shape[2] != expected_channels:
        raise ValueError(
            f"tensorrt_output_shape_must_end_in_{expected_channels}_for_{class_count}_classes"
        )
    if output_binding.dtype not in {"float16", "float32"}:
        raise ValueError("tensorrt_output_dtype_must_be_float16_or_float32")
    return TensorRTEngineContract(
        input_index=input_binding.index,
        output_index=output_binding.index,
        input_shape=expected_input,
        output_shape=(
            int(output_binding.shape[0]),
            int(output_binding.shape[1]),
            int(output_binding.shape[2]),
        ),
    )


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


def _bbox_iou(first: Sequence[float], second: Sequence[float]) -> float:
    left = max(float(first[0]), float(second[0]))
    top = max(float(first[1]), float(second[1]))
    right = min(float(first[2]), float(second[2]))
    bottom = min(float(first[3]), float(second[3]))
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    first_area = max(0.0, float(first[2]) - float(first[0])) * max(0.0, float(first[3]) - float(first[1]))
    second_area = max(0.0, float(second[2]) - float(second[0])) * max(0.0, float(second[3]) - float(second[1]))
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


def gate_target_detections(
    detections: Iterable[Mapping[str, Any]], *, expected_max_targets: int, duplicate_iou: float
) -> tuple[list[dict[str, Any]], str | None]:
    """Assign deterministic target IDs, or reject an ambiguous target set.

    ``expected_max_targets=0`` disables the target-count policy for generic
    detectors.  A pen profile can set it to two.  Rejected candidates are not
    returned so the depth-fusion/grasp chain cannot accidentally consume them.
    """
    if expected_max_targets < 0:
        raise ValueError("expected_max_targets_must_be_zero_or_positive")
    if not 0.0 < duplicate_iou <= 1.0:
        raise ValueError("duplicate_iou_must_be_in_(0,1]")
    copied = [dict(detection) for detection in detections]
    for detection in copied:
        bbox = detection.get("bbox_xyxy")
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            return [], "invalid_bbox"
    ordered = sorted(
        copied,
        key=lambda detection: (
            (float(detection["bbox_xyxy"][0]) + float(detection["bbox_xyxy"][2])) * 0.5,
            (float(detection["bbox_xyxy"][1]) + float(detection["bbox_xyxy"][3])) * 0.5,
            -float(detection.get("confidence", 0.0)),
            int(detection.get("class_id", -1)),
        ),
    )
    if expected_max_targets and len(ordered) > expected_max_targets:
        return [], "ambiguous_multi_target"
    for left_index, left in enumerate(ordered):
        for right in ordered[left_index + 1 :]:
            if _bbox_iou(left["bbox_xyxy"], right["bbox_xyxy"]) >= duplicate_iou:
                return [], "severe_bbox_overlap_suspected_duplicate"
    for index, detection in enumerate(ordered):
        detection["det_index"] = index
        detection["target_id"] = f"target_{index:02d}"
    return ordered, None
