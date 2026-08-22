"""Small, ROS-free primitives for collecting an auditable detection dataset.

The ROS node deliberately keeps policy in this module so that path safety,
manifest records and capture cadence can be tested on a development PC.
"""

from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping
from uuid import uuid4


class DatasetCaptureError(ValueError):
    """Raised when a collection request could place evidence in an unsafe path."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def require_external_absolute_dir(output_dir: str | Path, repository_root: str | Path) -> Path:
    """Return a resolved external output path, rejecting the repository itself.

    Dataset images are intentionally never stored in Git.  Resolving both paths
    also rejects a symlink that points back into the source tree.
    """
    candidate = Path(output_dir).expanduser()
    if not candidate.is_absolute():
        raise DatasetCaptureError("output_dir_must_be_an_absolute_path")
    resolved = candidate.resolve(strict=False)
    repo = Path(repository_root).resolve(strict=False)
    try:
        resolved.relative_to(repo)
    except ValueError:
        return resolved
    raise DatasetCaptureError("output_dir_must_be_outside_the_repository")


def validate_capture_options(*, min_interval_sec: float, max_images: int, jpeg_quality: int) -> None:
    if min_interval_sec < 0.0:
        raise DatasetCaptureError("min_interval_sec_must_be_non_negative")
    if max_images < 0:
        raise DatasetCaptureError("max_images_must_be_zero_or_positive")
    if not 1 <= jpeg_quality <= 100:
        raise DatasetCaptureError("jpeg_quality_must_be_between_1_and_100")


def safe_component(value: str, fallback: str = "frame") -> str:
    """Make a filesystem-safe, human-readable part of an output filename."""
    normalized = re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("_.-")
    return normalized[:80] or fallback


@dataclass(frozen=True)
class FrameEvidence:
    stamp_sec: int
    stamp_nanosec: int
    frame_id: str
    width: int
    height: int
    encoding: str
    received_at_utc: str
    sharpness_laplacian_variance: float | None = None
    mean_abs_difference_from_previous_saved: float | None = None


class DatasetSession:
    """Append-only JSONL manifest and safe final summary for one capture session."""

    def __init__(self, output_root: Path, *, configuration: Mapping[str, Any]) -> None:
        output_root.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.session_id = f"pen_{timestamp}_{uuid4().hex[:8]}"
        self.path = output_root / self.session_id
        self.path.mkdir(parents=False, exist_ok=False)
        self.images_dir = self.path / "images"
        self.images_dir.mkdir()
        self.manifest_path = self.path / "session_manifest.jsonl"
        self.summary_path = self.path / "session_summary.json"
        self._manifest = self.manifest_path.open("x", encoding="utf-8")
        self.configuration = dict(configuration)
        self.started_at_utc = utc_now()
        self.saved_images = 0
        self.received_messages = 0
        self.skipped_interval = 0
        self.skipped_decode_error = 0
        self.write_event(
            {
                "event": "session_started",
                "session_id": self.session_id,
                "timestamp_utc": self.started_at_utc,
                "configuration": self.configuration,
            }
        )

    def write_event(self, event: Mapping[str, Any]) -> None:
        self._manifest.write(json.dumps(dict(event), ensure_ascii=False, sort_keys=True) + "\n")
        self._manifest.flush()
        os.fsync(self._manifest.fileno())

    def next_image_path(self, evidence: FrameEvidence) -> Path:
        index = self.saved_images + 1
        frame = safe_component(evidence.frame_id, "no_frame")
        return self.images_dir / (
            f"pen_{index:06d}_{evidence.stamp_sec}_{evidence.stamp_nanosec:09d}_{frame}.jpg"
        )

    def record_saved(self, image_path: Path, evidence: FrameEvidence) -> None:
        self.saved_images += 1
        relative = image_path.relative_to(self.path).as_posix()
        self.write_event(
            {
                "event": "image_saved",
                "index": self.saved_images,
                "file": relative,
                "stamp": {"sec": evidence.stamp_sec, "nanosec": evidence.stamp_nanosec},
                "frame_id": evidence.frame_id,
                "width": evidence.width,
                "height": evidence.height,
                "encoding": evidence.encoding,
                "received_at_utc": evidence.received_at_utc,
                # These are evidence-quality reports only.  This collector never
                # drops a retained image based on either value.
                "sharpness_laplacian_variance": evidence.sharpness_laplacian_variance,
                "mean_abs_difference_from_previous_saved": evidence.mean_abs_difference_from_previous_saved,
            }
        )

    def close(self, *, reason: str) -> None:
        if self._manifest.closed:
            return
        ended_at = utc_now()
        summary = {
            "session_id": self.session_id,
            "started_at_utc": self.started_at_utc,
            "ended_at_utc": ended_at,
            "end_reason": reason,
            "configuration": self.configuration,
            "counts": {
                "received_messages": self.received_messages,
                "saved_images": self.saved_images,
                "skipped_min_interval": self.skipped_interval,
                "skipped_decode_error": self.skipped_decode_error,
            },
            "manifest": self.manifest_path.name,
            "images_dir": self.images_dir.name,
        }
        self.write_event({"event": "session_finished", "timestamp_utc": ended_at, "reason": reason})
        temporary = self.summary_path.with_name(".session_summary.tmp.json")
        temporary.write_text(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        temporary.replace(self.summary_path)
        self._manifest.close()

