"""ROS-free regression tests for safe pen-dataset session evidence."""

import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "deyes_stereo"))

from deyes_stereo.dataset_capture import (  # noqa: E402
    DatasetCaptureError,
    DatasetSession,
    FrameEvidence,
    require_external_absolute_dir,
    validate_capture_options,
)


def test_output_directory_must_be_absolute_and_outside_repository(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    repo.mkdir()
    with pytest.raises(DatasetCaptureError, match="absolute"):
        require_external_absolute_dir("relative/dataset", repo)
    with pytest.raises(DatasetCaptureError, match="outside"):
        require_external_absolute_dir(repo / "images", repo)

    external = tmp_path / "evidence" / "pen"
    assert require_external_absolute_dir(external, repo) == external.resolve()


def test_capture_options_have_explicit_safe_ranges() -> None:
    with pytest.raises(DatasetCaptureError, match="non_negative"):
        validate_capture_options(min_interval_sec=-0.01, max_images=0, jpeg_quality=95)
    with pytest.raises(DatasetCaptureError, match="zero_or_positive"):
        validate_capture_options(min_interval_sec=0.0, max_images=-1, jpeg_quality=95)
    with pytest.raises(DatasetCaptureError, match="between_1_and_100"):
        validate_capture_options(min_interval_sec=0.0, max_images=1, jpeg_quality=101)


def test_session_writes_original_header_metadata_and_summary(tmp_path: Path) -> None:
    session = DatasetSession(tmp_path / "external", configuration={"min_interval_sec": 0.5})
    evidence = FrameEvidence(
        stamp_sec=123,
        stamp_nanosec=456,
        frame_id="left/camera optical",
        width=640,
        height=360,
        encoding="bgr8",
        received_at_utc="2026-08-20T00:00:00.000+00:00",
        sharpness_laplacian_variance=10.5,
        mean_abs_difference_from_previous_saved=1.25,
    )
    image_path = session.next_image_path(evidence)
    image_path.touch()
    session.received_messages = 3
    session.skipped_interval = 2
    session.record_saved(image_path, evidence)
    session.close(reason="operator_ctrl_c")

    records = [json.loads(line) for line in session.manifest_path.read_text(encoding="utf-8").splitlines()]
    saved = next(record for record in records if record["event"] == "image_saved")
    assert saved["stamp"] == {"sec": 123, "nanosec": 456}
    assert saved["frame_id"] == "left/camera optical"
    assert saved["width"] == 640 and saved["height"] == 360
    assert saved["file"].startswith("images/pen_000001_123_000000456_")

    summary = json.loads(session.summary_path.read_text(encoding="utf-8"))
    assert summary["end_reason"] == "operator_ctrl_c"
    assert summary["counts"] == {
        "received_messages": 3,
        "saved_images": 1,
        "skipped_min_interval": 2,
        "skipped_decode_error": 0,
    }
