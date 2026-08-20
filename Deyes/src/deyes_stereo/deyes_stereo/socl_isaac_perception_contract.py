"""Fail-closed, ROS-free perception contract for the Socl_ous Isaac scene.

The Isaac camera publishes a depth image and a separately stamped CameraInfo.
This adapter is deliberately simulation-only: it can make a *derived* camera
info header match the depth header for offline/RViz integration, but it never
turns the simulated data into physical calibration evidence or a motion input.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


ISAAC_SIM_SOURCE = "isaac_sim"
EXPECTED_RGB_ENCODING = "rgb8"
EXPECTED_DEPTH_ENCODING = "32FC1"


@dataclass(frozen=True)
class Header:
    stamp_ns: int
    frame_id: str


@dataclass(frozen=True)
class ImageDescriptor:
    header: Header
    width: int
    height: int
    encoding: str
    source: str = ISAAC_SIM_SOURCE


@dataclass(frozen=True)
class CameraInfoDescriptor:
    header: Header
    width: int
    height: int
    projection: tuple[float, ...]
    source: str = ISAAC_SIM_SOURCE


@dataclass(frozen=True)
class BridgeMetadata:
    source: str
    simulation_validated: bool
    physical_validated: bool
    command_consumption_allowed: bool
    physical_consumption_allowed: bool
    camera_info_original_stamp_ns: int
    camera_info_original_frame_id: str
    depth_stamp_ns: int
    depth_frame_id: str
    original_stamp_skew_ns: int
    camera_info_reheadered: bool


@dataclass(frozen=True)
class BridgeResult:
    valid: bool
    reasons: tuple[str, ...]
    camera_info_rect: CameraInfoDescriptor | None
    metadata: BridgeMetadata | None


def _valid_projection(projection: Any) -> bool:
    try:
        values = np.asarray(projection, dtype=np.float64).reshape(-1)
    except (TypeError, ValueError):
        return False
    return bool(values.size == 12 and np.all(np.isfinite(values)) and values[0] > 0.0 and values[5] > 0.0)


def validate_isaac_input(
    depth: ImageDescriptor,
    camera_info: CameraInfoDescriptor,
    *,
    received_stamp_ns: int,
    max_stamp_skew_ns: int = 50_000_000,
    max_camera_info_age_ns: int = 500_000_000,
) -> tuple[str, ...]:
    """Validate source messages before deriving a matching CameraInfo header.

    The CameraInfo source frame is intentionally not required to equal the
    depth frame: Socl_ous currently reports ``sim_camera`` while depth reports
    ``Left_camera``.  The difference is retained in bridge metadata and the
    derived output uses the depth header exactly.
    """
    reasons: list[str] = []
    if depth.source != ISAAC_SIM_SOURCE or camera_info.source != ISAAC_SIM_SOURCE:
        reasons.append("source_must_be_isaac_sim")
    if depth.encoding != EXPECTED_DEPTH_ENCODING:
        reasons.append("depth_encoding_must_be_32FC1")
    if not depth.header.frame_id:
        reasons.append("depth_frame_id_missing")
    if depth.width <= 0 or depth.height <= 0:
        reasons.append("depth_size_invalid")
    if camera_info.width <= 0 or camera_info.height <= 0:
        reasons.append("camera_info_size_invalid")
    if depth.width != camera_info.width or depth.height != camera_info.height:
        reasons.append("depth_camera_info_size_mismatch")
    if not _valid_projection(camera_info.projection):
        reasons.append("camera_info_projection_invalid")
    skew = abs(int(depth.header.stamp_ns) - int(camera_info.header.stamp_ns))
    if skew > max_stamp_skew_ns:
        reasons.append("camera_info_stamp_skew_exceeds_limit")
    age = int(received_stamp_ns) - int(camera_info.header.stamp_ns)
    if age < 0 or age > max_camera_info_age_ns:
        reasons.append("camera_info_stale")
    return tuple(reasons)


def bridge_isaac_depth_camera_info(
    depth: ImageDescriptor,
    camera_info: CameraInfoDescriptor,
    *,
    received_stamp_ns: int,
    max_stamp_skew_ns: int = 50_000_000,
    max_camera_info_age_ns: int = 500_000_000,
) -> BridgeResult:
    """Derive a depth-aligned CameraInfo only after fail-closed validation."""
    reasons = validate_isaac_input(
        depth,
        camera_info,
        received_stamp_ns=received_stamp_ns,
        max_stamp_skew_ns=max_stamp_skew_ns,
        max_camera_info_age_ns=max_camera_info_age_ns,
    )
    metadata = BridgeMetadata(
        source=ISAAC_SIM_SOURCE,
        simulation_validated=not reasons,
        physical_validated=False,
        command_consumption_allowed=False,
        physical_consumption_allowed=False,
        camera_info_original_stamp_ns=int(camera_info.header.stamp_ns),
        camera_info_original_frame_id=str(camera_info.header.frame_id),
        depth_stamp_ns=int(depth.header.stamp_ns),
        depth_frame_id=str(depth.header.frame_id),
        original_stamp_skew_ns=abs(int(depth.header.stamp_ns) - int(camera_info.header.stamp_ns)),
        camera_info_reheadered=not reasons,
    )
    if reasons:
        return BridgeResult(False, reasons, None, metadata)
    rect = CameraInfoDescriptor(
        header=Header(stamp_ns=int(depth.header.stamp_ns), frame_id=str(depth.header.frame_id)),
        width=int(depth.width),
        height=int(depth.height),
        projection=tuple(float(value) for value in camera_info.projection),
        source=ISAAC_SIM_SOURCE,
    )
    return BridgeResult(True, (), rect, metadata)


def validate_physical_or_command_consumption(metadata: BridgeMetadata | None) -> tuple[bool, tuple[str, ...]]:
    """Explicitly prohibit simulated bridge data from physical/grasp control."""
    if metadata is None:
        return False, ("simulation_bridge_metadata_missing",)
    return False, ("isaac_sim_data_cannot_be_used_for_physical_or_command_consumption",)
