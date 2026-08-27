"""One-shot exact-stamp ROS 2 adapter for competition pick targets.

The joining, YAML loading, fixture decoding, and target construction stay
ROS-free. ``main`` is the only ROS boundary. The target topic is published at
most once; a valid result remains alive for the runner to stop explicitly,
while terminal rejection and input timeout leave with distinct exit codes.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import time
from typing import Any, Mapping

import numpy as np
import yaml

from .competition_pick_target_contract import TARGET_SCHEMA, TargetPolicy, build_competition_pick_target
from .ground_plane_contract import validate_rectified_depth_pair
from .venue_touch_projector_contract import MATRIX_DIRECTION, invert_rigid, validate_evidence_document


FIXED_TABLE_HEIGHT_M = 0.650
REFERENCE_TABLE_HEIGHT_M = 0.560
REFERENCE_PLANE_DISTANCE_M = 0.559428925
EXPECTED_65CM_PLANE_DISTANCE_M = 0.469428925
FIXED_TARGET_XY_M = (0.400, 0.010)
FIXED_TARGET_MANUAL_ACTION = (
    "DEGRADED: operator must place exactly one pen on the [400,10]mm "
    "right_arm_sdk marker"
)
FIXTURE_SCHEMA = "competition_pick_target_fixture/v1"
INPUT_KINDS = ("detection", "pen_features", "depth", "camera_info", "ground_plane")
TARGET_OUTPUT_QUEUE_DEPTH = 1  # Reliable transient-local terminal result.


def _stamp(payload: Mapping[str, Any]) -> int:
    if "stamp_ns" in payload:
        return int(payload["stamp_ns"])
    return int(payload.get("stamp_sec", 0) or 0) * 1_000_000_000 + int(
        payload.get("stamp_nanosec", 0) or 0
    )


def _reject(reason: str, stamp_ns: int = 0, **extra: Any) -> dict[str, Any]:
    return {
        "schema": TARGET_SCHEMA,
        "stamp_ns": int(stamp_ns),
        "valid": False,
        "trusted_for_venue_execution": False,
        "execution_allowed": False,
        "degraded": False,
        "commands_emitted": False,
        "reason": reason,
        **extra,
    }


def _load_yaml_mapping(path: Path, label: str) -> dict[str, Any]:
    if not path.is_file():
        raise ValueError(f"{label}_path_missing:{path}")
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"{label}_must_be_mapping")
    return document


class VenueProjector:
    """Validated ray provider backed only by venue_touch_projector/v1."""

    def __init__(self, document: Mapping[str, Any]) -> None:
        reasons = validate_evidence_document(dict(document))
        if reasons:
            raise ValueError("projector_evidence_invalid:" + ",".join(reasons))
        gates = document.get("gates")
        gates_pass = (
            isinstance(gates, Mapping)
            and bool(gates)
            and all(value is True for value in gates.values())
        )
        self.validated = True
        # A contradictory document cannot become usable merely because its top
        # level flag was edited; all recorded gates must still pass.
        self.usable = document.get("usable") is True and gates_pass
        self._camera_from_right = np.asarray(
            document[MATRIX_DIRECTION]["matrix"], dtype=np.float64
        ).reshape(4, 4)
        self._right_from_camera = invert_rigid(self._camera_from_right)

    @staticmethod
    def _projection(camera_info: Mapping[str, Any]) -> np.ndarray:
        raw = camera_info.get("p", camera_info.get("projection"))
        projection = np.asarray(raw, dtype=np.float64).reshape(-1)
        if projection.size != 12:
            raise ValueError("rectified_projection_invalid")
        matrix = projection.reshape(3, 4)
        if abs(float(matrix[0, 3])) > 1e-9 or abs(float(matrix[1, 3])) > 1e-9:
            raise ValueError("rectified_left_projection_translation_must_be_zero")
        return matrix[:, :3]

    def ray_for_pixel(
        self, u: float, v: float, camera_info: Mapping[str, Any]
    ) -> dict[str, Any]:
        if not (self.usable and self.validated):
            raise ValueError("projector_not_usable_and_validated")
        k = self._projection(camera_info)
        ray_camera = np.linalg.solve(k, np.asarray([u, v, 1.0], dtype=np.float64))
        origin_right = self._right_from_camera[:3, 3]
        direction_right = self._right_from_camera[:3, :3] @ ray_camera
        norm = float(np.linalg.norm(direction_right))
        if not math.isfinite(norm) or norm < 1e-9:
            raise ValueError("projector_ray_invalid")
        direction_right /= norm
        if abs(float(direction_right[2])) < 1e-9:
            raise ValueError("camera_ray_parallel_to_touch_plane")
        scale = (0.135 - float(origin_right[2])) / float(direction_right[2])
        if scale <= 0.0:
            raise ValueError("fixed_z_intersection_behind_camera")
        point_right = origin_right + scale * direction_right
        point_camera = (
            self._camera_from_right[:3, :3] @ point_right
            + self._camera_from_right[:3, 3]
        )
        return {
            "origin_m": origin_right.tolist(),
            "direction_unit": direction_right.tolist(),
            "predicted_camera_z_m": float(point_camera[2]),
            "usable": self.usable,
            "validated": self.validated,
        }


@dataclass(frozen=True)
class VenueRuntime:
    profile_path: Path
    projector_path: Path
    projector: VenueProjector
    touch_hull_xy_m: tuple[tuple[float, float], ...]
    reference_plane_distance_m: float
    expected_plane_distance_m: float
    bbox_edge_margin_px: int


def load_venue_runtime(
    profile_path: Path | str, projector_path: Path | str
) -> VenueRuntime:
    """Load independent venue profile/projector files and lock 65 cm truth."""
    profile_path = Path(profile_path).expanduser()
    projector_path = Path(projector_path).expanduser()
    profile = _load_yaml_mapping(profile_path, "venue_profile")
    projector_document = _load_yaml_mapping(projector_path, "projector")
    if profile.get("schema") != "competition_venue_profile/v1":
        raise ValueError("venue_profile_schema_invalid")
    if not math.isclose(
        float(profile.get("table_height_m", -1)), FIXED_TABLE_HEIGHT_M, abs_tol=1e-12
    ):
        raise ValueError("table_height_m_must_be_0.650")
    if not math.isclose(
        float(profile.get("reference_table_height_m", -1)),
        REFERENCE_TABLE_HEIGHT_M,
        abs_tol=1e-12,
    ):
        raise ValueError("reference_table_height_m_must_be_0.560")
    try:
        reference_distance = float(profile["reference_plane_distance_m"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("reference_plane_distance_m_must_be_configured") from exc
    try:
        expected_distance = float(profile["expected_plane_distance_m"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("expected_plane_distance_m_must_be_configured") from exc
    if not math.isclose(reference_distance, REFERENCE_PLANE_DISTANCE_M, abs_tol=1e-9):
        raise ValueError("reference_plane_distance_m_must_equal_0.559428925")
    computed = reference_distance - (
        FIXED_TABLE_HEIGHT_M - REFERENCE_TABLE_HEIGHT_M
    )
    if not math.isclose(expected_distance, computed, abs_tol=1e-9):
        raise ValueError("expected_plane_distance_m_must_equal_reference_minus_90mm")
    if not math.isclose(
        expected_distance, EXPECTED_65CM_PLANE_DISTANCE_M, abs_tol=1e-9
    ):
        raise ValueError("expected_plane_distance_m_must_equal_0.469428925")
    if not math.isclose(
        float(profile.get("touch_plane_z_m", -1)), 0.135, abs_tol=1e-12
    ):
        raise ValueError("touch_plane_z_m_must_be_0.135")
    if tuple(float(value) for value in profile.get("orientation_deg", [])) != (
        179.99,
        -12.0,
        0.0,
    ):
        raise ValueError("orientation_deg_must_be_179.99_minus12_0")
    fallbacks = profile.get("fallbacks")
    if not isinstance(fallbacks, Mapping):
        raise ValueError("fallbacks_missing")
    fixed_xy = fallbacks.get("fixed_xy")
    if (
        not isinstance(fixed_xy, Mapping)
        or tuple(float(v) for v in fixed_xy.get("xy_m", [])) != FIXED_TARGET_XY_M
    ):
        raise ValueError("fixed_xy_m_must_be_0.400_0.010")
    bbox = fallbacks.get("bbox_center")
    if not isinstance(bbox, Mapping):
        raise ValueError("bbox_center_policy_missing")
    hull = projector_document.get("calibration_convex_hull_xy_m")
    try:
        hull_tuple = tuple((float(point[0]), float(point[1])) for point in hull)
    except (TypeError, ValueError, IndexError) as exc:
        raise ValueError("projector_calibration_convex_hull_invalid") from exc
    if len(hull_tuple) < 3:
        raise ValueError("projector_calibration_convex_hull_invalid")
    return VenueRuntime(
        profile_path=profile_path,
        projector_path=projector_path,
        projector=VenueProjector(projector_document),
        touch_hull_xy_m=hull_tuple,
        reference_plane_distance_m=reference_distance,
        expected_plane_distance_m=expected_distance,
        bbox_edge_margin_px=int(bbox.get("edge_margin_px", 12)),
    )


@dataclass(frozen=True)
class Snapshot:
    stamp_ns: int
    detection: dict[str, Any]
    pen_features: dict[str, Any]
    depth_m: np.ndarray
    depth_encoding: str
    depth_frame_id: str
    camera_info: dict[str, Any]
    ground_plane: dict[str, Any]

    def to_fixture(self) -> dict[str, Any]:
        height, width = np.asarray(self.depth_m).shape
        return {
            "schema": FIXTURE_SCHEMA,
            "detection": self.detection,
            "pen_features": self.pen_features,
            "depth": {
                "stamp_ns": self.stamp_ns,
                "frame_id": self.depth_frame_id,
                "width": width,
                "height": height,
                "encoding": self.depth_encoding,
                "depth_m": np.asarray(self.depth_m, dtype=float).tolist(),
            },
            "camera_info": self.camera_info,
            "ground_plane": self.ground_plane,
        }


def snapshot_from_fixture(document: Mapping[str, Any]) -> Snapshot:
    if document.get("schema") != FIXTURE_SCHEMA:
        raise ValueError("fixture_schema_invalid")
    detection = document.get("detection")
    features = document.get("pen_features")
    depth = document.get("depth")
    camera = document.get("camera_info")
    plane = document.get("ground_plane")
    if not all(
        isinstance(value, Mapping)
        for value in (detection, features, depth, camera, plane)
    ):
        raise ValueError("fixture_inputs_must_be_mappings")
    array = np.asarray(depth.get("depth_m"), dtype=np.float32)
    return Snapshot(
        stamp_ns=_stamp(depth),
        detection=dict(detection),
        pen_features=dict(features),
        depth_m=array,
        depth_encoding=str(depth.get("encoding", "")),
        depth_frame_id=str(depth.get("frame_id", "")),
        camera_info=dict(camera),
        ground_plane=dict(plane),
    )


class ExactStampSnapshotJoiner:
    """Bounded five-way join; there is intentionally no approximate fallback."""

    def __init__(self, capacity: int = 8, max_age_ns: int = 500_000_000) -> None:
        self.capacity = max(1, int(capacity))
        self.max_age_ns = max(1, int(max_age_ns))
        self._pending: dict[int, dict[str, Any]] = {}
        self._received: dict[int, int] = {}
        self.latched = False

    def expire(self, now_ns: int) -> None:
        cutoff = int(now_ns) - self.max_age_ns
        for stamp in list(self._pending):
            if self._received[stamp] < cutoff:
                del self._pending[stamp]
                del self._received[stamp]

    def add(
        self, kind: str, stamp_ns: int, value: Any, now_ns: int
    ) -> Snapshot | None:
        if kind not in INPUT_KINDS:
            raise ValueError("snapshot_input_kind_invalid")
        if self.latched:
            return None
        if int(stamp_ns) <= 0:
            raise ValueError(f"{kind}_stamp_missing")
        self.expire(now_ns)
        bucket = self._pending.setdefault(int(stamp_ns), {})
        bucket[kind] = value
        # Keep the first-arrival time. A late stream must not rejuvenate stale
        # members of the same stamp bucket.
        self._received.setdefault(int(stamp_ns), int(now_ns))
        while len(self._pending) > self.capacity:
            oldest = min(self._received, key=self._received.get)
            del self._pending[oldest]
            del self._received[oldest]
        if any(kind_name not in bucket for kind_name in INPUT_KINDS):
            return None
        del self._pending[int(stamp_ns)]
        del self._received[int(stamp_ns)]
        self.latched = True
        depth_m, depth_encoding, depth_frame_id = bucket["depth"]
        return Snapshot(
            stamp_ns=int(stamp_ns),
            detection=dict(bucket["detection"]),
            pen_features=dict(bucket["pen_features"]),
            depth_m=np.asarray(depth_m),
            depth_encoding=str(depth_encoding),
            depth_frame_id=str(depth_frame_id),
            camera_info=dict(bucket["camera_info"]),
            ground_plane=dict(bucket["ground_plane"]),
        )


def _validate_snapshot(
    snapshot: Snapshot, *, allow_bbox_center: bool
) -> str | None:
    stamps = [
        snapshot.stamp_ns,
        _stamp(snapshot.detection),
        _stamp(snapshot.pen_features),
        _stamp(snapshot.camera_info),
        _stamp(snapshot.ground_plane),
    ]
    if not stamps[0] or len(set(stamps)) != 1:
        return "exact_stamp_mismatch"
    depth = np.asarray(snapshot.depth_m)
    if depth.ndim != 2:
        return "depth_image_invalid"
    height, width = depth.shape
    camera = snapshot.camera_info
    projection = camera.get("p", camera.get("projection"))
    pair = validate_rectified_depth_pair(
        depth_stamp_ns=snapshot.stamp_ns,
        depth_frame_id=snapshot.depth_frame_id,
        depth_width=width,
        depth_height=height,
        depth_encoding=snapshot.depth_encoding,
        info_stamp_ns=_stamp(camera),
        info_frame_id=str(camera.get("frame_id", "")),
        info_width=int(camera.get("width", 0) or 0),
        info_height=int(camera.get("height", 0) or 0),
        projection=projection,
    )
    if not pair.valid:
        return "rectified_depth_camera_info_contract:" + ",".join(pair.reasons)
    for label, payload in (
        ("detection", snapshot.detection),
        ("pen_features", snapshot.pen_features),
    ):
        if (
            str(payload.get("frame_id", payload.get("source_frame", "")))
            != snapshot.depth_frame_id
        ):
            return f"{label}_depth_frame_mismatch"
        if int(payload.get("image_width", 0) or 0) != width or int(
            payload.get("image_height", 0) or 0
        ) != height:
            return f"{label}_depth_size_mismatch"
    plane = snapshot.ground_plane
    if str(plane.get("camera_frame", "")) != snapshot.depth_frame_id:
        return "ground_plane_depth_frame_mismatch"
    if plane.get("coordinate_contract") != "dynamic_table_plane_camera_relative_only":
        return "ground_plane_coordinate_contract_invalid"
    boxes = snapshot.detection.get(
        "detections", snapshot.detection.get("boxes", [])
    )
    if not isinstance(boxes, list) or len(boxes) != 1:
        return "detection_count_must_be_exactly_one"
    if (
        snapshot.detection.get("complete") is not True
        or snapshot.detection.get("auto_grasp_permitted") is not True
    ):
        return "detection_not_complete_or_auto_grasp_permitted"
    features = snapshot.pen_features.get("features", [])
    if not isinstance(features, list):
        return "pen_features_must_be_list"
    if len(features) > 1:
        return "pen_feature_count_exceeds_one"
    if len(features) == 0 and not allow_bbox_center:
        return "pen_feature_count_must_be_exactly_one"
    if len(features) == 1:
        box_id = str(boxes[0].get("target_id", ""))
        feature_id = str(features[0].get("target_id", ""))
        if box_id and feature_id and box_id != feature_id:
            return "detection_pen_feature_target_id_mismatch"
    return None


def build_target_from_snapshot(
    snapshot: Snapshot,
    runtime: VenueRuntime,
    *,
    allow_bbox_center: bool,
    force_fixed_target: bool,
) -> dict[str, Any]:
    """Validate a frozen snapshot, then call the ROS-free target contract."""
    reason = _validate_snapshot(snapshot, allow_bbox_center=allow_bbox_center)
    if reason is not None:
        return _reject(reason, snapshot.stamp_ns)
    features = snapshot.pen_features
    policy_bbox = bool(allow_bbox_center)
    observed_pixel: list[float] | None = None
    if force_fixed_target:
        items = features.get("features", [])
        if not isinstance(items, list) or len(items) != 1:
            return _reject(
                "fixed_target_requires_observed_single_pen_pixel", snapshot.stamp_ns
            )
        feature = items[0]
        endpoints = feature.get("axis_endpoints_px", feature.get("endpoints_px"))
        try:
            if feature.get("axis_complete") is not True or len(endpoints) != 2:
                raise ValueError
            observed_pixel = [
                (float(endpoints[0][0]) + float(endpoints[1][0])) / 2.0,
                (float(endpoints[0][1]) + float(endpoints[1][1])) / 2.0,
            ]
        except (TypeError, ValueError, IndexError):
            return _reject(
                "fixed_target_requires_observed_single_pen_pixel", snapshot.stamp_ns
            )
        # Explicit force means the pixel is not used at all; this prevents a
        # random projection from masquerading as the fixed marker fallback.
        features = dict(features)
        features["features"] = []
        policy_bbox = False
    camera = dict(snapshot.camera_info)
    camera["stamp_ns"] = _stamp(snapshot.camera_info)
    camera["depth_stamp_ns"] = snapshot.stamp_ns
    result = build_competition_pick_target(
        detection=snapshot.detection,
        pen_features=features,
        depth_m=snapshot.depth_m,
        camera_info=camera,
        ground_plane=snapshot.ground_plane,
        projector=runtime.projector,
        touch_hull_xy_m=runtime.touch_hull_xy_m,
        policy=TargetPolicy(
            fixed_height_enabled=True,
            bbox_fallback_enabled=policy_bbox,
            fixed_xy_fallback_enabled=bool(force_fixed_target),
            fixed_xy_m=FIXED_TARGET_XY_M,
            bbox_edge_margin_px=runtime.bbox_edge_margin_px,
            reference_plane_distance_m=runtime.reference_plane_distance_m,
        ),
    )
    result["venue_profile_path"] = str(runtime.profile_path)
    result["projector_path"] = str(runtime.projector_path)
    result["expected_plane_distance_m"] = runtime.expected_plane_distance_m
    result["degraded"] = result.get("selection_source") == "fixed_xy_fallback"
    result["execution_allowed"] = result.get("valid") is True and (
        result.get("trusted_for_venue_execution") is True or result["degraded"]
    )
    if result["degraded"]:
        # Preserve the genuinely observed pixel for post-grasp ROI clearing;
        # it does not influence the forced fixed XY command.
        result["pixel_uv"] = observed_pixel
        result["degraded_mode"] = "forced_fixed_xy_marker"
        result["manual_action_required"] = FIXED_TARGET_MANUAL_ACTION
        result["operator_requirement"] = (
            "place_pen_at_right_arm_sdk_marker_[400,10]mm"
        )
        result["force_fixed_target"] = True
    return result


def build_target_from_fixture(
    fixture_path: Path | str,
    profile_path: Path | str,
    projector_path: Path | str,
    *,
    allow_bbox_center: bool = False,
    force_fixed_target: bool = False,
) -> dict[str, Any]:
    document = json.loads(Path(fixture_path).read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("fixture_must_be_object")
    return build_target_from_snapshot(
        snapshot_from_fixture(document),
        load_venue_runtime(profile_path, projector_path),
        allow_bbox_center=allow_bbox_center,
        force_fixed_target=force_fixed_target,
    )


def main(args: Any = None) -> int:
    """Run the ROS live adapter. Exit 2=timeout, 3=rejected, 4=config."""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from sensor_msgs.msg import CameraInfo, Image
        from std_msgs.msg import String
    except ImportError as exc:  # pragma: no cover - exercised on Jetson
        raise RuntimeError("competition_pick_target_node requires ROS 2") from exc

    def ros_stamp(message: Any) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )

    class CompetitionPickTargetNode(Node):
        def __init__(self) -> None:
            super().__init__("competition_pick_target_node")
            defaults = {
                "detection_topic": "/x1/detection/boxes",
                "pen_features_topic": "/x1/detection/pen_features",
                "depth_topic": "/x1/stereo/depth",
                "camera_info_topic": "/x1/stereo/left/camera_info_rect",
                "ground_plane_topic": "/x1/ground/plane",
                "output_topic": "/x1/competition/pick_target",
                "venue_profile_path": "",
                "projector_path": "",
                "queue_size": 8,
                "sync_cache_max_age_sec": 1.0,
                "input_timeout_sec": 30.0,
                "terminal_publish_grace_sec": 0.75,
                "fixed_table_height_m": 0.650,
                "allow_bbox_center": False,
                "allow_fixed_xy_fallback": False,
                "force_fixed_target": False,
            }
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            if not math.isclose(
                float(self.get_parameter("fixed_table_height_m").value),
                0.650,
                abs_tol=1e-12,
            ):
                raise ValueError("fixed_table_height_m_must_be_0.650")
            profile_path = str(
                self.get_parameter("venue_profile_path").value
            ).strip()
            projector_path = str(self.get_parameter("projector_path").value).strip()
            if not profile_path:
                raise ValueError("venue_profile_path_must_be_explicit")
            if not projector_path:
                raise ValueError(
                    "projector_path_must_be_explicit_and_independent_from_stereo_calibration"
                )
            self._runtime = load_venue_runtime(profile_path, projector_path)
            self._allow_bbox = bool(self.get_parameter("allow_bbox_center").value)
            parameter_force = bool(self.get_parameter("force_fixed_target").value)
            environment_force = os.environ.get("FORCE_FIXED_TARGET", "0")
            if environment_force not in {"0", "1"}:
                raise ValueError("FORCE_FIXED_TARGET_must_be_0_or_1")
            if parameter_force != (environment_force == "1"):
                raise ValueError("force_fixed_target_parameter_environment_mismatch")
            self._force_fixed = parameter_force
            self._joiner = ExactStampSnapshotJoiner(
                int(self.get_parameter("queue_size").value),
                int(
                    float(self.get_parameter("sync_cache_max_age_sec").value) * 1e9
                ),
            )
            self._started_ns = time.monotonic_ns()
            self._timeout_ns = int(
                float(self.get_parameter("input_timeout_sec").value) * 1e9
            )
            self._grace_ns = int(
                float(self.get_parameter("terminal_publish_grace_sec").value) * 1e9
            )
            self._published = False
            self._exit_code: int | None = None
            self._exit_after_ns: int | None = None
            self.should_exit = False
            target_output_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=TARGET_OUTPUT_QUEUE_DEPTH,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._publisher = self.create_publisher(
                String,
                str(self.get_parameter("output_topic").value),
                target_output_qos,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("detection_topic").value),
                lambda msg: self._json_input("detection", msg),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("pen_features_topic").value),
                lambda msg: self._json_input("pen_features", msg),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                str(self.get_parameter("depth_topic").value),
                self._depth_input,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                CameraInfo,
                str(self.get_parameter("camera_info_topic").value),
                self._camera_input,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("ground_plane_topic").value),
                lambda msg: self._json_input("ground_plane", msg),
                qos_profile_sensor_data,
            )
            self.create_timer(0.05, self._timer)
            self.get_logger().info(
                "competition target waiting for exact-stamp detection/features/"
                "32FC1 depth/rectified CameraInfo/ground plane; "
                f"projector_usable={self._runtime.projector.usable} "
                f"force_fixed_target={self._force_fixed}"
            )
            if self._force_fixed:
                self.get_logger().error(FIXED_TARGET_MANUAL_ACTION)

        @property
        def exit_code(self) -> int:
            return 0 if self._exit_code is None else self._exit_code

        def _terminal_error(
            self, reason: str, stamp_ns: int = 0, *, code: int = 3
        ) -> None:
            self._finish(_reject(reason, stamp_ns), exit_code=code)

        def _json_input(self, kind: str, message: Any) -> None:
            if self._published:
                return
            try:
                payload = json.loads(message.data)
                if not isinstance(payload, dict):
                    raise ValueError(f"{kind}_must_be_object")
                self._accept(kind, _stamp(payload), payload)
            except (json.JSONDecodeError, TypeError, ValueError) as exc:
                self._terminal_error(f"invalid_{kind}:{exc}")

        def _depth_input(self, message: Any) -> None:
            if self._published:
                return
            stamp_ns = ros_stamp(message)
            try:
                encoding = str(message.encoding)
                if encoding != "32FC1":
                    raise ValueError(f"depth_encoding_must_be_32FC1:{encoding}")
                row = np.frombuffer(message.data, dtype=np.float32).reshape(
                    int(message.height), int(message.step) // 4
                )
                depth = row[:, : int(message.width)].copy()
                self._accept(
                    "depth",
                    stamp_ns,
                    (depth, encoding, str(message.header.frame_id)),
                )
            except (TypeError, ValueError) as exc:
                self._terminal_error(f"invalid_depth:{exc}", stamp_ns)

        def _camera_input(self, message: Any) -> None:
            if self._published:
                return
            stamp_ns = ros_stamp(message)
            payload = {
                "stamp_ns": stamp_ns,
                "depth_stamp_ns": stamp_ns,
                "frame_id": str(message.header.frame_id),
                "width": int(message.width),
                "height": int(message.height),
                "distortion_model": str(message.distortion_model),
                "p": [float(value) for value in message.p],
            }
            self._accept("camera_info", stamp_ns, payload)

        def _accept(self, kind: str, stamp_ns: int, value: Any) -> None:
            try:
                snapshot = self._joiner.add(
                    kind, stamp_ns, value, time.monotonic_ns()
                )
            except ValueError as exc:
                self._terminal_error(str(exc), stamp_ns)
                return
            if snapshot is None:
                return
            result = build_target_from_snapshot(
                snapshot,
                self._runtime,
                allow_bbox_center=self._allow_bbox,
                force_fixed_target=self._force_fixed,
            )
            self._finish(
                result, exit_code=None if result.get("valid") is True else 3
            )

        def _finish(
            self, payload: dict[str, Any], *, exit_code: int | None
        ) -> None:
            if self._published:
                return
            message = String()
            message.data = json.dumps(
                payload, ensure_ascii=False, separators=(",", ":")
            )
            self._publisher.publish(message)
            self._published = True
            log = (
                self.get_logger().info
                if payload.get("valid") is True
                else self.get_logger().error
            )
            log("competition target terminal result: " + message.data)
            if exit_code is not None:
                self._exit_code = exit_code
                self._exit_after_ns = time.monotonic_ns() + self._grace_ns

        def _timer(self) -> None:
            now_ns = time.monotonic_ns()
            self._joiner.expire(now_ns)
            if not self._published and now_ns - self._started_ns >= self._timeout_ns:
                self._terminal_error(
                    "input_timeout_waiting_for_exact_stamp_snapshot", code=2
                )
            if self._exit_after_ns is not None and now_ns >= self._exit_after_ns:
                self.should_exit = True

    rclpy.init(args=args)
    node: CompetitionPickTargetNode | None = None
    try:
        node = CompetitionPickTargetNode()
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.1)
        return node.exit_code
    except KeyboardInterrupt:
        return 0
    except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        print(f"competition_pick_target configuration error: {exc}")
        return 4
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
