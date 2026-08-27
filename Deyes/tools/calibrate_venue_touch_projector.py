#!/usr/bin/env python3
"""Build auditable fixed-Z right-arm SDK projector evidence offline.

This tool has no ROS dependency and does not publish TF.  It consumes explicit
manual annotations of one physical jaw corner, accepted SDK touch records, and
the rectified-left P1 projection with zero distortion.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml


SCHEMA = "venue_touch_projector/v1"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _to_builtin(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _to_builtin(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_builtin(item) for item in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def convex_hull_xy(points: np.ndarray) -> np.ndarray:
    points = np.unique(np.asarray(points, dtype=np.float64), axis=0)
    if len(points) < 3:
        raise ValueError("at_least_three_unique_xy_points_required")
    ordered = sorted(map(tuple, points.tolist()))

    def cross(o: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    for point in ordered:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper: list[tuple[float, float]] = []
    for point in reversed(ordered):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    hull = np.asarray(lower[:-1] + upper[:-1], dtype=np.float64)
    if len(hull) < 3:
        raise ValueError("touch_xy_points_are_collinear")
    return hull


def matrix_from_rvec_tvec(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))[0]
    matrix[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return matrix


def _candidate(method: str, object_points_m: np.ndarray, image_points_px: np.ndarray,
               camera_matrix: np.ndarray, rvec: np.ndarray, tvec: np.ndarray) -> dict[str, Any]:
    matrix = matrix_from_rvec_tvec(rvec, tvec)
    camera_points = (matrix[:3, :3] @ object_points_m.T).T + matrix[:3, 3]
    projected = cv2.projectPoints(object_points_m, rvec, tvec, camera_matrix, np.zeros(5))[0][:, 0]
    errors = np.linalg.norm(projected - image_points_px, axis=1)
    positive = bool(np.all(camera_points[:, 2] > 0.0))
    return {
        "method": method,
        "camera_from_right_arm_sdk": matrix,
        "positive_depth": positive,
        "minimum_camera_depth_m": float(np.min(camera_points[:, 2])),
        "reprojection_errors_px": errors,
        "reprojection_rms_px": float(np.sqrt(np.mean(errors * errors))),
        "reprojection_p95_px": float(np.percentile(errors, 95)),
    }


def solve_candidates(object_points_m: np.ndarray, image_points_px: np.ndarray,
                     camera_matrix: np.ndarray) -> list[dict[str, Any]]:
    """Return all IPPE candidates plus an iterative candidate."""
    objects = np.asarray(object_points_m, dtype=np.float64)
    images = np.asarray(image_points_px, dtype=np.float64)
    if objects.shape != (len(images), 3) or len(objects) < 4:
        raise ValueError("pnp_requires_at_least_four_correspondences")
    centered = objects - objects.mean(axis=0)
    singular = np.linalg.svd(centered, compute_uv=False)
    if singular[1] < 1e-6:
        raise ValueError("touch_xy_points_are_collinear")
    if np.max(np.abs(objects[:, 2] - np.median(objects[:, 2]))) > 1e-9:
        raise ValueError("ippe_fixed_z_points_must_be_coplanar")
    candidates: list[dict[str, Any]] = []
    ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(
        objects, images, camera_matrix, np.zeros(5), flags=cv2.SOLVEPNP_IPPE
    )
    if ok:
        for index, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
            candidates.append(_candidate(f"IPPE[{index}]", objects, images, camera_matrix, rvec, tvec))
    ok, rvec, tvec = cv2.solvePnP(
        objects, images, camera_matrix, np.zeros(5), flags=cv2.SOLVEPNP_ITERATIVE
    )
    if ok:
        candidates.append(_candidate("ITERATIVE", objects, images, camera_matrix, rvec, tvec))
    if not candidates:
        raise RuntimeError("solvepnp_returned_no_candidates")
    return candidates


def select_positive_candidate(candidates: list[dict[str, Any]]) -> dict[str, Any]:
    positive = [candidate for candidate in candidates if candidate["positive_depth"]]
    if not positive:
        raise RuntimeError("no_all_positive_depth_pnp_candidate")
    return min(positive, key=lambda item: (item["reprojection_rms_px"], item["reprojection_p95_px"]))


def intersect_pixel(matrix: np.ndarray, camera_matrix: np.ndarray, pixel: np.ndarray,
                    fixed_z_m: float) -> np.ndarray:
    right_from_camera = np.linalg.inv(matrix)
    origin = right_from_camera[:3, 3]
    ray = right_from_camera[:3, :3] @ np.linalg.solve(camera_matrix, [pixel[0], pixel[1], 1.0])
    if abs(float(ray[2])) < 1e-9:
        raise RuntimeError("loo_ray_parallel_to_plane")
    scale = (fixed_z_m - origin[2]) / ray[2]
    if scale <= 0.0:
        raise RuntimeError("loo_intersection_behind_camera")
    return origin + scale * ray


def leave_one_out_errors_mm(object_points_m: np.ndarray, image_points_px: np.ndarray,
                            camera_matrix: np.ndarray, fixed_z_m: float) -> np.ndarray:
    errors: list[float] = []
    for index in range(len(object_points_m)):
        keep = np.arange(len(object_points_m)) != index
        selected = select_positive_candidate(solve_candidates(object_points_m[keep], image_points_px[keep], camera_matrix))
        prediction = intersect_pixel(selected["camera_from_right_arm_sdk"], camera_matrix,
                                     image_points_px[index], fixed_z_m)
        errors.append(float(np.linalg.norm(prediction[:2] - object_points_m[index, :2]) * 1000.0))
    return np.asarray(errors, dtype=np.float64)


def _euler_xyz_deg(rotation: np.ndarray) -> list[float]:
    sy = math.hypot(float(rotation[0, 0]), float(rotation[1, 0]))
    if sy > 1e-9:
        angles = [math.atan2(rotation[2, 1], rotation[2, 2]),
                  math.atan2(-rotation[2, 0], sy),
                  math.atan2(rotation[1, 0], rotation[0, 0])]
    else:
        angles = [math.atan2(-rotation[1, 2], rotation[1, 1]),
                  math.atan2(-rotation[2, 0], sy), 0.0]
    return [math.degrees(value) for value in angles]


def load_correspondences(annotations_path: Path, records_path: Path) -> tuple[dict[str, Any], np.ndarray, np.ndarray, list[dict[str, Any]]]:
    annotations = json.loads(annotations_path.read_text(encoding="utf-8"))
    records = json.loads(records_path.read_text(encoding="utf-8"))
    if annotations.get("schema") != "venue_touch_annotations/v1":
        raise ValueError("annotation_schema_invalid")
    if annotations.get("records_sha256") != sha256_file(records_path):
        raise ValueError("records_sha256_mismatch")
    by_index = {int(record["index"]): record for record in records}
    fixed_z_mm = float(annotations["fixed_z_plane_right_arm_sdk_mm"])
    if not 1.0 <= fixed_z_mm <= 1000.0:
        raise ValueError("fixed_z_mm_unit_or_range_invalid")
    object_points: list[list[float]] = []
    image_points: list[list[float]] = []
    used_records: list[dict[str, Any]] = []
    for item in annotations["annotations"]:
        index = int(item["record_index"])
        record = by_index.get(index)
        if record is None or record.get("accepted") is not True or record.get("touch") is None:
            raise ValueError(f"annotation_record_{index}_not_accepted")
        touch = [float(value) for value in record["touch"]]
        if not 100.0 <= touch[0] <= 1000.0 or not -1000.0 <= touch[1] <= 1000.0:
            raise ValueError(f"record_{index}_xy_mm_unit_or_range_invalid")
        object_points.append([touch[0] / 1000.0, touch[1] / 1000.0, fixed_z_mm / 1000.0])
        pixel = [float(value) for value in item["pixel_uv"]]
        if not 0.0 <= pixel[0] < 640.0 or not 0.0 <= pixel[1] < 360.0:
            raise ValueError(f"annotation_record_{index}_pixel_out_of_image")
        image_points.append(pixel)
        used_records.append(record)
    objects = np.asarray(object_points, dtype=np.float64)
    images = np.asarray(image_points, dtype=np.float64)
    if len(objects) != 6:
        raise ValueError("exactly_six_touch_annotations_required")
    convex_hull_xy(objects[:, :2])
    return annotations, objects, images, used_records


def build_evidence(annotations_path: Path, records_path: Path, stereo_path: Path) -> dict[str, Any]:
    annotations, objects, images, records = load_correspondences(annotations_path, records_path)
    stereo = yaml.safe_load(stereo_path.read_text(encoding="utf-8"))
    p1 = np.asarray(stereo["P1"], dtype=np.float64)
    if p1.shape != (3, 4) or not np.allclose(p1[:, 3], 0.0, atol=1e-12):
        raise ValueError("P1_must_be_rectified_left_projection_with_zero_translation")
    if np.any(np.asarray(stereo.get("D1", []), dtype=np.float64) != 0.0):
        raise ValueError("venue_touch_requires_zero_distortion_rectified_left_input")
    camera_matrix = p1[:, :3]
    candidates = solve_candidates(objects, images, camera_matrix)
    best = select_positive_candidate(candidates)
    matrix = best["camera_from_right_arm_sdk"]
    inverse = np.linalg.inv(matrix)
    loo = leave_one_out_errors_mm(objects, images, camera_matrix, float(objects[0, 2]))
    metrics = {
        "point_count": len(objects),
        "reprojection_errors_px": best["reprojection_errors_px"],
        "reprojection_rms_px": best["reprojection_rms_px"],
        "reprojection_p95_px": best["reprojection_p95_px"],
        "loo_base_xy_errors_mm": loo,
        "loo_base_xy_rms_mm": float(np.sqrt(np.mean(loo * loo))),
        "loo_base_xy_p95_mm": float(np.percentile(loo, 95)),
        "all_points_positive_camera_depth": best["positive_depth"],
        "minimum_camera_depth_m": best["minimum_camera_depth_m"],
    }
    gates = {
        "six_points_non_collinear": len(objects) == 6,
        "reprojection_rms_px_lte_4": metrics["reprojection_rms_px"] <= 4.0,
        "reprojection_p95_px_lte_6": metrics["reprojection_p95_px"] <= 6.0,
        "loo_base_xy_rms_mm_lte_15": metrics["loo_base_xy_rms_mm"] <= 15.0,
        "loo_base_xy_p95_mm_lte_25": metrics["loo_base_xy_p95_mm"] <= 25.0,
        "all_points_positive_camera_depth": bool(metrics["all_points_positive_camera_depth"]),
        "mm_to_m_sanity": bool(np.all(np.abs(objects[:, :2]) < 2.0) and 0.001 <= objects[0, 2] <= 1.0),
    }
    poses = np.asarray([record["touch"] for record in records], dtype=np.float64)
    hull = convex_hull_xy(objects[:, :2])
    output = {
        "schema": SCHEMA,
        "coordinate_frame": "right_arm_sdk",
        "matrix_direction": "camera_from_right_arm_sdk",
        "publishes_tf": False,
        "is_base_link_hand_eye": False,
        "warning": "Fixed-head venue projector only; never publish as TF or call it base_link hand-eye.",
        "usable": bool(all(gates.values())),
        "failed_gates": [name for name, passed in gates.items() if not passed],
        "degraded_fallback": "When unusable, do not project pixels; retain explicit fixed right_arm_sdk XY behavior.",
        "selected_candidate": best["method"],
        "camera_from_right_arm_sdk": {"units": "metres", "matrix": matrix},
        "right_arm_sdk_from_camera": {"units": "metres", "matrix": inverse},
        "camera": {
            "input": "left_rectified_640x360",
            "projection_source": str(stereo_path).replace("\\", "/"),
            "P1": p1,
            "distortion": [0.0] * 5,
        },
        "head_angles_deg": {
            "joint_11": float(stereo["head_joint_11_deg"]),
            "joint_12": float(stereo["head_joint_12_deg"]),
        },
        "fixed_tool_pose_right_arm_sdk": {
            "z_m": float(objects[0, 2]),
            "rpy_deg_median": np.median(poses[:, 3:6], axis=0),
            "observed_rpy_deg_max_range": np.ptp(poses[:, 3:6], axis=0),
        },
        "camera_from_right_arm_sdk_euler_xyz_deg": _euler_xyz_deg(matrix[:3, :3]),
        "workspace_xyz_m": [[0.35, 0.45], [-0.06, 0.16], [float(objects[0, 2]), float(objects[0, 2])]],
        "calibration_convex_hull_xy_m": hull,
        "metrics": metrics,
        "gates": gates,
        "thresholds": {
            "reprojection_rms_px_max": 4.0, "reprojection_p95_px_max": 6.0,
            "loo_base_xy_rms_mm_max": 15.0, "loo_base_xy_p95_mm_max": 25.0,
        },
        "candidates": candidates,
        "sources": {
            "annotations": {"path": str(annotations_path).replace("\\", "/"), "sha256": sha256_file(annotations_path)},
            "records": {"path": str(records_path).replace("\\", "/"), "sha256": sha256_file(records_path)},
            "stereo": {"path": str(stereo_path).replace("\\", "/"), "sha256": sha256_file(stereo_path)},
            "images": annotations["source_images"],
        },
    }
    return _to_builtin(output)


def write_overlays(annotations_path: Path, image_dir: Path, output_dir: Path) -> None:
    annotations = json.loads(annotations_path.read_text(encoding="utf-8"))
    expected_hashes = {item["image"]: item["sha256"] for item in annotations["source_images"]}
    output_dir.mkdir(parents=True, exist_ok=True)
    for item in annotations["annotations"]:
        source = image_dir / item["image"]
        if sha256_file(source) != expected_hashes.get(item["image"]):
            raise ValueError(f"source_image_sha256_mismatch:{item['image']}")
        image = cv2.imread(str(source), cv2.IMREAD_COLOR)
        if image is None:
            raise FileNotFoundError(source)
        u, v = (int(round(value)) for value in item["pixel_uv"])
        cv2.drawMarker(image, (u, v), (0, 255, 0), cv2.MARKER_CROSS, 25, 2)
        cv2.circle(image, (u, v), 7, (0, 0, 255), 2)
        cv2.putText(image, f"record {item['record_index']} upper-jaw corner ({u},{v})", (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
        if not cv2.imwrite(str(output_dir / f"overlay_{Path(item['image']).stem}.png"), image):
            raise RuntimeError("overlay_write_failed")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--annotations", type=Path, required=True)
    parser.add_argument("--records", type=Path, required=True)
    parser.add_argument("--stereo", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--image-dir", type=Path)
    parser.add_argument("--overlay-dir", type=Path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    evidence = build_evidence(args.annotations, args.records, args.stereo)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(evidence, sort_keys=False, allow_unicode=True), encoding="utf-8")
    if args.image_dir and args.overlay_dir:
        write_overlays(args.annotations, args.image_dir, args.overlay_dir)
    print(json.dumps({"output": str(args.output), "usable": evidence["usable"], "metrics": evidence["metrics"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
