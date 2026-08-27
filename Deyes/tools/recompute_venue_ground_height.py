#!/usr/bin/env python3
"""Recompute old-table plane evidence from archived rectified depth frames."""

from __future__ import annotations

import argparse
import hashlib
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from deyes_stereo.ground_plane_contract import fit_plane_ransac, project_rectified_depth_pixels


SCHEMA = "venue_ground_height_evidence/v1"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def expected_parallel_plane_distance_m(old_distance_m: float, old_height_mm: float,
                                       new_height_mm: float) -> float:
    """Parallel horizontal table raised toward a fixed camera reduces distance."""
    if not 0.0 < old_distance_m < 5.0:
        raise ValueError("old_distance_m_unit_or_range_invalid")
    delta_m = (float(new_height_mm) - float(old_height_mm)) / 1000.0
    result = float(old_distance_m) - delta_m
    if not 0.0 < result < 5.0:
        raise ValueError("expected_distance_m_unit_or_range_invalid")
    return result


def load_depth(path: Path, shape: tuple[int, int] = (360, 640)) -> np.ndarray:
    if path.suffix.lower() == ".npy":
        depth = np.load(path, allow_pickle=False)
    else:
        depth = np.fromfile(path, dtype=np.float32)
        if depth.size != shape[0] * shape[1]:
            raise ValueError(f"raw_depth_shape_mismatch:{path}")
        depth = depth.reshape(shape)
    depth = np.asarray(depth, dtype=np.float32)
    if depth.shape != shape:
        raise ValueError(f"depth_shape_mismatch:{path}:{depth.shape}")
    finite = depth[np.isfinite(depth) & (depth > 0.0)]
    if not len(finite) or float(np.median(finite)) > 5.0:
        raise ValueError(f"depth_mm_m_unit_check_failed:{path}")
    return depth


def depth_points(depth: np.ndarray, projection: np.ndarray, *, sample_step: int = 2,
                 min_depth_m: float = 0.20, max_depth_m: float = 1.50,
                 max_points: int = 6000) -> np.ndarray:
    sampled = depth[::sample_step, ::sample_step]
    valid = np.isfinite(sampled) & (sampled >= min_depth_m) & (sampled <= max_depth_m)
    rows, columns = np.nonzero(valid)
    z = sampled[valid].astype(np.float64)
    if len(z) < 3:
        raise RuntimeError("no_valid_depth_samples")
    points = project_rectified_depth_pixels(columns.astype(np.float64) * sample_step,
                                             rows.astype(np.float64) * sample_step, z, projection)
    if len(points) > max_points:
        points = points[np.linspace(0, len(points) - 1, max_points, dtype=np.int32)]
    return points


def fit_frame(path: Path, projection: np.ndarray) -> tuple[dict[str, Any], np.ndarray]:
    digest = sha256_file(path)
    depth = load_depth(path)
    points = depth_points(depth, projection)
    seed = int(digest[:8], 16)
    fit = fit_plane_ransac(points, 0.02, 120, seed=seed)
    if fit is None:
        raise RuntimeError(f"ransac_failed:{path}")
    normal = fit.normal.copy()
    if normal[2] < 0.0:
        normal = -normal
    inlier_points = points[fit.inlier_mask]
    return ({
        "path": str(path).replace("\\", "/"),
        "sha256": digest,
        "valid_depth_fraction": float(np.count_nonzero(np.isfinite(depth) & (depth >= 0.2) & (depth <= 1.5)) / depth.size),
        "independent_plane_normal_camera": normal,
        "independent_plane_center_camera_m": fit.center,
        "independent_plane_distance_camera_m": abs(float(normal @ fit.center)),
        "inlier_count": fit.inlier_count,
        "inlier_ratio": fit.inlier_ratio,
        "residual_rms_m": fit.residual_rms_m,
        "residual_p95_m": fit.residual_p95_m,
    }, inlier_points)


def build_evidence(depth_paths: list[Path], stereo_path: Path,
                   old_height_mm: float = 560.0, new_height_mm: float = 650.0) -> dict[str, Any]:
    stereo = yaml.safe_load(stereo_path.read_text(encoding="utf-8"))
    projection = np.asarray(stereo["P1"], dtype=np.float64)
    if projection.shape != (3, 4) or not np.allclose(projection[:, 3], 0.0, atol=1e-12):
        raise ValueError("P1_must_be_left_rectified_projection")
    frames: list[dict[str, Any]] = []
    inliers: list[np.ndarray] = []
    for path in depth_paths:
        frame, points = fit_frame(path, projection)
        frames.append(frame)
        inliers.append(points)
    normals = np.asarray([frame["independent_plane_normal_camera"] for frame in frames])
    reference = normals[0]
    normals = np.asarray([normal if float(normal @ reference) >= 0.0 else -normal for normal in normals])
    unified = np.median(normals, axis=0)
    unified /= np.linalg.norm(unified)
    if unified[2] < 0.0:
        unified = -unified
    distances: list[float] = []
    for frame, points in zip(frames, inliers):
        signed = points @ unified
        distance = float(np.median(signed))
        residuals = np.abs(signed - distance)
        frame["unified_plane_distance_camera_m"] = abs(distance)
        frame["unified_plane_residual_median_m"] = float(np.median(residuals))
        frame["unified_plane_residual_p95_m"] = float(np.percentile(residuals, 95))
        distances.append(abs(distance))
    median = float(np.median(distances))
    mad = float(np.median(np.abs(np.asarray(distances) - median)))
    expected = expected_parallel_plane_distance_m(median, old_height_mm, new_height_mm)

    def builtin(value: Any) -> Any:
        if isinstance(value, dict):
            return {key: builtin(item) for key, item in value.items()}
        if isinstance(value, list):
            return [builtin(item) for item in value]
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        return value

    return builtin({
        "schema": SCHEMA,
        "coordinate_contract": "camera_relative_parallel_table_height_evidence_only",
        "trusted_for_grasp": False,
        "source_frame_count": len(frames),
        "projection": {"P1": projection, "sha256": sha256_file(stereo_path)},
        "fit_parameters": {
            "depth_shape": [360, 640], "depth_units": "metres", "sample_step": 2,
            "min_depth_m": 0.20, "max_depth_m": 1.50, "max_points": 6000,
            "ransac_distance_threshold_m": 0.02, "ransac_iterations": 120,
            "seed": "first_32_bits_of_each_source_sha256",
        },
        "unified_plane_normal_camera": unified,
        "old_table": {
            "height_mm": old_height_mm,
            "plane_distances_camera_m": distances,
            "plane_distance_median_m": median,
            "plane_distance_mad_m": mad,
        },
        "new_table_expectation": {
            "height_mm": new_height_mm,
            "height_delta_mm": new_height_mm - old_height_mm,
            "expected_plane_distance_camera_m": expected,
            "relationship": "expected_distance = old_median_distance - (650mm - 560mm) / 1000",
            "assumptions": ["camera and head pose unchanged", "old and new tabletops parallel and horizontal", "650mm tabletop is 90mm closer along the table normal"],
        },
        "frames": frames,
    })


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stereo", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("depth", nargs="+", type=Path)
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)
    evidence = build_evidence(args.depth, args.stereo)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(evidence, sort_keys=False), encoding="utf-8")
    print(f"wrote {args.output}: old={evidence['old_table']['plane_distance_median_m']:.6f}m expected65cm={evidence['new_table_expectation']['expected_plane_distance_camera_m']:.6f}m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
