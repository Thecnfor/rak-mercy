#!/usr/bin/env python3
"""Generate a non-destructive USD override for the 650 mm competition scene."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping


SOURCE_HEIGHT_M = 0.80
TARGET_HEIGHT_M = 0.65
PEN_CENTER_ABOVE_TABLE_M = 0.008


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _table_block(table_id: str, thickness_m: float, bbox: list[float]) -> str:
    x0, y0, x1, y1 = bbox
    width, depth = x1 - x0, y1 - y0
    center_x, center_y = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    top_center = TARGET_HEIGHT_M - thickness_m / 2.0
    leg_height = TARGET_HEIGHT_M - thickness_m
    leg_center = leg_height / 2.0
    leg_size = min(0.055, width / 5.0, depth / 5.0)
    inset = max(leg_size, 0.035)
    leg_xy = (
        (x0 + inset, y0 + inset),
        (x0 + inset, y1 - inset),
        (x1 - inset, y0 + inset),
        (x1 - inset, y1 - inset),
    )
    legs = "\n".join(
        f'''            over "Leg_{index}"
            {{
                double3 xformOp:translate = ({x:.9f}, {y:.9f}, {leg_center:.3f})
                float3 xformOp:scale = ({leg_size:.3f}, {leg_size:.3f}, {leg_height:.3f})
            }}'''
        for index, (x, y) in enumerate(leg_xy, start=1)
    )
    return f'''        over "{table_id}"
        {{
            custom string teamRak:competitionTableHeight = "650mm_scene_override"
            over "Top"
            {{
                double3 xformOp:translate = ({center_x:.9f}, {center_y:.9f}, {top_center:.3f})
            }}
{legs}
        }}'''


def build_scene_override(
    source_config: Mapping[str, Any], source_usd: Path
) -> tuple[str, dict[str, Any]]:
    """Return a USD layer and audit manifest without changing the source scene."""
    source_usd = source_usd.resolve()
    if not source_usd.is_file():
        raise ValueError(f"source_usd_missing:{source_usd}")
    tables = source_config.get("tables")
    if not isinstance(tables, list) or len(tables) != 2:
        raise ValueError("exactly_two_source_tables_required")
    try:
        heights = [float(table["height_m"]) for table in tables]
        thicknesses = [float(table["top_thickness_m"]) for table in tables]
        table_ids = [str(table["id"]) for table in tables]
        bboxes = [[float(value) for value in table["scene_bbox_xy_m"]] for table in tables]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"source_table_config_invalid:{exc}") from exc
    if any(abs(height - SOURCE_HEIGHT_M) > 1e-9 for height in heights):
        raise ValueError("expected_0.80m_source_tables")
    if any(thickness <= 0 or thickness >= TARGET_HEIGHT_M for thickness in thicknesses):
        raise ValueError("source_table_thickness_invalid")
    pens = source_config.get("pens", {})
    fixed_poses = pens.get("fixed_poses") if isinstance(pens, Mapping) else None
    if not isinstance(fixed_poses, list) or len(fixed_poses) < 1:
        raise ValueError("source_pen_inventory_missing")
    pen_prefix = str(pens.get("source_table_id", table_ids[0]))
    pen_names = [f"{pen_prefix}_pen_{index}" for index in range(1, len(fixed_poses) + 1)]

    table_blocks = "\n".join(
        _table_block(table_id, thickness, bbox)
        for table_id, thickness, bbox in zip(table_ids, thicknesses, bboxes)
    )
    hidden_pens = "\n".join(
        f'''        over "{pen_name}" (
            active = false
        )
        {{
        }}'''
        for pen_name in pen_names[1:]
    )
    pen_center_z = TARGET_HEIGHT_M + PEN_CENTER_ABOVE_TABLE_M
    retained_xy = fixed_poses[0].get("center_xy_m")
    if not isinstance(retained_xy, list) or len(retained_xy) != 2:
        raise ValueError("retained_pen_center_xy_missing")
    pen_x, pen_y = (float(value) for value in retained_xy)
    layer = f'''#usda 1.0
(
    subLayers = [@{source_usd}@]
    customLayerData = {{
        string sourceSceneId = "{source_config.get('scene_id', 'unknown')}"
        string adaptation = "650mm_table_exactly_one_pen"
        string provenance = "simulation_override_only_not_physical_measurement"
    }}
)

over "World"
{{
    over "PhysicsScene"
    {{
        uint physxScene:timeStepsPerSecond = 60
    }}
    over "Tables"
    {{
{table_blocks}
    }}
    over "Pens"
    {{
        over "{pen_names[0]}"
        {{
            double3 xformOp:translate = ({pen_x:.9f}, {pen_y:.9f}, {pen_center_z:.3f})
            custom string teamRak:competitionPenSelection = "single_pen_fixture"
        }}
{hidden_pens}
    }}
    over "Robot"
    {{
        over "mercury_x1"
        {{
            over "Graph"
            {{
                over "DiffController"
                {{
                    over "differential_controller"
                    {{
                        custom double inputs:dt = 0.016666666666666666
                        double inputs:dt.connect = None
                    }}
                }}
            }}
        }}
    }}
}}
'''
    manifest = {
        "schema": "competition_65cm_scene_override/v1",
        "source_scene_id": str(source_config.get("scene_id", "unknown")),
        "source_usd": str(source_usd),
        "source_usd_sha256": _sha256(source_usd),
        "source_table_height_m": SOURCE_HEIGHT_M,
        "override_table_height_m": TARGET_HEIGHT_M,
        "height_delta_m": round(TARGET_HEIGHT_M - SOURCE_HEIGHT_M, 9),
        "source_pen_count": len(pen_names),
        "override_pen_count": 1,
        "retained_pen_prim": f"/World/Pens/{pen_names[0]}",
        "deactivated_pen_prims": [f"/World/Pens/{name}" for name in pen_names[1:]],
        "physical_parameters_modified": False,
        "claims": {
            "simulation_only": True,
            "visual_measurement": False,
            "physical_calibration": False,
        },
    }
    return layer, manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-config", type=Path, required=True)
    parser.add_argument("--source-usd", type=Path, required=True)
    parser.add_argument("--output-usda", type=Path, required=True)
    parser.add_argument("--output-manifest", type=Path, required=True)
    args = parser.parse_args()
    config = json.loads(args.source_config.read_text(encoding="utf-8"))
    layer, manifest = build_scene_override(config, args.source_usd)
    args.output_usda.parent.mkdir(parents=True, exist_ok=True)
    args.output_manifest.parent.mkdir(parents=True, exist_ok=True)
    args.output_usda.write_text(layer, encoding="utf-8")
    args.output_manifest.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
