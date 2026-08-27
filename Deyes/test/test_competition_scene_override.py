from pathlib import Path

from generate_competition_65cm_scene_override import build_scene_override


def test_override_lowers_both_tables_and_keeps_exactly_one_pen(tmp_path: Path):
    source_usd = tmp_path / "source.usd"
    source_usd.write_bytes(b"source-scene")
    source_config = {
        "scene_id": "v5",
        "tables": [
            {"id": "table_1", "height_m": 0.80, "top_thickness_m": 0.05, "scene_bbox_xy_m": [3.4, 0.45, 3.84, 1.05]},
            {"id": "table_2", "height_m": 0.80, "top_thickness_m": 0.05, "scene_bbox_xy_m": [2.55, 0.0, 3.19, 0.28]},
        ],
        "pens": {"source_table_id": "table_1", "fixed_poses": [
            {"center_xy_m": [3.49, 0.84]}, {}, {}, {}
        ]},
    }

    layer, manifest = build_scene_override(source_config, source_usd)

    assert manifest["source_scene_id"] == "v5"
    assert manifest["source_table_height_m"] == 0.80
    assert manifest["override_table_height_m"] == 0.65
    assert manifest["height_delta_m"] == -0.15
    assert manifest["source_pen_count"] == 4
    assert manifest["override_pen_count"] == 1
    assert manifest["physical_parameters_modified"] is False
    assert 'over "table_1"' in layer and 'over "table_2"' in layer
    assert layer.count("double3 xformOp:translate") == 11
    assert layer.count("active = false") == 3
    assert "0.625" in layer
    assert "0.658" in layer
    assert "physxScene:timeStepsPerSecond = 60" in layer
    assert "inputs:dt.connect = None" in layer
    assert "inputs:dt = 0.016666666666666666" in layer
    assert str(source_usd) in layer


def test_override_rejects_non_80cm_or_missing_scene_assets(tmp_path: Path):
    source_usd = tmp_path / "missing.usd"
    config = {
        "scene_id": "bad",
        "tables": [
            {"id": "table_1", "height_m": 0.65, "top_thickness_m": 0.05, "scene_bbox_xy_m": [0, 0, 1, 1]},
            {"id": "table_2", "height_m": 0.65, "top_thickness_m": 0.05, "scene_bbox_xy_m": [1, 0, 2, 1]},
        ],
        "pens": {"fixed_poses": [{}]},
    }
    try:
        build_scene_override(config, source_usd)
    except ValueError as exc:
        assert "source_usd_missing" in str(exc)
    else:
        raise AssertionError("missing source USD must fail closed")

    source_usd.write_bytes(b"source-scene")
    try:
        build_scene_override(config, source_usd)
    except ValueError as exc:
        assert "expected_0.80m_source_tables" in str(exc)
    else:
        raise AssertionError("unexpected source dimensions must fail closed")
