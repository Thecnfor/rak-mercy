from pathlib import Path

import pytest

from tools.distill_pen_student import elongated_box, resolve_dataset_root, yolo_box


def test_elongated_box_gate():
    assert elongated_box([10, 10, 110, 30], min_aspect=2.0, max_area_fraction=0.25,
                         image_width=200, image_height=100)
    assert not elongated_box([10, 10, 50, 50], min_aspect=2.0, max_area_fraction=0.25,
                             image_width=200, image_height=100)


def test_yolo_box_normalization():
    assert yolo_box([10, 20, 50, 60], image_width=100, image_height=100) == pytest.approx(
        (0.3, 0.4, 0.4, 0.4))


def test_relative_dataset_root_is_resolved_from_yaml(tmp_path: Path):
    data_file = tmp_path / "config" / "data.yaml"
    data_file.parent.mkdir()
    assert resolve_dataset_root(data_file, {"path": "../dataset"}) == (tmp_path / "dataset").resolve()
