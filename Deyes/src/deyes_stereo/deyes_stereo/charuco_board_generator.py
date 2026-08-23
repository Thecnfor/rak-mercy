"""Generate auditable ChArUco boards; output must be outside the repository."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

BOARD_SPECS = {
    "stereo": {"squares_x": 8, "squares_y": 6, "square_length_mm": 30, "marker_length_mm": 22, "dictionary": "DICT_5X5_1000"},
    "handeye": {"squares_x": 5, "squares_y": 4, "square_length_mm": 20, "marker_length_mm": 14, "dictionary": "DICT_5X5_1000"},
}


def require_charuco(cv2: Any) -> Any:
    aruco = getattr(cv2, "aruco", None)
    if aruco is None or not hasattr(aruco, "CharucoBoard"):
        raise RuntimeError("opencv_aruco_charuco_unavailable_fail_closed")
    if not hasattr(cv2, "calibrateRobotWorldHandEye"):
        raise RuntimeError("opencv_calibrateRobotWorldHandEye_unavailable_fail_closed")
    return aruco


def board_metadata(kind: str) -> dict[str, Any]:
    if kind not in BOARD_SPECS:
        raise ValueError("board_kind_must_be_stereo_or_handeye")
    spec = dict(BOARD_SPECS[kind])
    spec.update({"schema": "deyes_charuco_board/v1", "kind": kind,
                 "width_mm": spec["squares_x"] * spec["square_length_mm"],
                 "height_mm": spec["squares_y"] * spec["square_length_mm"]})
    return spec


def generate(kind: str, output_dir: Path, *, pixels_per_mm: int = 10) -> dict[str, Any]:
    import cv2
    aruco = require_charuco(cv2)
    spec = board_metadata(kind)
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, spec["dictionary"]))
    board = aruco.CharucoBoard((spec["squares_x"], spec["squares_y"]), spec["square_length_mm"], spec["marker_length_mm"], dictionary)
    image = board.generateImage((spec["width_mm"] * pixels_per_mm, spec["height_mm"] * pixels_per_mm))
    output_dir.mkdir(parents=True, exist_ok=True)
    image_path = output_dir / f"charuco_{kind}.png"
    if not cv2.imwrite(str(image_path), image):
        raise RuntimeError("board_image_write_failed")
    digest = hashlib.sha256(image_path.read_bytes()).hexdigest()
    spec.update({"file": image_path.name, "sha256": digest, "pixels_per_mm": pixels_per_mm,
                 "print_instruction": "Print at 100%; verify both dimension bars with calipers before capture."})
    (output_dir / f"charuco_{kind}.json").write_text(json.dumps(spec, indent=2) + "\n", encoding="utf-8")
    return spec


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kind", choices=sorted(BOARD_SPECS))
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args(argv)
    print(json.dumps(generate(args.kind, Path(args.output_dir).expanduser()), ensure_ascii=False))

