"""ChArUco observation helpers shared by the physical stereo collector.

The helpers intentionally reject an unavailable contrib OpenCV, wrong board
identity, partial stereo intersections (<12), and non-640x360 images before a
candidate can reach any calibration solver.
"""
from __future__ import annotations
from typing import Any
import numpy as np
from .charuco_board_generator import BOARD_SPECS, require_charuco

MIN_COMMON_IDS = 12

def detect(gray: np.ndarray, *, kind: str = "stereo") -> tuple[np.ndarray, np.ndarray]:
    import cv2
    if gray.ndim != 2 or gray.shape != (360, 640):
        raise ValueError("resolution_must_be_exactly_640x360")
    spec = BOARD_SPECS[kind]
    aruco = require_charuco(cv2)
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, spec["dictionary"]))
    board = aruco.CharucoBoard((spec["squares_x"], spec["squares_y"]), spec["square_length_mm"], spec["marker_length_mm"], dictionary)
    marker_corners, marker_ids, _ = aruco.detectMarkers(gray, dictionary)
    if marker_ids is None:
        raise ValueError("charuco_markers_not_found")
    count, corners, ids = aruco.interpolateCornersCharuco(marker_corners, marker_ids, gray, board)
    if ids is None or corners is None or int(count) < MIN_COMMON_IDS:
        raise ValueError("charuco_corners_below_12")
    return corners.reshape(-1, 2).astype(np.float32), ids.reshape(-1).astype(np.int32)

def intersect(left_corners: np.ndarray, left_ids: np.ndarray, right_corners: np.ndarray, right_ids: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    common = np.intersect1d(left_ids, right_ids)
    if common.size < MIN_COMMON_IDS:
        raise ValueError("left_right_common_charuco_ids_below_12")
    left_lookup = {int(key): index for index, key in enumerate(left_ids)}
    right_lookup = {int(key): index for index, key in enumerate(right_ids)}
    return (left_corners[[left_lookup[int(key)] for key in common]], right_corners[[right_lookup[int(key)] for key in common]], common)
