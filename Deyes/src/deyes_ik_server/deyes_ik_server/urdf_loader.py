"""Resolve the Mercury X1 URDF path on the venue robot and in CI.

Order:
  1. env ``DEYES_MERCURY_URDF`` (lets ops swap a calibrated URDF without code change)
  2. Default install path: ~/mercury_x1_ros2/install/mercury_robot_urdf/.../mercury_x1.urdf
  3. Source-tree URDF: ~/mercury_x1_ros2/src/.../urdf/mercury_x1/mercury_x1.urdf
  4. Legacy ROS1 URDF: ~/mercury_x1_ros/src/.../urdf/mercury_x1/mercury_x1.urdf
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Iterable

DEFAULT_SEARCH_PATHS: tuple[str, ...] = (
    "~/mercury_x1_ros2/install/mercury_robot_urdf/share/mercury_robot_urdf/urdf/mercury_x1/mercury_x1.urdf",
    "~/mercury_x1_ros2/src/mercury_robot_urdf/mercury_robot_urdf/urdf/mercury_x1/mercury_x1.urdf",
    "~/mercury_x1_ros/src/turn_on_mercury_robot/urdf/mercury_x1/mercury_x1.urdf",
)


def resolve_urdf(extra: Iterable[str] = ()) -> str:
    """Return the first existing URDF path. Raise FileNotError if none."""
    env_override = os.environ.get("DEYES_MERCURY_URDF")
    candidates: list[str] = []
    if env_override:
        candidates.append(env_override)
    candidates.extend(DEFAULT_SEARCH_PATHS)
    candidates.extend(extra)

    checked: list[str] = []
    for raw in candidates:
        expanded = Path(os.path.expanduser(raw)).resolve()
        checked.append(str(expanded))
        if expanded.is_file():
            return str(expanded)

    raise FileNotFoundError(
        "Mercury X1 URDF not found. Searched: " + ", ".join(checked)
        + ". Set DEYES_MERCURY_URDF to override."
    )


ARM_JOINT_NAMES = {
    "right": ("joint1_R", "joint2_R", "joint3_R", "joint4_R", "joint5_R", "joint6_R", "joint7_R"),
    "left": ("joint1_L", "joint2_L", "joint3_L", "joint4_L", "joint5_L", "joint6_L", "joint7_L"),
}


def arm_joint_names(arm_side: str) -> tuple[str, ...]:
    side = (arm_side or "").strip().lower()
    if side not in ARM_JOINT_NAMES:
        raise ValueError(f"unknown arm_side {arm_side!r}; expected one of {sorted(ARM_JOINT_NAMES)}")
    return ARM_JOINT_NAMES[side]