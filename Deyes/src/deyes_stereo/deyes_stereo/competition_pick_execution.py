"""Parameterized Mercury 650 mm table pick/place execution contract."""
from __future__ import annotations
from dataclasses import dataclass
import time
from typing import Any, Callable

ORIENTATION = (179.99, -12.0, 0.0)
TRANSPORT_POSE = (300.0, 10.0, 260.0, *ORIENTATION)


@dataclass(frozen=True)
class MotionProfile:
    high_z_mm: float = 235.0
    pregrasp_z_mm: float = 180.0
    approach_z_mm: float = 140.0
    contact_z_mm: float = 135.0
    lift_z_mm: tuple[float, float] = (180.0, 235.0)
    place_pre_z_mm: float = 200.0
    release_z_mm: float = 165.0
    retreat_z_mm: tuple[float, float] = (200.0, 260.0)
    move_speed: int = 8
    terminal_speed: int = 5
    timeout_sec: float = 8.0
    gripper_open: int = 70
    gripper_closed: int = 0
    transport_validated: bool = False


def _pose(x: float, y: float, z: float) -> tuple[float, ...]: return (x, y, z, *ORIENTATION)


def _status_ok(robot: Any) -> None:
    value = robot.get_robot_status()
    if not isinstance(value, list) or any(int(v) != 0 for v in value):
        raise RuntimeError(f"robot_status_not_ok:{value}")


def wait_pose(robot: Any, expected: tuple[float, ...], *, timeout: float = 8.0,
              clock: Callable[[], float] = time.monotonic,
              sleeper: Callable[[float], None] = time.sleep) -> list[float]:
    deadline = clock() + timeout; last = None
    while clock() < deadline:
        last = robot.get_base_coords(); _status_ok(robot)
        if last and len(last) >= 6:
            xyz = max(abs(float(last[i])-expected[i]) for i in range(3))
            rpy = max(abs(float(last[i])-expected[i]) for i in range(3, 6))
            if xyz <= 5.0 and rpy <= 2.0:
                return list(last)
        sleeper(.1)
    raise RuntimeError(f"pose_timeout expected={expected} actual={last}")


class Mercury650Executor:
    def __init__(self, robot: Any, profile: MotionProfile = MotionProfile(), *, waiter=wait_pose):
        self.robot, self.profile, self.waiter = robot, profile, waiter
        self._pick_attempted = False

    def _move(self, pose: tuple[float, ...], speed: int) -> None:
        if speed > (self.profile.terminal_speed if pose[2] in {self.profile.approach_z_mm, self.profile.contact_z_mm, self.profile.release_z_mm} else self.profile.move_speed):
            raise RuntimeError("speed_exceeds_stage_limit")
        self.robot.send_base_coords(list(pose), speed)
        self.waiter(self.robot, pose, timeout=self.profile.timeout_sec)
        _status_ok(self.robot)

    def pick(self, x_mm: float, y_mm: float) -> list[dict]:
        if self._pick_attempted: raise RuntimeError("pick_attempt_already_latched")
        self._pick_attempted = True; trace = []
        self.robot.set_gripper_value(self.profile.gripper_open, 20); _status_ok(self.robot)
        for name, z, speed in (("high", self.profile.high_z_mm, 8), ("pregrasp", self.profile.pregrasp_z_mm, 8),
                               ("approach", self.profile.approach_z_mm, 5), ("contact", self.profile.contact_z_mm, 5)):
            pose = _pose(x_mm, y_mm, z); self._move(pose, speed); trace.append({"phase": name, "pose": pose, "speed": speed})
        self.robot.set_gripper_value(self.profile.gripper_closed, 20); _status_ok(self.robot); trace.append({"phase":"close","value":0})
        for z in self.profile.lift_z_mm:
            pose = _pose(x_mm, y_mm, z); self._move(pose, 8); trace.append({"phase":"lift","pose":pose,"speed":8})
        if not self.profile.transport_validated:
            raise RuntimeError("transport_pose_not_ik_validated")
        self._move(TRANSPORT_POSE, 8); trace.append({"phase":"transport","pose":TRANSPORT_POSE,"speed":8})
        return trace

    def place(self, x_mm: float, y_mm: float) -> list[dict]:
        if not self.profile.transport_validated: raise RuntimeError("transport_pose_not_ik_validated")
        trace=[]
        for name,z,speed in (("place_pre",self.profile.place_pre_z_mm,8),("release",self.profile.release_z_mm,5)):
            pose=_pose(x_mm,y_mm,z); self._move(pose,speed); trace.append({"phase":name,"pose":pose,"speed":speed})
        self.robot.set_gripper_value(self.profile.gripper_open,20); _status_ok(self.robot); trace.append({"phase":"open","value":70})
        for z in self.profile.retreat_z_mm:
            pose=_pose(x_mm,y_mm,z); self._move(pose,8); trace.append({"phase":"retreat","pose":pose,"speed":8})
        self._move(TRANSPORT_POSE,8); trace.append({"phase":"transport","pose":TRANSPORT_POSE,"speed":8})
        return trace
