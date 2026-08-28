#!/usr/bin/env python3
"""Generate the hash-bound six-axis Mercury X1 clearance plan."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

from deyes_ik_server.ikpy_solver import IkpySolver7DOF


ORIENTATION_DEG = (179.99, -12.0, 0.0)
MAX_JOINT_STEP_RAD = math.radians(0.25)
MAX_TCP_CHORD_M = 0.001
TRANSPORT_SEED_RAD = (
    0.9039235121922143, 1.0947778286517673, -0.5541657632173106,
    -1.4630624027485444, 0.5699777942163569, 1.297798402557579,
)
VENDOR_OBSERVATION_RAD = tuple(math.radians(value) for value in (
    4.479, 94.999, 4.5, -84.29, 76.824, 92.6,
))


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _steps(previous_joints, joints, previous_xyz, xyz) -> int:
    joint_steps = max(abs(a-b) for a,b in zip(previous_joints,joints))/MAX_JOINT_STEP_RAD
    chord_steps = math.dist(previous_xyz,xyz)/MAX_TCP_CHORD_M if previous_xyz else 1.0
    return max(1, math.ceil(max(joint_steps, chord_steps)))


def build_plan(urdf: Path, profile: Path, scene_usd: Path) -> dict:
    solver = IkpySolver7DOF("right", str(urdf))
    seed_deg = [math.degrees(value) for value in TRANSPORT_SEED_RAD]
    targets = [
        ("before_pick", (0.400, 0.010, 0.235)),
        ("pregrasp", (0.400, 0.010, 0.180)),
        ("approach", (0.400, 0.010, 0.140)),
        ("contact", (0.400, 0.010, 0.135)),
        ("lift", (0.400, 0.010, 0.180)),
        ("lift", (0.400, 0.010, 0.235)),
        ("transport", (0.300, 0.010, 0.260)),
        ("place_pre", (0.400, 0.010, 0.200)),
        ("release_approach", (0.400, 0.010, 0.165)),
        ("retreat", (0.400, 0.010, 0.200)),
        ("retreat", (0.400, 0.010, 0.260)),
    ]
    solved=[]
    previous_joints=VENDOR_OBSERVATION_RAD
    previous_xyz=None
    for phase,xyz in targets:
        pose=[*xyz,*ORIENTATION_DEG]
        result=solver.solve(pose,seed_deg,max_iter=500,tol_m=.005)
        if not result.success:
            raise RuntimeError(f"ik_failed:{phase}:{result.failure_code}")
        joints=tuple(math.radians(value) for value in result.joint_deg)
        solved.append({"phase":phase,"tcp_pose_m_deg":pose,"right_arm_rad":list(joints),
            "fk_position_residual_mm":result.residual_m*1000.0,
            "fk_orientation_residual_deg":result.orientation_residual_deg,
            "interpolation_steps":_steps(previous_joints,joints,previous_xyz,xyz)})
        previous_joints=joints; previous_xyz=xyz; seed_deg=result.joint_deg
    open_gripper=[math.radians(40),-math.radians(40),-math.radians(40),math.radians(40)]
    steps=[{"phase":"before_pick","right_gripper_rad":open_gripper},*solved[:4],
        {"phase":"close","right_gripper_rad":[0.0]*4},*solved[4:9],
        {"phase":"release","right_gripper_rad":open_gripper},*solved[9:]]
    transport_stow=next(item["right_arm_rad"] for item in solved if item["phase"]=="transport")
    return {
        "schema":"isaac_clearance_joint_plan/v1",
        "urdf_sha256":_sha(urdf),"motion_profile_sha256":_sha(profile),
        "scene_usd_sha256":_sha(scene_usd),"ik_fk_passed":True,
        "max_joint_increment_deg":0.25,"max_tcp_chord_mm":1.0,
        "power_on_left_rad":None,"power_on_right_rad":None,
        "selected_order":"left_then_right",
        "stow_left_rad":list(VENDOR_OBSERVATION_RAD),
        "stow_right_rad":list(transport_stow),
        "steps":steps,
    }


def main() -> int:
    parser=argparse.ArgumentParser()
    parser.add_argument("--urdf",type=Path,required=True)
    parser.add_argument("--profile",type=Path,required=True)
    parser.add_argument("--scene-usd",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    plan=build_plan(args.urdf,args.profile,args.scene_usd)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(plan,indent=2,sort_keys=True)+"\n",encoding="utf-8")
    print(json.dumps({"output":str(args.output),"steps":len(plan["steps"]),"ik_fk_passed":True}))
    return 0


if __name__=="__main__": raise SystemExit(main())
