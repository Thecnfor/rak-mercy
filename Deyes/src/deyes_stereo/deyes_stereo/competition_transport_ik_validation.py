"""Reproducible FK verification for the official Mercury right-arm URDF."""
from __future__ import annotations
import argparse, json, math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

JOINTS=tuple(f"joint{i}_R" for i in range(1,7))

def _rpy(values):
    r,p,y=values; cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])

def _angle_delta_deg(actual, expected):
    relative=expected.T@actual; value=max(-1.,min(1.,(float(np.trace(relative))-1.)/2.))
    return math.degrees(math.acos(value))

def validate_official_urdf_solution(urdf_path, joints_rad, *, target_mm_deg=(300,10,260,179.99,-12,0)):
    root=ET.parse(str(urdf_path)).getroot(); by_name={j.attrib["name"]:j for j in root.findall("joint")}
    if set(JOINTS)-set(by_name): return {"validated":False,"reason":"official_urdf_joint_chain_missing"}
    values=[float(v) for v in joints_rad]
    if len(values)!=6 or not all(math.isfinite(v) for v in values): return {"validated":False,"reason":"ik_solution_invalid"}
    transform=np.eye(4); limits_ok=True; limit_evidence={}
    for name,q in zip(JOINTS,values):
        joint=by_name[name]; origin=joint.find("origin"); limit=joint.find("limit")
        xyz=[float(v) for v in origin.attrib.get("xyz","0 0 0").split()]; rpy=[float(v) for v in origin.attrib.get("rpy","0 0 0").split()]
        lo,hi=float(limit.attrib["lower"]),float(limit.attrib["upper"]); inside=lo<=q<=hi; limits_ok &= inside
        limit_evidence[name]={"value_rad":q,"lower_rad":lo,"upper_rad":hi,"inside":inside}
        fixed=np.eye(4); fixed[:3,:3]=_rpy(rpy); fixed[:3,3]=xyz
        moving=np.eye(4); moving[:3,:3]=_rpy((0,0,q)); transform=transform@fixed@moving
    expected_xyz=np.asarray(target_mm_deg[:3],float)/1000.; expected_rotation=_rpy(np.radians(target_mm_deg[3:]))
    pos_mm=float(np.linalg.norm(transform[:3,3]-expected_xyz)*1000.); angle_deg=_angle_delta_deg(transform[:3,:3],expected_rotation)
    return {"schema":"competition_transport_ik_validation/v1","validated":limits_ok and pos_mm<=5.0 and angle_deg<=2.0,
            "urdf":str(Path(urdf_path)),"base_frame":"link_body","tcp_link":"link6_R","joint_names":list(JOINTS),
            "joint_limits_passed":bool(limits_ok),"joint_limit_evidence":limit_evidence,"fk_xyz_m":transform[:3,3].tolist(),
            "position_residual_mm":pos_mm,"orientation_residual_deg":angle_deg,
            "thresholds":{"position_residual_mm":5.0,"orientation_residual_deg":2.0}}

def main(argv=None):
    parser=argparse.ArgumentParser(); parser.add_argument("urdf",type=Path); parser.add_argument("--joints-rad",nargs=6,type=float,required=True)
    args=parser.parse_args(argv); result=validate_official_urdf_solution(args.urdf,args.joints_rad); print(json.dumps(result,indent=2)); return 0 if result["validated"] else 2

if __name__=="__main__": raise SystemExit(main())
