#!/usr/bin/env python3
"""650 mm venue pick. One attempt only; transport is fail-closed."""
from __future__ import annotations
import argparse, json, os, time
from pathlib import Path
from deyes_stereo.competition_grasp_verification import GraspVerifier
from deyes_stereo.competition_pick_execution import Mercury650Executor, MotionProfile

DEFAULT_PROFILE=Path(os.environ.get("DEGRADED_VENUE_PROFILE","/home/elephant/deyes_competition_assets/competition_venue_65cm.yaml"))

def _motion_profile(path: Path) -> MotionProfile:
    try:
        import yaml
        transport=yaml.safe_load(path.read_text(encoding="utf-8"))["transport"]
        validated=(transport.get("transport_validated") is True and transport.get("joint_limits_passed") is True
                   and float(transport.get("fk_position_residual_mm",float("inf")))<=5.0
                   and float(transport.get("fk_orientation_residual_deg",float("inf")))<=2.0)
    except (OSError,KeyError,TypeError,ValueError,AttributeError): validated=False
    return MotionProfile(transport_validated=validated)


def main() -> int:
    parser=argparse.ArgumentParser()
    parser.add_argument("--x-mm",type=float,default=float(os.environ.get("DEGRADED_PICK_X","400")))
    parser.add_argument("--y-mm",type=float,default=float(os.environ.get("DEGRADED_PICK_Y","10")))
    parser.add_argument("--venue-profile",type=Path,default=DEFAULT_PROFILE)
    parser.add_argument("--dry-run",action="store_true")
    parser.add_argument("--result-json",type=Path); parser.add_argument("--empty-closed-feedback",type=float)
    parser.add_argument("--gripper-feedback",type=float); parser.add_argument("--pen-height-over-table-m",type=float)
    parser.add_argument("--roi-pen-last3",default="true,true,true"); args=parser.parse_args()
    profile=_motion_profile(args.venue_profile)
    plan={"schema":"competition_pick_execution/v1","target_xy_mm":[args.x_mm,args.y_mm],
          "z_sequence_mm":[235,180,140,135,180,235],"orientation_deg":[179.99,-12,0],
          "transport_pose_mm_deg":[300,10,260,179.99,-12,0],"transport_validated":profile.transport_validated}
    if args.dry_run: print(json.dumps(plan)); return 0
    from pymycobot import Mercury
    arm=Mercury(os.environ.get("DEGRADED_ARM_PORT","/dev/right_arm"),115200)
    if not arm.is_power_on(): arm.power_on(); time.sleep(1.5)
    arm.set_gripper_mode(0)
    try:
        trace=Mercury650Executor(arm,profile).pick(args.x_mm,args.y_mm)
        if args.empty_closed_feedback is None or args.gripper_feedback is None:
            result={"success":False,"navigation_permitted":False,"reason":"grasp_verification_inputs_missing","trace":trace}
        else:
            roi=[v.strip().lower()=="true" for v in args.roi_pen_last3.split(",")]
            result=GraspVerifier(args.empty_closed_feedback).verify(pen_height_over_table_m=args.pen_height_over_table_m,
                original_roi_has_pen=roi,gripper_feedback=args.gripper_feedback); result["trace"]=trace
    except RuntimeError as exc: result={"success":False,"navigation_permitted":False,"reason":str(exc)}
    if args.result_json: args.result_json.write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(result)); return 0 if result.get("success") else 2


if __name__ == "__main__": raise SystemExit(main())
