#!/usr/bin/env python3
"""650 mm venue release sequence; requires validated transport profile."""
from __future__ import annotations
import argparse, json, os, time
from pathlib import Path
from deyes_stereo.competition_pick_execution import Mercury650Executor, MotionProfile

DEFAULT_PROFILE=Path(os.environ.get("DEGRADED_VENUE_PROFILE","/home/elephant/deyes_competition_assets/competition_venue_65cm.yaml"))
def _motion_profile(path: Path) -> MotionProfile:
    try:
        import yaml
        transport=yaml.safe_load(path.read_text(encoding="utf-8"))["transport"]
        kinematics=(transport.get("kinematics_validated") is True
                    and transport.get("joint_limits_passed") is True
                    and float(transport.get("fk_position_residual_mm",float("inf")))<=5.0
                    and float(transport.get("fk_orientation_residual_deg",float("inf")))<=2.0)
        collision=transport.get("collision_clearance_validated") is True
        clearance=float(transport.get("tcp_vertical_clearance_conservative_mm",0.0))
        validated=(transport.get("transport_validated") is True and kinematics
                   and collision and clearance>0.0)
    except (OSError,KeyError,TypeError,ValueError,AttributeError):
        validated=False; kinematics=False; collision=False; clearance=0.0
    return MotionProfile(transport_validated=validated,
        kinematics_validated=kinematics,
        collision_clearance_validated=collision,
        conservative_clearance_mm=clearance)


def main() -> int:
    parser=argparse.ArgumentParser(); parser.add_argument("--x-mm",type=float,default=400.0); parser.add_argument("--y-mm",type=float,default=10.0)
    parser.add_argument("--venue-profile",type=Path,default=DEFAULT_PROFILE)
    parser.add_argument("--object-state",choices=("verified","unverified"),default="verified")
    parser.add_argument("--showcase-mode",action="store_true")
    parser.add_argument("--dry-run",action="store_true"); parser.add_argument("--result-json",type=Path); args=parser.parse_args()
    if args.object_state=="unverified" and not args.showcase_mode:
        parser.error("--object-state unverified requires --showcase-mode")
    profile=_motion_profile(args.venue_profile)
    plan={"schema":"competition_place_execution/v1","target_xy_mm":[args.x_mm,args.y_mm],
          "z_sequence_mm":[200,165,200,260],"transport_validated":profile.transport_validated,
          "kinematics_validated":profile.kinematics_validated,
          "collision_clearance_validated":profile.collision_clearance_validated,
          "conservative_clearance_mm":profile.conservative_clearance_mm,
          "object_state":args.object_state,"showcase_mode":args.showcase_mode,
          "motion_completed":False,"object_delivery_verified":False,
          "commands_emitted":False}
    if args.dry_run: print(json.dumps(plan)); return 0
    try:
        profile.require_hardware_admission()
        from pymycobot import Mercury
        arm=Mercury(os.environ.get("DEGRADED_ARM_PORT","/dev/right_arm"),115200)
        if not arm.is_power_on(): arm.power_on(); time.sleep(1.5)
        trace=Mercury650Executor(arm,profile).place(args.x_mm,args.y_mm)
        result={"schema":"competition_place_execution/v1","success":True,
                "motion_completed":True,"object_state":args.object_state,
                "object_delivery_verified":args.object_state=="verified",
                "showcase_mode":args.showcase_mode,"commands_emitted":True,
                "trace":trace}
    except (ImportError,RuntimeError,OSError,TypeError,ValueError) as exc:
        result={"schema":"competition_place_execution/v1","success":False,
                "motion_completed":False,"object_state":args.object_state,
                "object_delivery_verified":False,"showcase_mode":args.showcase_mode,
                "commands_emitted":False,"reason":str(exc)}
    if args.result_json:
        args.result_json.parent.mkdir(parents=True,exist_ok=True)
        args.result_json.write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(result)); return 0 if result["success"] else 2


if __name__ == "__main__": raise SystemExit(main())
