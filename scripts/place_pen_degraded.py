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
        validated=(transport.get("transport_validated") is True and transport.get("joint_limits_passed") is True
                   and float(transport.get("fk_position_residual_mm",float("inf")))<=5.0
                   and float(transport.get("fk_orientation_residual_deg",float("inf")))<=2.0)
    except (OSError,KeyError,TypeError,ValueError,AttributeError): validated=False
    return MotionProfile(transport_validated=validated)


def main() -> int:
    parser=argparse.ArgumentParser(); parser.add_argument("--x-mm",type=float,default=400.0); parser.add_argument("--y-mm",type=float,default=10.0)
    parser.add_argument("--venue-profile",type=Path,default=DEFAULT_PROFILE)
    parser.add_argument("--dry-run",action="store_true"); args=parser.parse_args()
    profile=_motion_profile(args.venue_profile)
    plan={"schema":"competition_place_execution/v1","target_xy_mm":[args.x_mm,args.y_mm],"z_sequence_mm":[200,165,200,260],"transport_validated":profile.transport_validated}
    if args.dry_run: print(json.dumps(plan)); return 0
    from pymycobot import Mercury
    arm=Mercury(os.environ.get("DEGRADED_ARM_PORT","/dev/right_arm"),115200)
    if not arm.is_power_on(): arm.power_on(); time.sleep(1.5)
    try: result={"success":True,"trace":Mercury650Executor(arm,profile).place(args.x_mm,args.y_mm)}
    except RuntimeError as exc: result={"success":False,"reason":str(exc)}
    print(json.dumps(result)); return 0 if result["success"] else 2


if __name__ == "__main__": raise SystemExit(main())
