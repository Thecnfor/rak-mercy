#!/usr/bin/env python3
"""650 mm venue release sequence; requires validated transport profile."""
from __future__ import annotations
import argparse, json, math, os, time
from pathlib import Path
from deyes_stereo.competition_pick_execution import Mercury650Executor, MotionProfile, motion_profile_from_venue

DEFAULT_PROFILE=Path(os.environ.get("DEGRADED_VENUE_PROFILE","/home/elephant/deyes_competition_assets/competition_venue_65cm.yaml"))


def _stable_gripper_feedback(arm, *, samples: int = 3, interval_sec: float = .1,
                             max_spread: float = 2.0) -> float:
    getter=getattr(arm,"get_gripper_value",None)
    if not callable(getter): raise RuntimeError("gripper_feedback_unavailable")
    values=[]
    for _ in range(samples):
        value=getter()
        try: number=float(value)
        except (TypeError,ValueError) as exc: raise RuntimeError(f"gripper_feedback_invalid:{value}") from exc
        if not math.isfinite(number): raise RuntimeError(f"gripper_feedback_invalid:{value}")
        values.append(number); time.sleep(interval_sec)
    if max(values)-min(values)>max_spread: raise RuntimeError(f"gripper_feedback_unstable:{values}")
    return sum(values)/len(values)


def _motion_profile(path: Path) -> MotionProfile:
    return motion_profile_from_venue(path)


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
    commands_emitted=False; trace=[]; release_feedback=None
    try:
        profile.require_hardware_admission()
        from pymycobot import Mercury
        arm=Mercury(os.environ.get("DEGRADED_ARM_PORT","/dev/right_arm"),115200)
        if arm.is_power_on()!=1:
            commands_emitted=True
            arm.power_on(); time.sleep(1.5)
            if arm.is_power_on()!=1: raise RuntimeError("power_on_feedback_failed")
        before_release=_stable_gripper_feedback(arm)

        def verify_release_feedback(robot) -> float:
            nonlocal release_feedback
            release_feedback=_stable_gripper_feedback(robot)
            if release_feedback-before_release<5.0:
                raise RuntimeError(
                    f"gripper_open_feedback_delta_below_5:{release_feedback-before_release}"
                )
            return release_feedback

        commands_emitted=True
        trace=Mercury650Executor(
            arm,profile,release_feedback_check=verify_release_feedback,trace_sink=trace
        ).place(args.x_mm,args.y_mm)
        result={"schema":"competition_place_execution/v1","success":True,
                "motion_completed":True,"object_state":args.object_state,
                "object_delivery_verified":args.object_state=="verified",
                "showcase_mode":args.showcase_mode,"commands_emitted":True,
                "release_feedback":release_feedback,"trace":trace}
    except (ImportError,RuntimeError,OSError,TypeError,ValueError,AttributeError) as exc:
        result={"schema":"competition_place_execution/v1","success":False,
                "motion_completed":False,"object_state":args.object_state,
                "object_delivery_verified":False,"showcase_mode":args.showcase_mode,
                "commands_emitted":commands_emitted,"reason":str(exc),"trace":trace}
    if args.result_json:
        args.result_json.parent.mkdir(parents=True,exist_ok=True)
        args.result_json.write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(result)); return 0 if result["success"] else 2


if __name__ == "__main__": raise SystemExit(main())
