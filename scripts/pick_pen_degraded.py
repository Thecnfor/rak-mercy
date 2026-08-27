#!/usr/bin/env python3
"""650 mm venue pick. One attempt only; transport and verification fail closed."""
from __future__ import annotations
import argparse, json, math, os, subprocess, sys, time
from pathlib import Path
from deyes_stereo.competition_grasp_verification import GraspVerifier
from deyes_stereo.competition_pick_execution import Mercury650Executor, MotionProfile
from deyes_stereo.competition_showcase_contract import validate_showcase_target

DEFAULT_PROFILE=Path(os.environ.get("DEGRADED_VENUE_PROFILE","/home/elephant/deyes_competition_assets/competition_venue_65cm.yaml"))
DEFAULT_FEEDBACK_ADAPTER=Path(os.environ.get("COMPETITION_GRASP_FEEDBACK_ADAPTER","/home/elephant/scripts/competition_grasp_feedback_adapter.py"))

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


def _capture_feedback(adapter: Path, target_json: Path, output: Path, *, empty_closed: float,
                      gripper: float, timeout_sec: float) -> dict:
    command=[sys.executable,str(adapter),"--target-json",str(target_json),"--output",str(output),
             "--timeout",str(timeout_sec),"--empty-closed-feedback",str(empty_closed),
             "--gripper-feedback",str(gripper)]
    try:
        completed=subprocess.run(command,text=True,capture_output=True,check=False,timeout=timeout_sec+3.)
    except subprocess.TimeoutExpired as exc: raise RuntimeError("grasp_feedback_adapter_timeout") from exc
    if completed.returncode:
        detail=(completed.stderr or completed.stdout).strip()
        raise RuntimeError(f"grasp_feedback_adapter_failed:{detail}")
    try: data=json.loads(output.read_text(encoding="utf-8"))
    except (OSError,json.JSONDecodeError) as exc: raise RuntimeError("grasp_feedback_adapter_output_invalid") from exc
    if data.get("schema")!="competition_grasp_feedback/v1" or data.get("live") is not True:
        raise RuntimeError("grasp_feedback_adapter_did_not_return_live_evidence")
    if abs(float(data.get("empty_closed_feedback"))-empty_closed)>1e-6 or abs(float(data.get("gripper_feedback"))-gripper)>1e-6:
        raise RuntimeError("grasp_feedback_adapter_hardware_values_mismatch")
    roi=data.get("roi_pen_last3")
    if roi != [False,False,False]: raise RuntimeError("grasp_feedback_original_roi_not_clear_three_frames")
    if data.get("detector_frames_last3_ambiguous") != [False,False,False]:
        raise RuntimeError("grasp_feedback_detector_frames_not_explicitly_nonambiguous")
    return data


def _require_target_xy_binding(document: dict, x_mm: float, y_mm: float) -> None:
    sdk=document.get("right_arm_sdk_target_m")
    if not isinstance(sdk,list) or len(sdk)<2:
        raise RuntimeError("target_xy_missing")
    try: expected=(float(sdk[0])*1000.0,float(sdk[1])*1000.0)
    except (TypeError,ValueError) as exc: raise RuntimeError("target_xy_invalid") from exc
    if not all(math.isfinite(value) for value in (*expected,x_mm,y_mm)):
        raise RuntimeError("target_xy_invalid")
    if not (math.isclose(x_mm,expected[0],abs_tol=1e-6)
            and math.isclose(y_mm,expected[1],abs_tol=1e-6)):
        raise RuntimeError(f"target_xy_argument_mismatch:expected={expected},actual={(x_mm,y_mm)}")


def main() -> int:
    parser=argparse.ArgumentParser()
    parser.add_argument("--x-mm",type=float,default=float(os.environ.get("DEGRADED_PICK_X","400")))
    parser.add_argument("--y-mm",type=float,default=float(os.environ.get("DEGRADED_PICK_Y","10")))
    parser.add_argument("--venue-profile",type=Path,default=DEFAULT_PROFILE)
    parser.add_argument("--dry-run",action="store_true")
    parser.add_argument("--result-json",type=Path)
    target_group=parser.add_mutually_exclusive_group()
    target_group.add_argument("--target-json",type=Path)
    target_group.add_argument("--showcase-target-json",type=Path)
    parser.add_argument("--feedback-adapter",type=Path,default=DEFAULT_FEEDBACK_ADAPTER)
    parser.add_argument("--feedback-json",type=Path)
    parser.add_argument("--feedback-timeout-sec",type=float,default=8.0)
    args=parser.parse_args()
    profile=_motion_profile(args.venue_profile)
    plan={"schema":"competition_pick_execution/v1","target_xy_mm":[args.x_mm,args.y_mm],
          "z_sequence_mm":[235,180,140,135,180,235],"orientation_deg":[179.99,-12,0],
          "transport_pose_mm_deg":[300,10,260,179.99,-12,0],
          "transport_validated":profile.transport_validated,
          "kinematics_validated":profile.kinematics_validated,
          "collision_clearance_validated":profile.collision_clearance_validated,
          "conservative_clearance_mm":profile.conservative_clearance_mm,
          "commands_emitted":False}
    if args.dry_run: print(json.dumps(plan)); return 0
    trace=[]; motion_completed=False; transport_pose_reached=False
    hardware_ok=False; commands_emitted=False
    try:
        if args.result_json is None or (args.target_json is None and args.showcase_target_json is None):
            raise RuntimeError("live_pick_requires_result_and_target_json_paths")
        showcase_target=None; target_document=None
        if args.showcase_target_json is not None:
            showcase_target=validate_showcase_target(json.loads(
                args.showcase_target_json.read_text(encoding="utf-8")
            ))
            target_document=showcase_target
        elif args.feedback_json is None:
            raise RuntimeError("verified_live_pick_requires_feedback_json_path")
        elif not args.feedback_adapter.is_file():
            raise RuntimeError(f"grasp_feedback_adapter_missing:{args.feedback_adapter}")
        else:
            target_document=json.loads(args.target_json.read_text(encoding="utf-8"))
            if target_document.get("schema")!="competition_pick_target/v1":
                raise RuntimeError("competition_target_schema_mismatch")
        _require_target_xy_binding(target_document,args.x_mm,args.y_mm)
        profile.require_hardware_admission()
        from pymycobot import Mercury
        arm=Mercury(os.environ.get("DEGRADED_ARM_PORT","/dev/right_arm"),115200)
        if arm.is_power_on()!=1:
            commands_emitted=True
            arm.power_on(); time.sleep(1.5)
            if arm.is_power_on()!=1: raise RuntimeError("power_on_feedback_failed")
        commands_emitted=True
        arm.set_gripper_mode(0)
        arm.set_gripper_value(profile.gripper_closed,20); time.sleep(.5)
        empty_closed=_stable_gripper_feedback(arm)
        trace=Mercury650Executor(arm,profile,trace_sink=trace).pick(args.x_mm,args.y_mm)
        motion_completed=True
        transport_pose_reached=bool(trace and trace[-1].get("phase")=="transport")
        gripper_feedback=_stable_gripper_feedback(arm)
        hardware_ok=True
        if showcase_target is not None:
            result={"schema":"competition_grasp_verification/v1","success":False,
                    "navigation_permitted":False,
                    "reason":"showcase_target_has_no_sensor_verification",
                    "verification_failure_class":"perception"}
        else:
            try:
                evidence=_capture_feedback(args.feedback_adapter,args.target_json,args.feedback_json,
                                           empty_closed=empty_closed,gripper=gripper_feedback,
                                           timeout_sec=args.feedback_timeout_sec)
                result=GraspVerifier(empty_closed).verify(pen_height_over_table_m=None,
                    original_roi_has_pen=evidence["roi_pen_last3"],gripper_feedback=gripper_feedback)
                result["feedback_evidence"]=evidence
                result["verification_failure_class"]=(None if result.get("success")
                                                       else "object_absent")
            except (RuntimeError,OSError,TypeError,ValueError) as exc:
                result={"schema":"competition_grasp_verification/v1","success":False,
                        "navigation_permitted":False,"reason":str(exc),
                        "verification_failure_class":"perception"}
        result.update({"trace":trace,"motion_completed":motion_completed,
                       "transport_pose_reached":transport_pose_reached,
                       "hardware_ok":hardware_ok,
                       "object_grasp_verified":result.get("success") is True,
                       "commands_emitted":commands_emitted})
    except (ImportError,RuntimeError,OSError,TypeError,ValueError,AttributeError,json.JSONDecodeError) as exc:
        result={"schema":"competition_grasp_verification/v1","success":False,
                "navigation_permitted":False,"reason":str(exc),
                "verification_failure_class":"hardware",
                "motion_completed":motion_completed,
                "transport_pose_reached":transport_pose_reached,
                "hardware_ok":False,"object_grasp_verified":False,
                "commands_emitted":commands_emitted,"trace":trace}
    if args.result_json:
        args.result_json.parent.mkdir(parents=True,exist_ok=True)
        args.result_json.write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(result)); return 0 if result.get("success") else 2


if __name__ == "__main__": raise SystemExit(main())
