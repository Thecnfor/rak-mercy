#!/usr/bin/env python3
"""Isaac 5.1 real-physics clearance recorder for the Mercury X1 venue scene.

Runs only inside full Kit.  It never opens serial devices, never teleports the
pen, and writes a failed raw run whenever any prerequisite is unavailable.
"""
from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import traceback


BASE_ADAPTER = Path("/var/workspace/docker/isaac/safe_tools/isolated_scene_adapter.py")
ROBOT_USD = Path("/var/workspace/docker/isaac/scenes/active/robots/mercury_x1.usd")
SOURCE_USD = Path("/var/workspace/docker/isaac/scenes/team_rak_finals_20260820_dual_arm_transfer_v5_low_tables/outputs/team_rak_finals_20260820_dual_arm_transfer_v5_low_tables.usd")
SCENE_CONFIG = Path("/var/workspace/docker/isaac/scenes/team_rak_finals_20260820_dual_arm_transfer_v5_low_tables/scene_config.json")
PHYSICS_HZ = 60.0
MAX_JOINT_STEP_RAD = math.radians(0.25)
ARM_UNCERTAINTY_MM = 8.0
RIGHT_NAMES = tuple(f"joint{i}_R" for i in range(1, 7))
LEFT_NAMES = tuple(f"joint{i}_L" for i in range(1, 7))
GRIPPER_NAMES = (
    "right_gripper_left_finger_joint", "right_gripper_right_finger_joint",
    "right_gripper_left_joint2", "right_gripper_right_join2",
)


def _sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _devices() -> list[str]:
    found=[]
    for pattern in ("right_arm", "left_arm", "ttyUSB*", "ttyACM*"):
        found.extend(str(path) for path in Path("/dev").glob(pattern))
    return sorted(set(found))


def _load_base():
    spec=importlib.util.spec_from_file_location("clearance_safe_base",BASE_ADAPTER)
    if spec is None or spec.loader is None: raise RuntimeError("safe_base_adapter_import_failed")
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module); return module


def _aabb(stage, path: str, cache=None):
    from pxr import Usd, UsdGeom
    if cache is None:
        cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_], useExtentsHint=True)
    prim=stage.GetPrimAtPath(path)
    if not prim.IsValid(): raise RuntimeError(f"aabb_prim_missing:{path}")
    bounds=cache.ComputeWorldBound(prim).ComputeAlignedRange()
    return tuple(float(v) for v in bounds.GetMin()), tuple(float(v) for v in bounds.GetMax())


def _aabb_distance_mm(first, second) -> float:
    lo1,hi1=first; lo2,hi2=second
    delta=[max(lo2[i]-hi1[i],lo1[i]-hi2[i],0.0) for i in range(3)]
    return math.sqrt(sum(value*value for value in delta))*1000.0


def _collision_inventory(stage):
    from pxr import UsdPhysics
    active=[]
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            active.append(str(prim.GetPath()))
    structural=[]; disabled=[]
    for prim in stage.TraverseAll():
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            path=str(prim.GetPath()); structural.append(path)
            attr=prim.GetAttribute("physics:collisionEnabled")
            if attr.IsValid() and attr.HasAuthoredValueOpinion() and attr.Get() is False: disabled.append(path)
    required=(
        "/World/Tables/table_1/Top", "/World/Tables/table_2/Top",
        "/World/Robot/mercury_x1/link1_L/collisions", "/World/Robot/mercury_x1/link7_L/collisions",
        "/World/Robot/mercury_x1/link1_R/collisions", "/World/Robot/mercury_x1/link7_R/collisions",
        "/World/Robot/mercury_x1/left_gripper/left_gripper_base/collisions",
        "/World/Robot/mercury_x1/right_gripper/right_gripper_base/collisions",
    )
    missing=[path for path in required if path not in active]
    inactive=sorted(set(structural)-set(active))
    return structural,active,disabled,missing,inactive


def _contact_allowed(first: str, second: str, phase: str) -> bool:
    pair="|".join(sorted((first,second)))
    if "wheel" in pair.lower() and "/World/Field/Floor" in pair: return True
    pen="/World/Pens/table_1_pen_1"
    table="/World/Tables/"
    if "/World/Pens/" in pair and table in pair:
        if pen not in pair: return True
        return phase in {"before_pick","pregrasp","approach","contact","close","release_approach","release","settle"}
    if pen in pair and "/right_gripper/" in pair and phase in {"close","lift","transport","place_pre","release_approach","release"}: return True
    return False


async def run() -> None:
    import carb
    import numpy as np
    import omni.kit.app
    import omni.timeline
    import omni.usd
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.types import ArticulationAction
    from omni.physx import get_physx_simulation_interface
    from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Usd, UsdGeom, UsdPhysics

    output=Path(os.environ.get("X1_CLEARANCE_RAW_JSON","/var/workspace/temp/x1-clearance/raw.json"))
    output.parent.mkdir(parents=True,exist_ok=True)
    result={"passed":False,"failure_reasons":[],"run":{}}
    app=omni.kit.app.get_app()
    try:
        if os.environ.get("ROS_DOMAIN_ID")!="46": raise RuntimeError("ros_domain_must_be_46")
        if os.environ.get("X1_NAMESPACE","").strip("/")!="x1_sim": raise RuntimeError("namespace_must_be_x1_sim")
        if _devices(): raise RuntimeError("real_robot_device_present")
        for _ in range(5): await app.next_update_async()
        base=_load_base(); await base.configure()
        stage=omni.usd.get_context().get_stage()
        if stage is None: raise RuntimeError("stage_missing")
        scene_usd=Path(str(base.ARGS.scene_usd))
        collision,active_collision,disabled,missing,inactive_collision=_collision_inventory(stage)
        source_stage=Usd.Stage.Open(str(SOURCE_USD))
        if source_stage is None: raise RuntimeError("source_scene_stage_open_failed")
        source_collision,_,source_disabled,_,_=_collision_inventory(source_stage)
        top_surfaces={}
        for table in ("table_1","table_2"):
            _,hi=_aabb(stage,f"/World/Tables/{table}/Top"); top_surfaces[table]=hi[2]
        expected_removed=[
            "/World/Pens/table_1_pen_3/CollisionAndFallbackVisual",
            "/World/Pens/table_1_pen_4/CollisionAndFallbackVisual",
        ]
        removed_collision=sorted(set(source_collision)-set(active_collision))
        asset_ok=(not source_disabled and not disabled and not missing
                  and not (set(expected_removed)-set(removed_collision))
                  and all(abs(value-.650)<=1e-6 for value in top_surfaces.values()))
        result["assets"]={"scene_usd_sha256":_sha(scene_usd),"robot_usd_sha256":_sha(ROBOT_USD),
            "scene_config_sha256":_sha(SCENE_CONFIG),"collision_prim_count":len(active_collision),
            "collision_prims":active_collision,
            "source_collision_prim_count":len(source_collision),
            "source_collision_prims":source_collision,
            "source_scene_usd_sha256":_sha(SOURCE_USD),
            "active_collision_prim_count":len(active_collision),
            "active_collision_prims":active_collision,
            "removed_collision_prims":removed_collision,
            "all_required_collisions_enabled":not disabled and not missing,"disabled":disabled,
            "missing":missing,"table_top_surface_z_m":top_surfaces}
        result["simulation"]={"physics_hz":PHYSICS_HZ,"synthetic_attachment":False,
            "rigid_body_disabled":False,"teleport_used":False}
        if not asset_ok: raise RuntimeError("asset_or_table_height_audit_failed")

        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0)
        timeline=omni.timeline.get_timeline_interface(); timeline.play()
        for _ in range(60): await app.next_update_async()
        arm=SingleArticulation("/World/Robot/mercury_x1/base_link",reset_xform_properties=False); arm.initialize()
        # Both grippers are joints of the single Mercury X1 articulation; they
        # are not independent articulation roots in the official USD.
        gripper=arm
        arm_names=tuple(str(value) for value in arm.dof_names); grip_names=arm_names
        power_on=np.asarray(arm.get_joint_positions(),dtype=float)
        result["observed_power_on"]={"arm_dof_names":list(arm_names),
            "arm_joint_positions_rad":[float(value) for value in power_on],
            "gripper_dof_names":list(GRIPPER_NAMES),
            "gripper_joint_positions_rad":[float(power_on[arm_names.index(name)]) for name in GRIPPER_NAMES]}
        if not all(name in arm_names for name in LEFT_NAMES+RIGHT_NAMES): raise RuntimeError("arm_dof_names_missing")
        if not all(name in grip_names for name in GRIPPER_NAMES): raise RuntimeError("gripper_dof_names_missing")
        plan_path=Path(os.environ.get("X1_CLEARANCE_JOINT_PLAN",""))
        nav_path=Path(os.environ.get("X1_CLEARANCE_NAV_RESULT",""))
        if not plan_path.is_file(): raise RuntimeError("joint_plan_missing")
        if not nav_path.is_file(): raise RuntimeError("nav2_result_missing")
        plan=json.loads(plan_path.read_text(encoding="utf-8")); nav=json.loads(nav_path.read_text(encoding="utf-8"))
        if plan.get("schema")!="isaac_clearance_joint_plan/v1": raise RuntimeError("joint_plan_schema_mismatch")
        if nav.get("schema")!="isaac_clearance_nav_result/v1": raise RuntimeError("nav2_result_schema_mismatch")

        phase="power_on"; forbidden=[]; allowed=[]; all_contacts=[]
        min_arm=float("inf"); min_finger=float("inf")
        pen_path="/World/Pens/table_1_pen_1"
        pen_peak_z=-float("inf")
        tables=[f"/World/Tables/{table}/{part}" for table in ("table_1","table_2") for part in ("Top","Leg_1","Leg_2","Leg_3","Leg_4")]
        arm_collision=[path for path in active_collision if "/World/Robot/mercury_x1/" in path and
                       ("/link" in path or "gripper_base/collisions" in path)]
        finger_collision=[path for path in active_collision if "/right_gripper/" in path and "gripper_base" not in path]
        bbox_cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_],useExtentsHint=True)
        xform_cache=UsdGeom.XformCache(Usd.TimeCode.Default())
        dynamic_paths=arm_collision+finger_collision
        local_ranges={path:bbox_cache.ComputeLocalBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
                      for path in dynamic_paths}
        table_bounds={path:_aabb(stage,path,bbox_cache) for path in tables}
        table_top_bounds={table:_aabb(stage,f"/World/Tables/{table}/Top",bbox_cache)
                          for table in ("table_1","table_2")}

        def fast_aabb(path):
            bounds=local_ranges[path]; lo=bounds.GetMin(); hi=bounds.GetMax()
            matrix=xform_cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))
            points=[matrix.Transform(Gf.Vec3d(x,y,z)) for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])]
            return (tuple(min(point[i] for point in points) for i in range(3)),
                    tuple(max(point[i] for point in points) for i in range(3)))

        def sample_geometry():
            nonlocal min_arm,min_finger,pen_peak_z
            xform_cache.Clear()
            pen_position=UsdGeom.Xformable(stage.GetPrimAtPath(pen_path)).ComputeLocalToWorldTransform(0).ExtractTranslation()
            pen_peak_z=max(pen_peak_z,float(pen_position[2]))
            for robot_path in arm_collision:
                rb=fast_aabb(robot_path)
                for obstacle in tables:
                    min_arm=min(min_arm,_aabb_distance_mm(rb,table_bounds[obstacle]))
            for finger in finger_collision:
                fb=fast_aabb(finger)
                for table in ("table_1","table_2"):
                    min_finger=min(min_finger,_aabb_distance_mm(fb,table_top_bounds[table]))
            headers,_=get_physx_simulation_interface().get_contact_report()
            for header in headers:
                first=str(PhysicsSchemaTools.intToSdfPath(header.collider0)); second=str(PhysicsSchemaTools.intToSdfPath(header.collider1))
                event={"phase":phase,"collider0":first,"collider1":second,"type":int(header.type)}
                all_contacts.append(event)
                (allowed if _contact_allowed(first,second,phase) else forbidden).append(event)

        async def command(controller, names, targets, label, minimum_steps=1):
            nonlocal phase
            phase=label
            dof_names=tuple(str(v) for v in controller.dof_names)
            joint_indices=np.asarray([dof_names.index(name) for name in names],dtype=np.int32)
            current_full=np.asarray(controller.get_joint_positions(),dtype=float)
            initial=current_full[joint_indices].copy()
            target=np.asarray(targets,dtype=float)
            if len(target)!=len(joint_indices): raise RuntimeError(f"joint_target_shape_invalid:{label}")
            diagnostic={"label":label,"joint_names":list(names),
                "initial":[float(value) for value in initial],
                "target":[float(value) for value in target]}
            result.setdefault("joint_command_diagnostics",[]).append(diagnostic)
            final=initial.copy()
            try:
                steps=max(int(minimum_steps),int(math.ceil(float(np.max(np.abs(target-initial)))/MAX_JOINT_STEP_RAD)),1)
                diagnostic["interpolation_frames"]=steps
                for index in range(1,steps+1):
                    desired=initial+(target-initial)*(index/steps)
                    controller.apply_action(ArticulationAction(
                        joint_positions=desired,joint_indices=joint_indices))
                    await app.next_update_async(); sample_geometry()
                settled=False
                for settle_frame in range(1,121):
                    controller.apply_action(ArticulationAction(
                        joint_positions=target,joint_indices=joint_indices))
                    await app.next_update_async(); sample_geometry()
                    final=np.asarray(controller.get_joint_positions(),dtype=float)[joint_indices]
                    if float(np.max(np.abs(final-target)))<=math.radians(1.0):
                        diagnostic["settle_frames"]=settle_frame
                        settled=True
                        break
                if not settled:
                    diagnostic["settle_frames"]=120
                    raise RuntimeError(f"joint_feedback_not_converged:{label}")
            finally:
                try:
                    final=np.asarray(controller.get_joint_positions(),dtype=float)[joint_indices]
                    per_joint_error_deg=np.degrees(np.abs(final-target))
                    diagnostic.update({
                        "final":[float(value) for value in final],
                        "per_joint_error_deg":[float(value) for value in per_joint_error_deg],
                        "max_error_deg":float(np.max(per_joint_error_deg)),
                        "moved_delta_deg":[float(value) for value in np.degrees(final-initial)],
                    })
                except Exception as feedback_exc:
                    diagnostic["feedback_capture_error"]=str(feedback_exc)

        current=np.asarray(arm.get_joint_positions(),dtype=float)
        initial_left=[float(current[arm_names.index(name)]) for name in LEFT_NAMES]
        initial_right=[float(current[arm_names.index(name)]) for name in RIGHT_NAMES]
        order=plan["selected_order"].split("_then_")
        for side in order:
            names=LEFT_NAMES if side=="left" else RIGHT_NAMES
            await command(arm,names,plan[f"stow_{side}_rad"],f"stow_{side}")
        result["initial_pose"]={"both_arms_auto_stowed":True,"selected_order":plan["selected_order"],
            "power_on_left_rad":initial_left,"power_on_right_rad":initial_right,
            "stow_left_rad":plan["stow_left_rad"],"stow_right_rad":plan["stow_right_rad"]}

        pen_before=UsdGeom.Xformable(stage.GetPrimAtPath(pen_path)).ComputeLocalToWorldTransform(0).ExtractTranslation()
        pen_peak_z=float(pen_before[2])
        for step in plan["steps"]:
            if "right_arm_rad" in step: await command(arm,RIGHT_NAMES,step["right_arm_rad"],step["phase"],step.get("interpolation_steps",1))
            elif "right_gripper_rad" in step: await command(gripper,GRIPPER_NAMES,step["right_gripper_rad"],step["phase"])
        phase="settle"
        for _ in range(120): await app.next_update_async(); sample_geometry()
        pen_after=UsdGeom.Xformable(stage.GetPrimAtPath(pen_path)).ComputeLocalToWorldTransform(0).ExtractTranslation()
        lift_mm=float(pen_peak_z-pen_before[2])*1000.0
        table2_lo,table2_hi=_aabb(stage,"/World/Tables/table_2/Top")
        placed_on_table2=(table2_lo[0]<=pen_after[0]<=table2_hi[0]
            and table2_lo[1]<=pen_after[1]<=table2_hi[1]
            and table2_hi[2]<=pen_after[2]<=table2_hi[2]+0.030)
        run_data={"passed":False,"nav_table_1_reached":nav.get("table_1_reached") is True,
            "nav_table_2_reached":nav.get("table_2_reached") is True,"ik_fk_passed":plan.get("ik_fk_passed") is True,
            "joint_feedback_passed":True,"stage_timeouts_passed":True,
            "dynamic_contact_grasp":any(pen_path in (item["collider0"]+item["collider1"]) and "/right_gripper/" in (item["collider0"]+item["collider1"]) for item in allowed),
            "pen_placed_on_table_2":bool(placed_on_table2),"forbidden_contacts":forbidden,
            "minimum_arm_raw_mm":min_arm,"minimum_arm_conservative_mm":min_arm-ARM_UNCERTAINTY_MM,
            "minimum_fingertip_table_raw_mm":min_finger,
            "minimum_navigation_raw_mm":float(nav.get("minimum_raw_mm",0.0)),
            "minimum_navigation_conservative_mm":float(nav.get("minimum_raw_mm",0.0))-ARM_UNCERTAINTY_MM,
            "pen_lift_mm":lift_mm,"allowed_contacts":allowed,"contact_event_count":len(all_contacts)}
        run_data["passed"]=(run_data["nav_table_1_reached"] and run_data["nav_table_2_reached"] and run_data["ik_fk_passed"]
            and run_data["dynamic_contact_grasp"] and run_data["pen_placed_on_table_2"] and not forbidden
            and min_arm>=18 and min_finger>=2 and run_data["minimum_navigation_raw_mm"]>=58 and lift_mm>=30)
        result["run"]=run_data; result["passed"]=run_data["passed"]
        if not result["passed"]: result["failure_reasons"].append("acceptance_threshold_not_met")
    except Exception as exc:
        result["failure_reasons"].append(str(exc)); result["traceback"]=traceback.format_exc()
    finally:
        output.write_text(json.dumps(result,indent=2,sort_keys=True)+"\n",encoding="utf-8")
        carb.log_warn(f"Isaac real clearance raw evidence: passed={result['passed']} output={output}")
        app.post_quit(0 if result["passed"] else 24)


TASK=None
def _done(task):
    import carb
    error=task.exception()
    if error is not None: carb.log_error(str(error))


def main():
    global TASK
    from omni.kit.async_engine import run_coroutine
    TASK=run_coroutine(run()); TASK.add_done_callback(_done)


if __name__=="__main__": main()
