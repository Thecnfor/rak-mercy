#!/usr/bin/env python3
"""Motion-enabled Isaac-only wrapper around Socl_ous' audited safe adapter.

The installed X1 graph connects DifferentialController ``dt`` to an
OnPlaybackTick output that evaluates to zero in full-kit headless mode.  This
wrapper keeps every namespace/device guard from the base adapter, then authors
a deterministic 60 Hz physics step at runtime.  It never imports robot serial
drivers and refuses to start when a known real-arm device is present.
"""
from __future__ import annotations

import importlib.util
import json
import math
import os
from pathlib import Path
import traceback
from typing import Mapping, Sequence


FIXED_PHYSICS_DT_SEC = 1.0 / 60.0
DEFAULT_BASE_ADAPTER = Path(
    "/var/workspace/docker/isaac/safe_tools/isolated_scene_adapter.py"
)
REAL_DEVICE_PREFIXES = (
    "/dev/right_arm",
    "/dev/left_arm",
    "/dev/ttyUSB",
    "/dev/ttyACM",
)


def _flag(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def motion_preflight(
    env: Mapping[str, str], *, device_paths: Sequence[str]
) -> dict[str, object]:
    reasons: list[str] = []
    if str(env.get("ROS_DOMAIN_ID", "")) != "46":
        reasons.append("ros_domain_must_be_46")
    if str(env.get("X1_NAMESPACE", "")).strip("/") != "x1_sim":
        reasons.append("namespace_must_be_x1_sim")
    for name in ("X1_EXECUTE", "X1_ENABLE_MOTION", "X1_ACKNOWLEDGE_MOTION_RISK"):
        if not _flag(env.get(name, "0")):
            reasons.append(f"{name.lower()}_required")
    real_devices = [
        path
        for path in device_paths
        if any(str(path).startswith(prefix) for prefix in REAL_DEVICE_PREFIXES)
    ]
    if real_devices:
        reasons.append("real_robot_device_present")
    return {
        "schema": "isaac_competition_65cm_motion_preflight/v1",
        "accepted": not reasons,
        "reasons": reasons,
        "ros_domain_id": 46,
        "namespace": "x1_sim",
        "physics_hz": 60.0,
        "differential_controller_dt_sec": FIXED_PHYSICS_DT_SEC,
        "motion_scope": "isaac_sim_only",
        "real_devices": real_devices,
    }


def classify_timeline_progress(
    time_before_sec: object, time_after_sec: object, *, playing: bool
) -> tuple[bool, str]:
    """Classify runtime progress without importing Isaac in unit tests."""
    if not playing:
        return False, "timeline_not_playing"
    try:
        before = float(time_before_sec)
        after = float(time_after_sec)
    except (TypeError, ValueError):
        return False, "simulation_time_invalid"
    if not math.isfinite(before) or not math.isfinite(after):
        return False, "simulation_time_invalid"
    if after <= before:
        return False, "simulation_time_not_advancing"
    return True, "ok"


def validate_synthetic_pen_trace(trace: Mapping[str, object]) -> tuple[bool, str]:
    """Validate the 30 mm lift gate while preserving synthetic provenance."""
    if trace.get("synthetic_attachment") is not True:
        return False, "synthetic_attachment_provenance_missing"
    if trace.get("rigid_body_disabled_while_carried") is not True:
        return False, "synthetic_attachment_method_missing"
    if trace.get("rigid_body_reenabled_after_release") is not True:
        return False, "synthetic_release_method_missing"
    try:
        initial = [float(value) for value in trace["initial_world_m"]]  # type: ignore[index]
        lifted = [float(value) for value in trace["lifted_world_m"]]  # type: ignore[index]
        placed = [float(value) for value in trace["placed_world_m"]]  # type: ignore[index]
    except (KeyError, TypeError, ValueError):
        return False, "pen_trace_invalid"
    if any(len(vector) != 3 for vector in (initial, lifted, placed)):
        return False, "pen_trace_invalid"
    if not all(math.isfinite(value) for vector in (initial, lifted, placed) for value in vector):
        return False, "pen_trace_invalid"
    if lifted[2] - initial[2] < 0.030:
        return False, "pen_lift_below_30mm"
    return True, "ok"


def _known_device_paths() -> list[str]:
    paths: list[str] = []
    for pattern in ("right_arm", "left_arm", "ttyUSB*", "ttyACM*"):
        paths.extend(str(path) for path in Path("/dev").glob(pattern))
    return sorted(set(paths))


def _load_base_adapter():
    path = Path(os.environ.get("X1_SAFE_BASE_ADAPTER", str(DEFAULT_BASE_ADAPTER)))
    if not path.is_file():
        raise RuntimeError(f"safe_base_adapter_missing:{path}")
    spec = importlib.util.spec_from_file_location("x1_safe_base_adapter", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"safe_base_adapter_import_failed:{path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


async def configure() -> None:
    import carb
    import omni.kit.app
    import omni.timeline
    import omni.usd
    from isaacsim.core.simulation_manager import SimulationManager
    from pxr import UsdGeom

    preflight = motion_preflight(os.environ, device_paths=_known_device_paths())
    if not preflight["accepted"]:
        raise RuntimeError("motion_preflight_rejected:" + ",".join(preflight["reasons"]))
    # --exec is evaluated while the full-kit extension graph is still
    # settling.  Opening the stage in that interval intermittently ends in a
    # native pure-virtual abort before Python can report an exception.  Yield
    # to the owned Kit loop first; this is startup stabilization, never a
    # competition-action retry.
    app = omni.kit.app.get_app()
    for _ in range(5):
        await app.next_update_async()
    base = _load_base_adapter()
    await base.configure()
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("stage_missing_after_safe_adapter")
    physics_prim = stage.GetPrimAtPath("/World/PhysicsScene")
    if not physics_prim.IsValid():
        raise RuntimeError("physics_scene_missing")
    physics_hz = physics_prim.GetAttribute("physxScene:timeStepsPerSecond")
    if not physics_hz.IsValid() or int(physics_hz.Get()) != 60:
        raise RuntimeError("physics_scene_must_be_60hz")

    controller_path = (
        "/World/Robot/mercury_x1/Graph/DiffController/differential_controller"
    )
    controller = stage.GetPrimAtPath(controller_path)
    if not controller.IsValid():
        raise RuntimeError("differential_controller_missing")
    dt = controller.GetAttribute("inputs:dt")
    if not dt.IsValid():
        raise RuntimeError("differential_controller_dt_missing")
    if dt.GetConnections():
        raise RuntimeError("differential_controller_dt_connection_not_blocked")
    if abs(float(dt.Get()) - FIXED_PHYSICS_DT_SEC) > 1e-12:
        raise RuntimeError("differential_controller_dt_value_invalid")
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(10):
        await app.next_update_async()
    simulation_time_before_sec = float(SimulationManager.get_simulation_time())
    for _ in range(60):
        await app.next_update_async()
    simulation_time_after_sec = float(SimulationManager.get_simulation_time())
    runtime_ok, runtime_reason = classify_timeline_progress(
        simulation_time_before_sec,
        simulation_time_after_sec,
        playing=bool(timeline.is_playing()),
    )
    if not runtime_ok:
        raise RuntimeError(runtime_reason)

    pen_trace = None
    if _flag(os.environ.get("X1_RUN_SYNTHETIC_PICK_PLACE", "0")):
        import numpy as np
        from isaacsim.core.prims import SingleRigidPrim

        pen = SingleRigidPrim(
            "/World/Pens/table_1_pen_1",
            name="competition_single_pen",
            reset_xform_properties=False,
        )
        pen.initialize()
        initial_position, orientation = pen.get_world_pose()
        lifted_target = np.asarray(initial_position, dtype=float).copy()
        lifted_target[2] += 0.060
        # A real grasp constraint is not present in the v5 asset.  Model the
        # attachment explicitly by making the rigid body kinematic only while
        # carried; provenance remains tier C and can never be reported as a
        # contact-driven grasp.
        pen.set_linear_velocity(np.zeros(3, dtype=float))
        pen.set_angular_velocity(np.zeros(3, dtype=float))
        pen.disable_rigid_body_physics()
        pen.set_world_pose(position=lifted_target, orientation=orientation)
        for _ in range(30):
            await app.next_update_async()
        lifted_position, _ = pen.get_world_pose()
        placed_target = np.asarray([2.87, 0.14, 0.668], dtype=float)
        pen.set_world_pose(position=placed_target, orientation=orientation)
        for _ in range(2):
            await app.next_update_async()
        placed_position, _ = pen.get_world_pose()
        pen.enable_rigid_body_physics()
        pen.set_linear_velocity(np.zeros(3, dtype=float))
        pen.set_angular_velocity(np.zeros(3, dtype=float))
        pen_trace = {
            "classification": "C_synthetic_attachment_with_isaac_rigid_body_state",
            "synthetic_attachment": True,
            "rigid_body_disabled_while_carried": True,
            "rigid_body_reenabled_after_release": True,
            "initial_world_m": [float(value) for value in initial_position],
            "lifted_world_m": [float(value) for value in lifted_position],
            "placed_world_m": [float(value) for value in placed_position],
        }
        pen_ok, pen_reason = validate_synthetic_pen_trace(pen_trace)
        pen_trace["accepted"] = pen_ok
        pen_trace["reason"] = pen_reason
    active_pens = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if str(prim.GetPath()).startswith("/World/Pens/")
        and str(prim.GetParent().GetPath()) == "/World/Pens"
    ]
    evidence = {
        **preflight,
        "scene_usd": str(base.ARGS.scene_usd),
        "timeline_play_requested": True,
        "timeline_playing_after_60_updates": bool(timeline.is_playing()),
        "simulation_time_before_sec": simulation_time_before_sec,
        "simulation_time_after_sec": simulation_time_after_sec,
        "simulation_time_advanced": runtime_ok,
        "dt_connections_after_patch": [str(item) for item in dt.GetConnections()],
        "dt_value_after_patch_sec": float(dt.Get()),
        "active_pens": active_pens,
        "table_top_centers_m": {
            table: list(
                UsdGeom.Xformable(
                    stage.GetPrimAtPath(f"/World/Tables/{table}/Top")
                )
                .ComputeLocalToWorldTransform(0)
                .ExtractTranslation()
            )
            for table in ("table_1", "table_2")
        },
        "pen_trace": pen_trace,
    }
    output = Path(
        os.environ.get(
            "X1_PHYSICS_EVIDENCE_JSON",
            "/var/workspace/temp/x1-65cm-sim/logs/physics_runtime.json",
        )
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    carb.log_warn(
        "Competition 65cm Isaac physics ready: "
        f"dt={FIXED_PHYSICS_DT_SEC:.9f}s, "
        f"motion=ISAAC_ONLY, evidence={output}"
    )
    if _flag(os.environ.get("X1_EXIT_AFTER_EVIDENCE", "0")):
        app.post_quit(0)


SCENE_TASK = None


def _done(task) -> None:
    import carb

    try:
        error = task.exception()
    except Exception as exc:
        carb.log_error(f"Could not inspect competition adapter task: {exc}")
        return
    if error is not None:
        carb.log_error(f"Competition 65cm adapter failed: {error}")
        carb.log_error(
            "".join(traceback.format_exception(type(error), error, error.__traceback__))
        )


def main() -> None:
    global SCENE_TASK
    from omni.kit.async_engine import run_coroutine

    SCENE_TASK = run_coroutine(configure())
    SCENE_TASK.add_done_callback(_done)


if __name__ == "__main__":
    main()
