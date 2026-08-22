# Pen pick dry-run: perception-to-motion hand-off

This is an offline-safe hand-off. `pen_pick_dry_run` never creates Nav2,
Mercury arm, base, or gripper action clients, and it never publishes a motion
command. It consumes the already fail-closed `/x1/grasp/pen_candidates` JSON
and publishes a reviewable intent sequence on `/x1/pick/dry_run_plan`.

## Current path and deliberate boundary

```text
rectified left YOLO -> pen_features (same stamp) -> depth + rectified CameraInfo
  -> dynamic camera-relative table removal -> validated base_link candidate
  -> pen_pick_dry_run -> [future reviewed Mercury single-arm adapter]
```

The candidate contract must provide exactly one target, exact-frame timestamp,
`valid:true`, `trusted_for_grasp:true`, `target_frame:base_link`, the base-frame
grasp point, pen axis, approach normal, and depth/detection quality fields.
The dry-run planner rejects stale input, weak depth/detection evidence, a
missing/invalid site workspace, incompatible axes, and every pose that exits
that workspace. The default equal min/max bounds are intentionally invalid.
On success it emits pre-grasp, approach, grasp, close-gripper intent, lift,
and safe retreat. The current bench/offline workflow deliberately does not
navigate. A navigation-arrival gate is available only through the explicit
`include_navigation_gate:true` future option.

`tool_basis_columns_base` is explicit: `[gripper_long_axis, lateral_axis,
outbound_approach_axis]`. A hardware adapter must bind this to the confirmed
end-effector TCP/tool convention; it must not reinterpret it silently.

## Execution boundary (not implemented)

Even with all three parameters below set true, this node reports
`execution_eligible:false` and `motion_adapter_not_implemented`. This is
intentional. A future adapter must independently check the live candidate age,
Nav2 arrival, arm feedback, collision/reachability result, E-stop/drive state,
gripper feedback, and operator approval immediately before each action.

- `site_profile_validated`: a physical workspace, tool orientation, approach
  sign, collision model, and named arm are validated for this robot/table.
- `enable_execution`: explicit per-launch authorization, default `false`.
- `operator_approved`: explicit attended approval, default `false`.

## Offline end-to-end acceptance

`pen_pick_simulation.py` replays the candidate through a single selected arm
(`left` or `right`) plus gripper. `FakeNav2Adapter` remains a future interface
fixture and is not a prerequisite for the current local pick simulation. No
simulation path represents two arms jointly grasping one pen.

The simulator never imports `pymycobot`, rclpy, or a Nav2 client, and never
publishes `joint_states`. Its `pen_pick_trace/v1` JSON records terminal status,
ordered per-step result, timeout, and selected arm. The first rejection,
timeout, unreachable pose, gripper failure, stale/invalid candidate, missing
approval, or cancellation ends the trace immediately; later steps are absent.

Use `write_simulation_trace` only for evidence output. It rejects paths outside
`E:/a_robot/temp/deyes`. Tests use a generated minimal candidate and do not
copy models, images, reports, or other `temp/deyes` data into Git.

When moving to hardware, retain the planner, gates, JSON trace, and tests, and
replace only fake adapters with separately reviewed concrete adapters. The real
adapter must still provide cancellation, feedback, timeout, result, collision,
and selected-arm/gripper feedback before it can be considered for execution.

## On-site gates before any adapter is added or enabled

1. Use a newly measured 9x6-inner-corner board square length for physical
   stereo calibration. The YAML must say `source: physical_checkerboard` and
   `validated: true`; specification/debug calibration is never a substitute.
2. Complete the four-distance depth checks and stability acceptance for the
   same camera pair/resolution.
3. Complete physical `left_camera_optical_frame -> base_link` hand-eye
   calibration with identity-bound robot/camera pair evidence and validated
   residual limits. Do not use the dynamic table-plane TF for this.
4. Validate the specific arm TCP, gripper polarity/force, tool orientation,
   table/robot workspace bounds, collision clearance, and Nav2 base stopping
   pose under attended low-speed dry-runs.
5. Only then implement and review a concrete Mercury/Nav2 adapter against the
   discovered robot interfaces. It must fail closed if any live gate changes.

## Offline two-arm co-grasp experiment

The ROS-free co-grasp modules model the explicitly selected experiment in
which both grippers contact the same pen. This is separate from the competition
rubric's usual two-arm case (two arms grasping different objects), and it must
not be presented as a physically validated or scoring-complete behavior.

`dual_pen_cograsp_contract` accepts exactly one trusted `base_link` candidate
from `/x1/grasp/pen_candidates`. It uses the two
`grasp_interval_base_m` points, assigns the higher base-link Y contact to the
left arm, and rejects ambiguous, stale, short, long, untrusted, or out-of-zone
geometry. A validated site profile must separately provide left/right tool-point
workspaces and a safe lift vector; defaults cannot produce a usable plan.

The no-navigation plan is:

```text
both pre-grasp ready barrier
  -> synchronous approach
  -> synchronous contact
  -> synchronous close
  -> confirm both grippers
  -> synchronous lift
  -> verified hold duration
```

The plan publishes an object basis only. A future hardware adapter must apply
independently validated left/right TCP transforms and IK; the offline layer does
not infer joint angles or tool quaternions. Any single-side rejection, timeout,
cancel, missing feedback, excessive barrier skew, or missing grip confirmation
prevents later phases. An unverified stop, or a lift/hold failure, locks the
trace for attended recovery.

Run the complete ROS-free regression on the development computer:

```powershell
$env:PYTHONPATH=(Resolve-Path Deyes/src/deyes_stereo).Path
python -m pytest Deyes/test -q
```

The end-to-end synthetic replay exercises rectified image/YOLO features,
32FC1 depth, rectified projection, camera-relative table removal, known
synthetic extrinsics, base-frame candidates, the co-grasp plan, fake adapters,
and the hold barrier. It proves software contracts and failure routing only;
it does not prove stereo accuracy, physical reachability, collision clearance,
gripper force, or safe two-arm contact with a real pen.
