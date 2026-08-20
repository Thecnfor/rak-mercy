# Pen pick dry-run: perception-to-motion hand-off

This is an offline-safe hand-off. `pen_pick_dry_run` never creates Nav2,
Mercury arm, base, or gripper action clients, and it never publishes a motion
command. It consumes the already fail-closed `/x1/grasp/pen_candidates` JSON
and publishes a reviewable intent sequence on `/x1/pick/dry_run_plan`.

## Current path and deliberate boundary

```text
rectified left YOLO -> pen_features (same stamp) -> depth + rectified CameraInfo
  -> dynamic camera-relative table removal -> validated base_link candidate
  -> pen_pick_dry_run -> [future reviewed Nav2/Mercury adapter]
```

The candidate contract must provide exactly one target, exact-frame timestamp,
`valid:true`, `trusted_for_grasp:true`, `target_frame:base_link`, the base-frame
grasp point, pen axis, approach normal, and depth/detection quality fields.
The dry-run planner rejects stale input, weak depth/detection evidence, a
missing/invalid site workspace, incompatible axes, and every pose that exits
that workspace. The default equal min/max bounds are intentionally invalid.
On success it emits: navigation-arrival verification,
pre-grasp, approach, grasp, close-gripper intent, lift, and safe retreat.

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
