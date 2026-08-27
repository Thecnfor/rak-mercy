# Socl_ous Mercury X1 65 cm simulation evidence

This report is the simulation-owner result for production baseline `5b149e8`.
The simulation branch is `feature/socl-65cm-fullchain-sim`; the non-zero-dt
adapter is `aeee63b` and the deterministic fault matrix is `e47a827`.

## Acceptance classification

The complete chain is **tier C**, not tier B.  The deterministic adapter ran
navigation to table 1, stability, synthetic stereo/depth, exactly-one-pen
detection, projection, trajectory admission, pick, grasp verification,
navigation to table 2, place, and retreat.  All inputs and action results in
that complete trace are explicitly synthetic and `physical_validated=false`.

Tier B is limited to individually observed Isaac/ROS components:

- Isaac Sim 5.1 opened the v5 scene in ROS domain 46 with no matching real-arm
  or serial device.  Timeline time advanced from 0.333333 s to 2.333333 s.
- `/World/PhysicsScene` ran at 60 Hz.  DifferentialController `inputs:dt` was
  unconnected and fixed to 0.016666666666666666 s.  The final log contains no
  `Invalid deltaTime` event.
- A 0.30 m/s `/x1_sim/cmd_vel` pulse changed observed `/odom` x from
  0.0000134 m to 0.0029985 m.  This proves motion but also exposes severe slip;
  it is not a successful table-navigation claim.
- The isolated sparse six-joint command `[0.2,0.3,-0.2,-0.3,0.2,0.3]` produced
  `[0.2002,0.2997,-0.2002,-0.3011,0.2,0.3]` on `/x1_sim/joint_states`.
- The four right-gripper roots opened to approximately
  `[0.6705,-0.6333,-0.6977,0.5531]` rad and returned near zero after close.
- Both 1280x720 RGB and 32FC1 depth topics contained non-zero bytes.  This is
  only a topic/data health observation: no live YOLO/projector result is
  claimed and the sampled four images were not one exact-stamp stereo set.

The pen pose trace is also tier C.  The v5 asset has no contact grasp
constraint, so the adapter explicitly disables rigid-body dynamics while the
pen is synthetically attached, observes z `0.656 -> 0.716 m` (+60 mm), places
it at `[2.87,0.14,0.668] m`, then re-enables rigid-body dynamics.  This passes
the 30 mm state-machine gate and the table-2 XY/Z fixture bounds, but is not a
contact-driven grasp.  The orchestrator independently requires both
`grasp_verification.success=true` and `navigation_permitted=true`; an adapter
cannot reach table 2 merely by marking the verification phase successful.

The physical head truth `[-54.93,2.63] deg` is retained in the tier-C contract,
but it was **not applied or verified** in the Isaac physics run.  The v5
`head`/`eye` joint observations were approximately zero and the mapping from
those articulation joints to the venue head convention is unverified.  This
is an explicit remaining asset-adapter gap, not a 65 cm truth change.

## Fault matrix

Eleven injected failures stop at their owning phase with `retry_count=0`:
table-1 navigation, stale snapshot, zero pen, multiple pens, invalid depth,
healthy-plane/650 mm conflict, unavailable projector, target bounds, IK,
grasp verification, and table-2 navigation.  Missing and low-quality planes
continue only with `fixed_height_unverified` warnings and the fixed 650 mm
height.  Fixed XY succeeds only under the explicit degraded flag and remains
synthetic.

## Reproduction

On Socl_ous, with the repository at this branch:

```bash
cd /var/workspace/rak-mercy
Deyes/tools/run_socl_65cm_fullchain_sim.sh
```

The script generates a non-destructive 650 mm USD layer, runs the tier-C
success/fault matrix, accepts the already-authorized Omniverse EULA through
`OMNI_KIT_ACCEPT_EULA=YES`, starts headless Isaac in ROS domain 46, applies the
motion preflight, writes physics evidence, and exits.  Logs and generated USD
assets remain under `/var/workspace/temp/x1-65cm-sim`; it never imports a robot
serial driver or opens a physical command device.

The final one-key self-test exited 0 in 53 seconds and wrote an accepted
physics/pen JSON.  Its Isaac log contains no native pure-virtual abort,
`Invalid deltaTime`, or disabled-rigid-body velocity error.

Startup history is retained rather than hidden: an earlier `--exec` launch
aborted natively with `pure virtual method called` before the scene was ready,
and a later run wrote accepted evidence but outlived its 120-second wrapper
because full-kit shutdown stalled.  Commit `24f0c89` yields several Kit-owned
update frames before opening the stage and makes the one-key wrapper watch a
fresh, run-specific evidence file, interrupt exactly that simulation process,
and never retry a competition action.  No further Isaac relaunch is required.
The subsequent review hardening pins the external safe adapter SHA, validates
table-2 place bounds and adds bounded PID cleanup; per supervisor direction it
was checked with unit tests and shell syntax without another Isaac relaunch.

## Tests

```bash
PYTHONPATH=Deyes:Deyes/tools:Deyes/src/deyes_stereo \
  pytest -q Deyes/test/test_competition_fullchain_sim.py \
  Deyes/test/test_isaac_competition_65cm_adapter.py \
  Deyes/test/test_competition_scene_override.py
# 24 passed

PYTHONPATH=Deyes:Deyes/tools:Deyes/src/deyes_stereo pytest -q Deyes/test
# 363 passed, 2 failed
```

The two aggregate failures are unchanged production-baseline tests:
`scripts/race_onekey_try.sh` invokes `PYTHON_BIN=py`, while this Linux host has
`python3` and no `py`.  The script is byte-identical to baseline `5b149e8` and
is integration-owned; the simulation branch does not patch production deploy.

## Not passed offline

- No Nav2 action reached either table in Isaac.  The base's contact/slip model
  makes the small odometry response unsuitable for a tier-B navigation claim.
- No contact-driven pen grasp or release exists in the v5 asset.
- Isaac's full-kit startup/shutdown path has demonstrated a native race; the
  successful one-key run is reproducible evidence, not a guarantee that Kit
  itself can never fail during application bootstrap.
- Production `competition_pick_target` live exact-stamp vision is owned by the
  integration branch and is not claimed here.  The real venue PnP remains
  `usable:false` at 4.1917 px RMS.
- Collision-clearance and live hardware admission remain fail-closed field or
  integration checks; simulation does not alter the 2026-08-27 robot truth.

The hashes and exact external log paths are in `manifest.json` in this folder.
