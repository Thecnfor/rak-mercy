# Vision-to-grasp camera-optical candidate contract

`vision_grasp_candidate_contract.py` is the ROS-free replay boundary shared by
simulation and physical-camera inputs. Its output schema is
`vision_grasp_candidates/camera_optical/v1`; `source` may distinguish the
producer, but does not change fields or validation.

```text
YOLO pen/pencil boxes -> pen_feature split/mask/axis -> exact-stamp 32FC1 depth
  + exact-stamp rectified CameraInfo + exact-stamp camera-relative table plane
  -> camera-optical grasp candidates
```

Each accepted candidate has `target_frame` equal to the stable ROS optical
frame `left_camera_optical_frame`, a
`grasp_point_camera_optical_m`, `axis_camera_optical_unit`,
`approach_normal_camera_optical_unit`, endpoints, grasp interval and quality
values. It always has `trusted_for_grasp:false` and
`physical_execution_eligible:false`; it contains no base-frame coordinates.
The coordinate-chain/hand-eye owner may consume the optical coordinates only
after its own separately validated transform gate.

`coordinate_chain_point` is a ready-to-complete point request with that stable
`source_frame`, exact `stamp_ns`, and `position_m`. The coordinate-chain owner
must add its reviewed tool `target_frame`; no matrix or transform is embedded
or accepted here.

All four producers must match **exactly** on timestamp, frame and dimensions.
The depth encoding must be `32FC1`, the depth array must match its metadata,
and the plane must be fresh camera-relative evidence. A candidate also rejects
edge truncation, incomplete axis, invalid depth, invalid endpoints, and an
expired or future timestamp. There is no nearest-frame or stale-target
fallback.

Run the software replay regression (no ROS graph or hardware connection):

```powershell
$env:PYTHONPATH=(Resolve-Path Deyes/src/deyes_stereo).Path
python -m pytest Deyes/test/test_vision_grasp_candidate_contract.py -q
```

## ROS 2 perception wrapper

`vision_grasp_candidate` is the runtime wrapper around the exact same pure
function. It subscribes to `/x1/detection/pen_features`, `/x1/stereo/depth`,
the matching rectified `CameraInfo`, and `/x1/ground/plane`. It publishes
`/x1/grasp/candidates_camera` plus
`/x1/grasp/candidates_camera/coordinate_chain_templates`. It has no TF,
motion, gripper, or robot client. Start it only after the upstream rectified
image/depth/plane producers are configured:

```bash
ros2 launch deyes_bringup vision_grasp_candidate.launch.py
```

The default cache and target-age gate are both 0.50 seconds. A source frame
must be `left_camera_optical_frame`; Isaac inputs must remap to that frame.

The tests cover one pen, two pens, merged-YOLO-box split IDs, edge truncation,
NaN depth, timestamp/frame/size mismatch, and target expiry. They use the same
function and output schema for `source="replay"`, `source="isaac_sim"`, and a
physical-topic adapter. Simulated inputs must remap their optical camera to
`left_camera_optical_frame`; aliases fail closed.
