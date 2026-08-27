# deyes_ik_server

`ExecuteCartesianStage` action server for Mercury X1, backed by [ikpy](https://github.com/Phylliade/ikpy).

The server slot at `Deyes/src/deyes_interfaces/action/ExecuteCartesianStage.action` was
already defined ahead of time — this package fills it with an actual solver
so the state machine can move from "publish TF2" to "actuate" the moment
vision calibration is trusted.

## What it solves
- 7-DOF right or left arm of Mercury X1, joint names ``joint{1..7}_{R,L}``
- Pose contract: ``[x_m, y_m, z_m, rx_deg, ry_deg, rz_deg]`` (XYZ extrinsic Euler)
- Output: ``float64[6] final_joint_deg`` (vendor firmware refuses degrees outside URDF limits)

## Why ikpy
- Numerical Levenberg-Marquardt with built-in Jacobian — **no manual
  derivation**. We trade sub-millisecond solve times for not having to
  debug a closed-form solver at 8pm the night before a competition.
- Reads joint limits / axes / origins straight from the URDF, so a
  re-calibrated URDF just needs the ``DEYES_MERCURY_URDF`` env var.
- Pure Python, single ``pip install ikpy scipy``.

## File map
```text
deyes_ik_server/
├── deyes_ik_server/
│   ├── ikpy_solver.py           # IkpySolver7DOF — numerical IK
│   ├── mock_solver.py           # MockSolver — fails closed if ikpy missing
│   ├── urdf_loader.py           # Resolve URDF + arm joint names
│   ├── ik_action_server.py      # ExecuteCartesianStage action server
│   └── ikpy_solver_smoketest.py # CLI: solve one pose, print + verify FK
├── launch/ik_server.launch.py
├── package.xml
├── setup.py
└── setup.cfg
```

## Deploy (after teammate's vision calibration finishes)

```bash
# 1. Sync this package onto the X1
rsync -av --delete \
  /Volumes/YC/ROBOTAC/rak-mercy/Deyes/src/deyes_ik_server/ \
  elephant@192.168.43.60:/home/elephant/mercury_x1_ros2/src/deyes_ik_server/

# 2. Install ikpy (user-level, does not touch system Python)
ssh elephant@192.168.43.60 'pip3 install --user ikpy scipy'

# 3. Build inside the ROS2 workspace
ssh elephant@192.168.43.60 '
  cd /home/elephant/mercury_x1_ros2 &&
  source /opt/ros/galactic/setup.bash &&
  export PYTHONPATH=$HOME/.local/lib/python3.8/site-packages:$PYTHONPATH &&
  colcon build --symlink-install --packages-select deyes_ik_server &&
  source install/setup.bash'

# 4. Smoke-test the solver (no arm connected)
ssh elephant@192.168.43.60 '
  cd /home/elephant/mercury_x1_ros2 &&
  source /opt/ros/galactic/setup.bash &&
  source install/setup.bash &&
  export PYTHONPATH=$HOME/.local/lib/python3.8/site-packages:$PYTHONPATH &&
  ros2 run deyes_ik_server ikpy_solver_smoketest \
    --arm-side right \
    --pose 0.30 -0.10 0.18 90 0 90'

# Expected: success=True, residual_m ~= 1e-4, joint_deg printed.

# 5. Launch the action server (mock first, swap to ikpy after smoke-test)
ssh elephant@192.168.43.60 '
  cd /home/elephant/mercury_x1_ros2 &&
  source install/setup.bash &&
  ros2 launch deyes_ik_server ik_server.launch.py solver_type:=ikpy'
```

## Fail-closed design

| Failure | Result | Action contract field |
|---|---|---|
| ikpy import error | ``RuntimeError("ikpy is not installed")`` | ``success=False, failure_code=IK_VALUE_ERROR:...`` |
| URDF path missing | ``FileNotFoundError`` raised at construction | Action server never starts |
| Target outside workspace | ikpy converges to nearest reachable | ``success=True, residual_m`` returned; if > tol, ``failure_code=FK_RESIDUAL_..._M`` |
| Joint limit overflow (vendor) | ikpy clamps; we re-check against ``JOINT_LIMIT_DEG`` in ``ik_pick_pen.py`` | runtime error in script → bash auto-degrades to ``*_hardcoded.py`` |

## Drop-in integration with race_onekey

| `race_onekey.sh` | `race_onekey_ik.sh` |
|---|---|
| `pick_pen_hardcoded.py` (Z-only descent) | `ik_pick_pen.py` (IK with mm targets from env) |
| `place_pen_hardcoded.py` | `ik_place_pen.py` |
| always hardcoded | `VISION_IK=1` (default in `race_onekey_ik.sh`) → IK; `FORCE_HARDCODED=1` → always hardcoded; non-zero IK RC auto-degrades to hardcoded |

## Pending: vision hand-off
The Deyes pipeline publishes grasp candidates on `/x1/grasp/candidates_camera`
in mm. The bridge `coordinate_chain_candidate_bridge_node.py` validates
them and republishes as TF2 requests. Once the visual-cal teammate's
calibration_id is trusted, the calling state machine should:
1. Listen on TF2 for the contact point in ``base_link``.
2. Build an ``ExecuteCartesianStage`` goal: ``pose_base=[x_m,y_m,z_m,...]``.
3. Set ``arm_side="right"``, ``stage="pre_grasp"`` (or ``grasp``, ``lift``).
4. Call ``/x1/ik/execute_cartesian_stage``.
5. On ``result.success``, send the 7-element ``result.final_joint_deg`` via
   pymycobot ``Mercury.send_angles``.

Today the same handshake happens in ``scripts/ik_pick_pen.py`` via env vars.