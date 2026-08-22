# ROS 1 navigation → ROS 2 single-shot pick integration

The ROS 1 adapter is deliberately inactive until all three parameters are set:
`~enable_navigation:=true`, `~operator_confirmed:=true`, and a non-empty
`~site_profile_path`.  Its site profile has an empty allowlist by default.
It sends at most one `move_base` goal per mission, never publishes `cmd_vel`,
never kills processes, never retries, and emits failure evidence on ABORTED,
CANCELED/PREEMPTED, unavailable server, or the 90-second timeout. A
`MoveBaseGoal` is itself a motion command: successful evidence therefore marks
`navigation_goal_sent:true` and `commands_emitted:true`; the adapter merely
does not bypass navigation with direct velocity commands.

## Required contracts

- ROS 2 publishes `std_msgs/msg/String` `/x1/pick/nav_mission`; bridge maps it
  to ROS 1 `std_msgs/String` of the same name.
- Adapter publishes ROS 1 `std_msgs/String` `/x1/pick/navigation_evidence`;
  bridge maps it back to ROS 2.  It carries `mission_id`, `nav_epoch`, result,
  pose error and measured odom speed.
- The ROS 2 coordinator only arms the snapshot when the evidence says
  `succeeded` and independently verifies its fresh ID, map error and stable
  odometry window.
- `/map -> /odom -> base_link` must be supplied by the existing navigation
  stack.  The adapter reads `/amcl_pose` and `/odom`; it does not own TF.

## Tomorrow's dry-run order

1. On both machines, perform read-only checks first:

   ```bash
   rostopic list; rostopic type /amcl_pose; rostopic type /odom
   rosrun tf tf_echo map base_link
   ros2 topic list -t | grep -E 'nav_mission|navigation_evidence|nav_gate|transaction_status'
   ros2 action list -t
   ```

2. Start the bridge with only its standard dynamic mappings, then verify types
   before any mission is published:

   ```bash
   ros2 run ros1_bridge dynamic_bridge
   rostopic type /x1/pick/nav_mission
   rostopic type /x1/pick/navigation_evidence
   ros2 topic info -v /x1/pick/nav_mission
   ros2 topic info -v /x1/pick/navigation_evidence
   ```

   ROS 1 and ROS 2 must use the same clock source. If either side has
   `use_sim_time=true`, bridge `/clock` and verify both clocks before testing
   evidence freshness.

3. Copy `maps/pick_navigation.site.template.yaml` outside the repository and
   add exactly one surveyed table-front `map` pose.  Keep `enable_navigation`
   false and start the adapter from the repository root:

   ```bash
   python3 scripts/pick_navigation_adapter_ros1.py \
     _enable_navigation:=false \
     _operator_confirmed:=false \
     _site_profile_path:=/absolute/path/pick_navigation.site.yaml
   ```

   Start the ROS 2 side in its safe default mode:

   ```bash
   ros2 launch deyes_bringup navigation_single_shot_pick.launch.py \
     dry_run:=true enable_live_execution:=false operator_confirmed:=false \
     model_path:=/absolute/path/pen.engine \
     stereo_calibration_path:=/absolute/path/stereo.yaml \
     extrinsics_path:=/absolute/path/handeye.yaml \
     site_profile_path:=/absolute/path/right_arm_execution.yaml
   ```

   Publish one identity-bound mission whose pose exactly matches the site
   allowlist. With navigation disabled it must be rejected and no move-base
   goal may be created:

   ```bash
   ros2 topic pub --once /x1/pick/nav_mission std_msgs/msg/String \
     "{data: '{\"mission_id\":\"table-pick-001\",\"nav_epoch\":1,\"target_id\":\"table-1-front\",\"pose\":{\"frame_id\":\"map\",\"x\":0.0,\"y\":0.0,\"yaw_rad\":0.0}}'}"
   ```

   Replace the three zero pose values with the exact surveyed allowlist pose;
   a near-but-not-identical mission is rejected.

4. With the base area clear and operator controlling the robot, set all three
   gates true, run one approved mission, and inspect evidence.  It must not
   become `succeeded` until AMCL pose error is at most 0.05 m/0.08 rad and
   `/odom` remains below 0.01 m/s and 0.02 rad/s for 0.5 s.

5. Only then connect coordinator `/x1/pick/nav_gate` to the single-shot
   snapshot node in dry-run mode.  Do not enable the arm during this check.
