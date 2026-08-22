# ROS2 pick execution adapter

`pick_ros2_execution` consumes `/x1/coordinate_chain/result` only when it has
`trusted_for_execution:true`, plus `/x1/pick/dry_run_plan`.  Its default is
`dry_run:true`; no invocation in this repository starts the node.

Live admission needs every one of: `dry_run:false`, `enable_live_execution`,
`operator_confirmed`, `validated_calibration`, a ready
`FollowJointTrajectory` server and fresh `JointState` feedback.  Every stage
also needs a separate confirmation message and an approved named joint
trajectory.  Cartesian poses are never converted to joint commands here.

The node instantiates the Mercury `FollowJointTrajectory`, `JointState`, and
`SetBool` interface probes, but deliberately inhibits goal/service dispatch
until an independently reviewed collision and tracking monitor is added.
Cancellation is harmless until such a goal exists.  The same admission and
step validators are ROS-free and used by simulation tests.
