# Mercury X1 Turing: official interface review and adapter boundary

Review date: 2026-08-20. Evidence was checked against the official
[`mercury_x1_ros2`](https://github.com/elephantrobotics/mercury_x1_ros2)
repository at commit `527e1c787c2bd86189de7c8df0f9879380ffd9c5` and the
official [ROS 2 overview](https://docs.elephantrobotics.com/docs/Mercury_X1_cn/6-SDKDevelopment/6.3-ROS2/).
The official ROS 2 documentation maps Galactic to Ubuntu 20.04; this project
therefore targets the robot's supplied Ubuntu 20.04/ROS 2 Galactic stack.

## Evidence and decision

| Official evidence | Review result | Project decision |
| --- | --- | --- |
| `turn_on_mercury_robot_turing.launch.py` is the six-axis Turing bring-up. | Turing is the formal dual-six-axis model. | Treat the head as separate, not a seventh arm axis. |
| `mercury_nav2.launch.py` launches the Nav2 stack. Bundled Galactic Nav2 provides `nav2_msgs/action/NavigateToPose`. | This is a standard goal/feedback/cancel/result action boundary. | Future base adapter may use `/navigate_to_pose` only after a read-only type probe and site validation. |
| `slider_control_turing.py` subscribes to `sensor_msgs/msg/JointState`, slices 6+6+2 joints, offsets both arm J5 by +90 degrees, then calls `pymycobot.Mercury.send_angles(..., 16, _async=True)` and two right-arm `send_angle` calls. | It is a GUI/teleoperation bridge, not an action server and gives no safe pick lifecycle. | Never publish `joint_states` as an automated grasp command. |

The stock `mercury_nav2.launch.py` currently includes the non-Turing
`turn_on_mercury_robot.launch.py`. A deployment review must therefore verify
the actual Turing base/lidar bring-up composition before using that convenience
launch unchanged; this project only contracts against the discovered Nav2
action server, not the launch-file assumption.

## Required adapter contracts

`motion_adapter_contract.py` encodes these requirements and is attached to
every dry-run plan. The read-only `motion_interface_probe` can inspect the ROS
graph, but creates no action client, no `joint_states` subscription, no vendor
SDK object, and no command publisher.

- Nav2: `/navigate_to_pose`, type `nav2_msgs/action/NavigateToPose`; goal,
  feedback, cancel, timeout, and result must be observed/handled.
- Turing arm adapter: a new adapter must provide goal acceptance, feedback,
  cancellation, timeout, result, collision checking, selected-arm feedback,
  and gripper feedback. The physical platform has two arms, but this project
  plans one selected arm per pen; it does not define cooperative dual-arm pen
  grasping. `JointState` visibility is diagnostic evidence only, never
  authorization.
- Gripper: explicit goal, feedback, timeout, and result are mandatory. The
  official slider bridge is not evidence for a gripper contract.

All contract results remain `execution_permitted:false`; a separately reviewed
adapter plus physical validation is required before that may change.

## Future on-robot read-only discovery (do not run during navigation training)

After the user declares the robot/navigation idle and approves SSH, run only
these discovery commands first, saving output under `temp/deyes` rather than
the repository:

```bash
ros2 action list -t
ros2 topic list -t
ros2 service list -t
ros2 node list
ros2 topic type /joint_states
ros2 topic info /joint_states --verbose
ros2 node info /control_slider
test -c /dev/left_arm; test -c /dev/right_arm
ls -l /dev/left_arm /dev/right_arm
```

Do not publish `joint_states`, send an action goal, invoke a service, open a
serial device, or start/stop any launch process during this discovery stage.
