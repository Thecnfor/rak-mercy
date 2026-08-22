# 导航与 Deyes 视觉抓取联调交接

给导航负责人的版本。明天的目标是：导航把机器人停到桌前并给出可信的“已到位且静止”证据，Deyes 随后冻结一帧，完成笔识别、深度坐标和抓取 dry-run。首轮联调不让机械臂动作。

## 先确认版本

```bash
git checkout integration/navigation-system-20260821
git pull --ff-only
git rev-parse --short HEAD
```

远端基线应不低于 `d9b7137`。

## 双方边界

导航侧负责：

- 提供 `/map -> /odom -> base_link`；
- 提供 `/amcl_pose`、`/odom` 和可用的 `move_base`；
- 把机器人停在现场测定的桌前位姿；
- 发布到位误差、速度和任务身份；
- 抓取未结束前保持底盘静止，不发送下一个目标。

Deyes 侧负责：

- 双目采集、CUDA 深度、桌面平面和 YOLO 单帧识别；
- 把目标从 `left_camera_optical_frame` 转到 `base_link`；
- 生成抓取计划并发布执行状态；
- 未通过标定或安全门禁时拒绝动作。

导航不需要理解检测框、深度或机械臂坐标；Deyes 不发布 `cmd_vel`，也不接管现有导航栈。

## 接口约定

全部联调接口是 `std_msgs/msg/String` 内的 JSON。

| 方向 | 话题 | 用途 |
|---|---|---|
| ROS 2 → 导航 | `/x1/pick/nav_mission` | 唯一一次导航任务 |
| 导航 → ROS 2 | `/x1/pick/navigation_evidence` | 到位误差和静止证据 |
| Deyes 内部输出 | `/x1/pick/nav_gate` | `PICK_ARMED` 后视觉才冻结快照 |
| Deyes 状态 | `/x1/pick/transaction_status` | 快照和事务状态 |
| Deyes 状态 | `/x1/pick/execution_status` | dry-run/抓取最终结果 |
| 人工服务 | `/x1/pick/nav_reset` | 清除一次事务锁定 |

导航任务格式：

```json
{
  "mission_id": "table-pick-001",
  "nav_epoch": 1,
  "target_id": "table-1-front",
  "pose": {"frame_id": "map", "x": 0.0, "y": 0.0, "yaw_rad": 0.0}
}
```

`mission_id` 每次任务唯一，`nav_epoch` 每次递增。`target_id` 和 `pose` 必须与现场 allowlist YAML 完全一致，不能由程序临时近似修改。

到位证据格式：

```json
{
  "schema": "pick_navigation_evidence/v1",
  "mission_id": "table-pick-001",
  "nav_epoch": 1,
  "result": "succeeded",
  "stamp_ns": 1234567890000000000,
  "position_error_m": 0.02,
  "yaw_error_rad": 0.03,
  "linear_speed_mps": 0.0,
  "angular_speed_radps": 0.0,
  "reason": "arrival_verified"
}
```

放行要求：位置误差 `≤0.05 m`、航向误差 `≤0.08 rad`、线速度 `≤0.01 m/s`、角速度 `≤0.02 rad/s`，连续静止至少 `0.5 s`。证据需约 `10 Hz` 连续发布，单条数据年龄不能超过 `0.35 s`。

## 明天启动顺序

### 1. 导航侧只读检查

```bash
rostopic type /amcl_pose
rostopic type /odom
rosrun tf tf_echo map base_link
rostopic hz /amcl_pose
rostopic hz /odom
```

先用原有导航方式确认能够到桌前，停车后 `/odom` 速度归零。此时不要启动机械臂。

### 2. 填现场桌前位姿

复制 `maps/pick_navigation.site.template.yaml` 到仓库外，例如：

```yaml
schema: pick_navigation_site/v1
allowed_targets:
  - target_id: table-1-front
    pose:
      frame_id: map
      x: 1.23
      y: 0.45
      yaw_rad: -1.57
```

这里必须填写现场测量/验证后的值。仓库模板 allowlist 为空，直接使用一定会拒绝任务。

### 3. 启动 ROS 1/ROS 2 桥

```bash
ros2 run ros1_bridge dynamic_bridge

rostopic type /x1/pick/nav_mission
ros2 topic info -v /x1/pick/navigation_evidence
```

两边必须使用同一时间源。如果任何一侧设置 `use_sim_time=true`，先正确桥接 `/clock`；实机建议全部使用系统时间。

### 4. 启动导航适配器

在 ROS 1 环境、仓库根目录执行：

```bash
python3 scripts/pick_navigation_adapter_ros1.py \
  _enable_navigation:=true \
  _operator_confirmed:=true \
  _site_profile_path:=/home/elephant/temp/deyes/pick_navigation.site.yaml
```

该适配器只发一个 `move_base` goal，不直接发 `cmd_vel`，不自动重试。它完成一单后会锁存；下一单要重启适配器，并为新任务递增 `nav_epoch`。

### 5. Deyes 侧启动安全闭环

由视觉同学准备物理双目、手眼和右臂现场 YAML 后执行：

```bash
ROBOT_IP=192.168.137.17 \
STEREO_CALIB=/path/physical_stereo.yaml \
HANDEYE_CALIB=/path/base_link_T_left_camera.yaml \
RIGHT_ARM_SITE=/path/right_arm_execution.yaml \
bash Deyes/tools/run_real_robot_dry_run.sh
```

这个入口固定 `dry_run=true`、`enable_live_execution=false`，不会驱动机械臂。

### 6. 发布一单任务

把下面位姿改成与 allowlist 完全相同的值：

```bash
ros2 topic pub --once /x1/pick/nav_mission std_msgs/msg/String \
  "{data: '{\"mission_id\":\"table-pick-001\",\"nav_epoch\":1,\"target_id\":\"table-1-front\",\"pose\":{\"frame_id\":\"map\",\"x\":1.23,\"y\":0.45,\"yaw_rad\":-1.57}}'}"
```

同时观察：

```bash
rostopic echo /x1/pick/navigation_evidence
ros2 topic echo /x1/pick/nav_gate
ros2 topic echo /x1/pick/transaction_status
ros2 topic echo /x1/pick/execution_status
```

## 首轮成功判据

应看到以下顺序：

```text
NAVIGATING
→ ARRIVED_VERIFY
→ PICK_ARMED
→ snapshot_frozen
→ dry_run_ready
→ dry_run_complete
```

并确认：

- 导航只发送一个目标，底盘到位后持续静止；
- `mission_id` 和 `nav_epoch` 从导航一直传到抓取结果；
- 快照只冻结一次，YOLO 只推理一次；
- `hardware_commands_emitted=false`；
- 没有机械臂和夹爪动作。

dry-run 完成后不会给导航“允许离开”的现场动作授权；不要因看到 `dry_run_complete` 就自动开走。正式动作阶段只有明确看到 `/x1/pick/nav_gate` 的 `state=LEAVE_GRANTED` 才能离开桌前。

## 常见问题归属

| 现象 | 优先检查 |
|---|---|
| 一直 `NAVIGATING` | bridge、mission 是否到 ROS 1、move_base 状态 |
| `navigation_evidence_timeout` | 95 秒内没有新鲜证据、时钟不一致 |
| `arrival_pose_out_of_tolerance` | AMCL 与 allowlist 桌前位姿 |
| `arrival_not_stationary` | `/odom` 未归零、局部规划器仍输出运动 |
| `mission_pose_not_exact_site_allowlist_match` | 任务 pose 与 YAML 字段不完全一致 |
| 到位但没有 `PICK_ARMED` | evidence 发布不足 0.5 秒或频率/时间戳不合格 |
| 已 `PICK_ARMED` 但不冻结 | 双目、桌面平面、关节/里程计稳定条件 |
| `LOCKED` | 查看 `reason`；修复后调用 reset，并重启单次导航适配器 |

ROS 2 协调器人工复位：

```bash
ros2 service call /x1/pick/nav_reset std_srvs/srv/Trigger '{}'
```

失败后不要自动重发任务、自动换目标或边移动边识别。保存四个联调话题、AMCL、odom 和 TF 日志后再定位责任侧。
