# Socl_ous X1 Isaac Sim（ROS 2 Jazzy）安全适配说明

本适配层只负责把**已经过现场验证的关节目标**转换为 Isaac Sim 接受的稀疏
`sensor_msgs/msg/JointState` 载荷。它不会从双笔协同计划的笛卡尔工具点推导关节角，
也不会采用现有示例里的 UR5e 假 FK、高层 action 或未验证的夹爪假设。

## 固定接口

- `ROS_DOMAIN_ID=45`
- 双臂命令：`/joint_command`，类型 `sensor_msgs/msg/JointState`
- 双臂关节：`joint1_L..joint6_L`、`joint1_R..joint6_R`
- 夹爪命令：`/gripper_command`，类型 `sensor_msgs/msg/JointState`
- 关节反馈：`/joint_states`，类型 `sensor_msgs/msg/JointState`
- 每侧夹爪四关节位置符号：`(+,-,-,+)`

命令必须通过 `name` 稀疏寻址；不得生成包含未知关节的全长零数组。左右夹爪开度
是两个独立输入，即使数值相同，也不能由一个“合并开度”替代。

## 第一次连接：只读检查 ROS 图

远端日志必须定向到 `/var/tmp`，避免 `$HOME/.ros` 因磁盘配额阻塞命令：

```bash
export ROS_DOMAIN_ID=45
export ROS_LOG_DIR=/var/tmp/rak-mercy-${USER}/ros-log
mkdir -p "$ROS_LOG_DIR"
source /opt/ros/jazzy/setup.bash
source /var/workspace/docker/isaac/workspace/install/setup.bash

ros2 node list
ros2 topic list -t
ros2 topic info -v /joint_command
ros2 topic info -v /gripper_command
ros2 topic info -v /joint_states
ros2 topic echo --once /joint_states
```

先保存以上 ROS 图和一帧反馈。确认 domain、三个消息类型、实际左右夹爪关节名和
反馈是否齐全；这一阶段不要运行 `ros2 topic pub`，也不要重启 Isaac Sim。

## 适配契约

代码位于
`Deyes/src/deyes_stereo/deyes_stereo/socl_ous_jazzy_adapter_contract.py`，无 ROS
环境也可导入。正常顺序是：

1. 从只读 ROS 图构造 `SoclOusInterfaceEvidence`。
2. 从最新 `/joint_states` 构造 `JointFeedbackSnapshot`。
3. 建立 `SoclOusExecutionProfile`：填入图中真实夹爪关节名、每个关节限制、唯一
   `profile_id`，并在审核后设置 `validated=True` 和
   `source="validated_sim_joint_targets"`。
4. 对每个阶段提供独立的 `SoclOusPhaseTargets`。臂阶段必须分别给左、右六个关节
   目标；`close` 必须分别给左、右夹爪标量。目标必须来自验证过的 X1 规划/IK，
   不能由本适配器猜测。
5. 协同执行器在前一 barrier 成功后签发新的 `SoclOusPhaseAuthorization`；授权必须
   绑定 profile、目标、阶段和阶段序号。`lift`/`hold` 还必须携带双夹爪确认。适配器
   不允许跳阶段，也不把“发布成功”当作“已抓稳”。
6. 先调用 `audit_socl_ous_phase(...)`。它只返回门禁结论，不构造发布载荷。
7. 保持 `enable_execution=False` 完成 dry-run。此时返回
   `commands_emitted=false`、`publish_allowed=false`，也不会返回 `command` 字段。
8. 只有仿真无人进入危险区域、计划与反馈仍新鲜、所有限制再次检查通过后，才对
   单个阶段显式设置 `enable_execution=True`。每阶段执行后必须重新读取反馈再进入
   下一阶段。

`prepare_socl_ous_phase_command(...)` 只构造一条可发布载荷，本身不发布，所以
`commands_emitted` 始终为 `false`。真正的 ROS 发布器应只在 `publish_allowed=true`
时组装 `JointState`，调用 `publish()` 后才能在自己的审计日志里记录
`commands_emitted=true`。当前仓库故意没有提供绕过这些门禁的一键执行节点。

## 失败即停止

以下任一情况都不得发布：接口未实时检查、domain 不是 45、消息类型不一致、反馈或
计划过期、反馈缺失任一所需关节、profile/阶段目标未验证或 ID 不一致、目标越限、
计划不是 `dual_pen_cograsp_plan/v1` ready barrier。`confirm` 和 `hold` 是纯反馈阶段，
永远不生成命令。实机 Mercury X1 使用另一套经过现场验证的适配器，本仿真 profile
不得复制到实机。
