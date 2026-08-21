# Mercury X1 双臂低速联调边界

当前项目可从深度与抓取候选生成 `pre_grasp → approach → grasp → close → lift → retreat` **意图**，但尚未包含实机机械臂执行器。`mercury_arm_safety_contract.py` 是下一层适配器的唯一输入门：它只返回 dry-run 预览，绝不发布 `JointState`、调用 `pymycobot`、打开串口或发送 ROS action。

## 默认状态

- `dry_run: true`；任何 `request_execution: true` 直接拒绝为 `live_execution_not_implemented`。
- 未选择 `left/right`、未测量六个关节上下限、或未测量 `base_link` 工作空间时拒绝。
- 未来联调上限固定为 `5 deg/s`、`0.02 m/s`、`0.05 m/s²` 和 `20%` 夹爪力度；它们是上限，不是自动动作指令。
- 每个结果都含 `state`、`reason/failure_code`、限速和 `commands_emitted:false`。

## 上机前的安全顺序

1. 只运行 `motion_interface_probe`，确认实际 Turing 双臂、夹爪和状态反馈接口；不要向 `/joint_states` 发布。
2. 让每只机械臂独立、手持急停、低速示教，记录六个允许关节角和工具点在 `base_link` 的实际无碰撞包络。
3. 将测量结果复制为**仓库外**的现场 YAML（模板为 `config/stereo/mercury_arm_safety.defaults.yaml`）；不得把未验证参数提交为正式边界。
4. 先使用 contract 的 joint/cartesian/gripper dry-run 预览检查每个目标与限速；再由单独评审的、具备反馈/取消/超时/碰撞检查的执行适配器消费。

官方 `slider_control_turing` 是 GUI 到 vendor SDK 的桥接，不是安全动作服务；自动抓取禁止把它或 `sensor_msgs/JointState` 当作执行接口。

## 首个单关节最小增量试动（尚未执行）

只有以下条件全部成立，才允许将 dry-run 预览交给未来经过评审的实机适配器：机器人固定、工作区清空、急停可触达、仅一侧手臂上电、另一侧保持收纳、操作者保持 deadman、实际关节反馈新鲜、现场 YAML 已填入该臂六个实测限位，且适配器具备 cancel/timeout/result/碰撞检查和夹爪反馈。

预览接口（它不会发送任何动作）：

```python
from deyes_stereo.mercury_arm_safety_contract import MercuryArmSafetyProfile, build_single_joint_jog_preview

result = build_single_joint_jog_preview(
    current_positions_deg=feedback_from_adapter,  # 六轴实时反馈，禁止猜测
    joint_index=0, delta_deg=0.5, speed_deg_s=2.0,
    profile=measured_left_or_right_profile,
)
assert result["commands_emitted"] is False
```

该接口只接受 `joint_index=0..5`、单轴 `0 < |delta_deg| <= 1.0`、`speed <= 2 deg/s`；目标跨出现场关节限位即拒绝。真正试动必须由单独适配器二次确认 `dry_run=false` 的人工授权流程；本仓库当前没有该路径。

## 手眼标定多姿态采集接口

已有 `handeye_calibration` 接受多条相同物理特征的对应点，输出 `base_link_T_left_camera`。物理双目标定通过后，在不夹取任何物体时选择至少 6 个空间分散的棋盘格角点：每条记录含 `camera_point_m`（校正左目）与工具低速触达得到的 `base_point_m`。采样必须覆盖 X/Y/Z 和不同姿态，禁止全部取同一平面或同一视角。输入 JSON 的必要外层字段：

```json
{
  "calibration_id":"handeye-YYYYMMDD-01",
  "robot_id":"现场机器人编号",
  "camera_pair_id":"双目相机对编号",
  "stereo_calibration_id":"已验证双目标定 ID",
  "operator_confirmation":true,
  "correspondences":[
    {"camera_point_m":[0.0,0.0,0.0],"base_point_m":[0.0,0.0,0.0]}
  ]
}
```

结果仅在至少六点、RMS `<=5 mm`、P95 `<=10 mm`、身份一致且人工复核后才能作为 `pen_grasp` 的 `extrinsics_path`；采集文件与结果保存到仓库外 `temp/deyes/calibration/`。
