# Mercury X1 65 cm 比赛链路：答辩技术路线

## 1. 设计目标

系统让 Mercury X1 完成：导航到桌①、稳定观测、识别单支笔、计算抓取目标、右臂抓取与验证、导航到桌②、放置并撤离。

设计原则只有三条：

1. 实机测量是真值，仿真不能反向修改实机参数。
2. 正常比赛、固定点退化和展示续跑三种结果必须可区分，不能把“动作跑完”冒充“抓取成功”。
3. 导航、碰撞、串口、机器人反馈或运动失败立即停止；感知失败可以在明确的展示模式下继续动作展示。

## 2. 总体架构

```text
Windows 一键部署器
        │ SSH/文件与环境检查
        ▼
Jetson：Deyes ROS 2 感知链
双目采集 → 校正 CameraInfo → CUDA 深度 → 桌面平面
        → YOLO 单笔 → pen_feature
        │ 同一 exact stamp 冻结五路输入
        ▼
competition_pick_target_node
可信投影目标 / 固定点退化 / showcase synthetic target
        │ 一次 JSON 目标
        ▼
race_onekey_try.sh 事务编排器
ROS 1 goal3 → 头部 → pick → 抓起判定 → goal4 → place → summary
        │
        ▼
competition_transaction_result/v1 + trace.jsonl
```

ROS-free contract 负责校验输入和状态转换；ROS 节点只做消息适配；硬件脚本只在门禁通过后发送单次命令。这样同一业务规则可以被 fixture、仿真和实机复用。

## 3. 实机真值与坐标路线

| 项目 | 固定值 |
|---|---|
| 桌高 | 650 mm |
| 头部角度 | `[-54.93, 2.63]°` |
| 末端姿态 | `[179.99, -12, 0]°` |
| 抓取 Z | `235 → 180 → 140 → 135 → 180 → 235 mm` |
| 放置 Z | `200 → 165 → 200 → 260 mm` |
| 夹爪 | 开 70，闭 0 |
| 运输位姿 | `[300,10,260,179.99,-12,0]` |

560 mm 桌面升级到 650 mm 后，相机到平面的距离是：

```text
0.559428925 - 0.090 = 0.469428925 m
```

目标生成使用同一时间戳的检测、笔轴特征、32FC1 深度、校正 CameraInfo 和桌面平面。健康平面与 650 mm 偏差超过 25 mm 时硬停止；平面缺失或低质量时使用固定桌高并标记 `unverified`。

当前触点 PnP 的 RMS 为 4.1917 px，大于 4 px，因此 `usable:false`。随机 XY 正式抓取不会放行。现场若未重标定，只能由操作员把笔放在 `[400,10] mm` 标记点并显式采用固定点退化路径。

## 4. 运行状态机

```text
部署预检
  → 导航 goal3 到桌①
  → 到位稳定
  → 头部到固定角度并确认反馈
  → 启动感知并冻结一次 exact-stamp 快照
  → 生成一次目标
  → IK/FK/限位/碰撞门禁
  → 开夹、下降、闭夹、抬升、到运输位姿
  → 抓起判定
  → 导航 goal4 到桌②
  → 下降、开夹、抬升撤离
  → 写事务结果与 trace
```

抓起判定保留两个事实来源：笔相对桌面抬升至少 30 mm；或原 ROI 连续三帧无笔且夹爪反馈相对空夹基线变化至少 5。正常比赛模式只有验证成功才得到 `competition_success:true`。

## 5. 三种执行分支

### 正常比赛路径

真实感知产生可信 target，运动完成且抓起验证通过，然后执行 goal4 和 place。终态为：

```json
{"showcase_complete": true, "competition_success": true, "object_grasp_verified": true}
```

### 固定点退化路径

PnP 不可用时，操作员把笔放在 `[400,10] mm`，显式启用固定点。日志必须含 `degraded:true`，不声称随机位置能力。

### 展示续跑路径

默认 `-Run` 在部署预检通过后，若运行时感知拒绝或物体未验证，可用明确的 synthetic 固定展示 target 继续抓取动作、运输、goal4 和空放置。动作跑完返回：

```json
{"showcase_complete": true, "competition_success": false, "object_grasp_verified": false}
```

它不会改写 `navigation_permitted`、PnP trust 或物体抓取事实。`-StrictResultGates` 可恢复严格比赛行为。

## 6. 不可绕过的停止条件

- 部署 dry-run、ONNX SHA、TensorRT binding 或 OpenCV 隔离失败。
- goal3/goal4 导航失败，健康平面偏差超过 25 mm。
- 碰撞净空未验证，串口、上电、机器人状态、夹爪或位姿反馈失败。
- 运动命令失败或 target JSON 与实际 XY 不一致。
- 未知配置错误或 target 节点启动失败。

所有动作零自动重试。已发送的命令必须在结果 JSON 中留下 `commands_emitted:true`，避免“部分动作发生但记录显示未执行”。

## 7. 模型、部署与运动门禁

- ONNX SHA256 固定为 `8916cbf25949d6e8b03c01e6ca1c7871aeac0ad105c931f1dd9881cb5d5a4c4e`。
- TensorRT engine 必须由该 ONNX 在目标 Jetson 生成，并保存版本、CUDA、输入输出和 engine SHA sidecar。
- OpenCV CUDA 使用隔离前缀，不覆盖系统库；现场用 `ldd` 与 ROS topic 频率确认实际加载路径。
- 六轴模型采用 9-link mask、6 个 active links；运输放行同时要求 IK/FK 残差、关节限位和正碰撞净空证据。

## 8. 验证结论

| 层级 | 结果 | 说明 |
|---|---|---|
| A 代码/fixture | 通过 | 最新集成与仿真合并后 `447 passed` |
| B Isaac physics/ROS | 组件通过 | 60 Hz，`dt=1/60s`，timeline 推进，domain 46，无真实设备 |
| C synthetic 全链 | 通过 | 单笔、导航状态机、+60 mm 抬升、桌②放置；抓取 attachment 明确为 synthetic |
| D Jetson/实机 | 待现场 | 碰撞净空、Jetson 依赖/topic、PnP 或固定标记、一次真实全链 trace |

结论是“可发布独立候选分支”，不是“实机比赛已经成功”。Isaac 的完整抓放不能证明真实接触抓取，synthetic 感知也不能证明现场视觉。

## 9. 明日现场最短流程

1. 先测量并审核正碰撞净空，更新 profile；当前 `collision_clearance_validated:false`、保守净空 0 mm 会在任何机械动作前停止。
2. 在 Windows 终端设置机器人连接信息并执行默认展示链：

```powershell
$env:ROBOT_IP="机器人IP"
$env:ROBOT_PASSWORD="SSH密码"
.\tools\deploy_competition_onekey.ps1 -StopExisting -Run
```

3. 若要严格比赛结果门禁：

```powershell
.\tools\deploy_competition_onekey.ps1 -StopExisting -Run -StrictResultGates
```

4. 若 PnP 仍不可用，人工把笔放到 `[400,10] mm` 标记点；正常随机位置能力必须等重新标定后再启用。

现场判读只看两个终态：`SHOWCASE COMPLETE` 表示动作链跑完；`COMPETITION SUCCESS` 才表示抓笔、运输和放置结果被验证。

## 10. 答辩总结

本方案的核心不是单个识别模型，而是“可追溯的一次性事务”：把双目感知、坐标投影、六轴运动和导航串成同一状态机，并用双状态结果区分工程展示与比赛成功。它既能在感知不理想时完成可见展示，又不会掩盖 PnP、碰撞净空或真实抓取尚未验收的事实。
