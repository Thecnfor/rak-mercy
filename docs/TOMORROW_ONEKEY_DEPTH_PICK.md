# 65 cm 场地一键部署与比赛执行

## 默认命令

在 Windows 仓库根目录设置连接信息后部署：

```powershell
$env:ROBOT_IP="机器人IP"
$env:ROBOT_PASSWORD="SSH密码"
.\tools\deploy_competition_onekey.ps1 -StopExisting
```

部署会增量上传四个 ROS 2 包（`deyes_interfaces`、`deyes_capture_cpp`、`deyes_stereo`、`deyes_bringup`）、完整配置与 `scripts`，以及仓库模型
`Deyes/models/pen/pen_student_01875_416_v1.onnx`。ONNX 必须匹配固定 SHA-256：
`8916cbf25949d6e8b03c01e6ca1c7871aeac0ad105c931f1dd9881cb5d5a4c4e`。

部署在 Jetson 上用 `trtexec --fp16` 生成或复用本机 engine。部署器会用当前 TensorRT runtime 反序列化 engine（兼容 TensorRT 8 binding API 与 TensorRT 10 tensor API），把每个 binding 的实际 index、name、input/output、shape、dtype 写入 `*.manifest.json`，并记录 ONNX/engine SHA、TensorRT、CUDA、aarch64、`[1,3,416,416]` 和 `yolov5:[1,N,5+C]`。只有 TensorRT、CUDA、架构、模型链及反序列化后的实际 binding ABI 均与 sidecar 一致才复用。runner 同时校验 sidecar、binding ABI 与 engine 文件；不会把运行时自算 SHA 当作部署预期值。

随后以隔离 OpenCV 构建，并以解析后的真实路径检查 `ldd cuda_stereo_depth_node` 的全部 OpenCV 依赖都位于 `$HOME/opencv-4.8.0-cuda`。部署 dry-run 会实际启动 C++ capture、CUDA depth、ground plane、YOLO 和 pen feature 30 秒，不执行机械臂；要求 depth 不低于 12 Hz、配对 P95 skew 不大于 10 ms，并实际等待和验证 rectified CameraInfo、YOLO boxes/status（包含 engine/model SHA 链）、ground plane/status、pen feature/status。任一 topic 缺失或契约不符都会停止，验证摘要写入 `deploy_vision_dry_run/vision_contract.json`。

dry-run 通过后，一键入口可继续做完整的生产门禁检查：

```powershell
.\tools\deploy_competition_onekey.ps1 -StopExisting -Run
```

`-Run` 默认启用比赛展示续跑。部署预检、goal3/goal4、头部反馈、碰撞净空、串口、机器人状态和位姿反馈仍是不可绕过的硬门禁；仅运行时相机/CUDA/YOLO/target 感知失败或抓起未验证时，改用独立的 `competition_showcase_target/v1` 在 `[400,10]` mm 完成空抓、运输位姿、goal4 与放置展示。该 target 明确标记 `sensor_target_available:false`、`synthetic_target:true`，不冒充视觉 target，也不会把 `navigation_permitted` 改成 true。

需要恢复原有“抓取未验证即停止”行为时使用：

```powershell
.\tools\deploy_competition_onekey.ps1 -StopExisting -Run -StrictResultGates
```

当前默认场地 profile **不会发出 Mercury 命令**：官方 URDF 的六轴 IK、FK、关节限位证据独立通过，但现有 5 mm 只是名义 TCP Z 差，考虑 5 mm 反馈容差后的保守净空为 0 mm，且未验证工具/连杆/扫掠体碰撞。因此 `kinematics_validated:true`、`collision_clearance_validated:false`、`transport_validated:false`。生产 pick/place 会在构造 Mercury、上电、夹爪或移动之前停止；两者的 `--dry-run` 仍会输出 `commands_emitted:false` 的计划。只有现场形成可审计的正保守净空/碰撞包络证据并更新场地 profile 后，才能解除该门禁；不得把名义 TCP Z 差改称碰撞净空。

`-Run` 不再隐式设置任何 `ALLOW_DEGRADED`，也不需要人工 token。默认 `FIXED_TABLE_HEIGHT_MM=650`、`ALLOW_BBOX_CENTER=0`、`ALLOW_FIXED_XY_FALLBACK=0`、`FORCE_FIXED_TARGET=0`，正式视觉路径必须使用单笔 YOLO 与 `pen_feature` 轴中点，不默认绕过 feature。展示续跑的 synthetic target 是单独结果类型，不改变正式 target 合同。ground plane 缺失或质量差时按已选择的固定 650 mm 继续并记录 `fixed_height_unverified`；健康平面只要与 650 mm 偏差超过 25 mm 就立即停止。

只有人工已把笔放到场地 `[400,10]` mm 标记点，才允许同时指定下面两个选项。单独指定任一选项都会在部署器/runner 入口直接拒绝：

```powershell
.\tools\deploy_competition_onekey.ps1 -StopExisting -Run -AllowFixedXyFallback -ForceFixedTarget
```

## 单事务顺序

`ROS 1 goal3_right → 头部反馈 → ROS 2 C++ capture + CUDA depth + ground plane + YOLO + pen feature + competition target node → 正常 competition target 或运行时感知失败后的 synthetic [400,10] mm showcase target → 单次 pick → 运输位姿 → 抓起验证结果 → 正常成功或展示续跑决策 → ROS 1 goal4_back → place → competition_transaction_result/v1`

视觉可用时必须保持到抓起验证完成。运行时视觉/target 或物体抓起验证失败在默认展示模式下记录 degraded 后继续；导航、健康桌高冲突、碰撞净空、串口、机器人/夹爪反馈和运动位姿失败仍停止，且所有路径都不自动重试。ROS 1/ROS 2 每次切换都会清理另一套环境并重新 source；切回 ROS 1 后，runner 会显式恢复 colcon 安装中的 `deyes_stereo` Python 路径供 place 使用，不依赖残留的 ROS 2 环境。

每次比赛日志位于 `~/temp/deyes/competition/<transaction_id>/`，至少包含正常 `target.json` 或 `showcase_target.json`、`admission.json`、`engine_manifest.json`、`engine_validation.json`、`grasp_verification.json`、`pick_decision.json`、`place.json`、`transaction_result.json`、`trace.jsonl` 和各步骤日志。展示完整但物体未验证时命令退出 0，同时 `competition_success:false`、`showcase_complete:true`，终端打印醒目的 `SHOWCASE COMPLETE` 和 `COMPETITION SUCCESS: false`。

## 故障矩阵

| 故障 | 行为 | 机械臂/导航后续 |
|---|---|---|
| ONNX、engine、sidecar SHA 或实际 TensorRT binding ABI 不符 | 部署或 runner 停止；不复用旧 engine | 不动作 |
| CUDA/OpenCV 模块、aarch64/7.2 架构或 `ldd` 隔离检查失败 | 部署停止 | 不动作 |
| dry-run depth < 12 Hz、skew > 10 ms，或 CameraInfo/YOLO/ground plane/pen feature 任一状态/输出 topic 缺失或契约不符 | 部署停止 | 不动作 |
| goal3、头部命令或反馈失败 | runner 硬停止 | 不抓取 |
| YOLO 为零笔、多笔、ambiguous、未完成，或未明确允许自动抓取 | 严格模式停止；默认展示模式生成独立 synthetic `[400,10]` target | 展示空抓、运输、goal4、空放置 |
| detection、pen feature、32FC1 depth、rectified CameraInfo、ground plane stamp 不完全相同，或 depth 不是 `32FC1` | 严格模式停止；默认展示模式 fixed-marker 续跑 | 不宣称感知成功 |
| 触点 PnP `RMS=4.1917 px > 4 px`、`usable:false` | 正常随机 XY 仍 fail-closed；默认展示模式使用独立 synthetic target | 不伪造 projector/competition success |
| ground plane 缺失/质量差 | 固定 650 mm 继续，记录 `fixed_height_unverified` | 仅按 target 合同继续 |
| 健康 ground plane 与 650 mm 偏差 > 25 mm | runner 停止 | 不抓取 |
| IK/FK/关节限位通过，但碰撞净空未验证或保守净空 ≤ 0 mm | pick/place 在 Mercury 构造/上电前停止；dry-run 仍输出计划 | 不碰串口、不发夹爪/移动命令 |
| bbox center 不可用 | target 停止，除非实现允许的其他正常路径 | 不抓取 |
| 固定 XY 未显式允许 | target 停止 | 不抓取 |
| 固定 XY 与 force 未同时启用 | 入口立即停止 | 不会自动改为 `[400,10]` |
| 固定 XY 已强制，但 target 不是 `selection_source=fixed_xy_fallback`、XY 不是 `[400,10]` mm、或无同 stamp 观测像素 | runner 停止 | 不抓取或不放行抓取验证 |
| pick 串口、机器人状态、位姿或稳定夹爪反馈失败 | runner 硬停止 | 不去 goal4、不 place |
| pick 动作及运输位姿完成，但 ROI 感知不可用或未证明抓到物体 | 严格模式停止；默认展示模式保持 `navigation_permitted:false` 并用独立 showcase 决策续跑 | goal4、空放置，最终 competition=false/showcase=true |
| goal4、place 串口/反馈失败或 `place.json` 不成功 | runner 停止 | 不重复执行 |

`-StopExisting` 只向明确命名的旧 Deyes 视觉/target 节点发送 `SIGINT`。部署不删除远端目录、不覆盖系统 OpenCV，也不操作机械臂。

## 现场待验

以下项目不能在 ROS-free 开发机伪造通过，必须在 Jetson/机器人现场确认：

1. Jetson TensorRT 能从固定 ONNX 构建 FP16 engine，sidecar 版本及枚举出的实际 binding ABI 与设备 engine 一致。
2. CUDA OpenCV probe 和构建后 `ldd` 均指向隔离前缀。
3. 30 秒视觉 dry-run 的 depth ≥ 12 Hz、pair P95 skew ≤ 10 ms，且 `vision_contract.json` 包含 CameraInfo、YOLO engine/model 链、ground plane 与 pen feature 的真实 topic 证据。
4. `competition_pick_target_node.py` 的严格同 stamp live adapter 已实现；必须在 Jetson 上验证 detection、pen feature、32FC1 depth、rectified CameraInfo 与 ground plane 的真实 QoS/时间戳能够冻结成一次 `competition_pick_target/v1`。占位、超时或拒绝 payload 仍由 snapshot adapter fail-closed。
5. 正式 profile 为 `competition_venue_65cm.yaml`；相机校正使用 `venue_20260827_quick_stereo.yaml`，触点投影器必须单独使用 `venue_20260827_touch_projector.yaml`，不得混用。当前投影器 RMS `4.1917 px > 4 px`，必须保持 `usable:false`；正常随机 XY 因而 fail-closed。
6. 正常 pick 使用 `--target-json` 与 live feedback；synthetic 展示 pick 使用独立 `--showcase-target-json`。GraspVerifier 只有同时输出 `success=true` 与 `navigation_permitted=true` 才能形成 `competition_success:true`；默认展示续跑只能通过 `continue_showcase` 去 goal4，且保持物体未验证事实。place 需写出动作成功、`object_state` 与 `object_delivery_verified` 一致的 `competition_place_execution/v1`。
7. goal3/goal4、头部、右臂串口反馈与急停现场可用；真实抓起能输出 `success=true` 与 `navigation_permitted=true`。
8. 对运输位姿及完整抓取/放置扫掠路径完成工具、连杆、桌面与自碰撞的现场/全几何净空验证，得到大于 0 mm 的保守下界；当前该项未通过，属于 Tier D 解锁项，默认 profile 保持 fail-closed。
