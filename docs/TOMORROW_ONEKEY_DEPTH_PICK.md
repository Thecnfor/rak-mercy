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

部署在 Jetson 上用 `trtexec --fp16` 生成或复用本机 engine。`*.manifest.json` 记录 ONNX/engine SHA、TensorRT、CUDA、aarch64、`[1,3,416,416]` 和 `yolov5:[1,N,5+C]`。runner 同时校验 sidecar 与 engine 文件；不会把运行时自算 SHA 当作部署预期值。

随后以隔离 OpenCV 构建，并检查 `ldd cuda_stereo_depth_node` 只指向 `$HOME/opencv-4.8.0-cuda`。部署 dry-run 会实际启动 C++ capture、CUDA depth、ground plane、YOLO 和 pen feature 30 秒，不执行机械臂；要求 depth 不低于 12 Hz、配对 P95 skew 不大于 10 ms。

dry-run 通过后可直接全自动执行：

```powershell
.\tools\deploy_competition_onekey.ps1 -StopExisting -Run
```

`-Run` 不再隐式设置任何 `ALLOW_DEGRADED`，也不需要人工 token。默认 `FIXED_TABLE_HEIGHT_MM=650`、`ALLOW_BBOX_CENTER=1`、`ALLOW_FIXED_XY_FALLBACK=0`、`FORCE_FIXED_TARGET=0`。ground plane 缺失或质量差时按已选择的固定 650 mm 继续并记录 `fixed_height_unverified`；健康平面只要与 650 mm 偏差超过 25 mm 就立即停止。

只有操作者明确允许固定 XY 时才加：

```powershell
.\tools\deploy_competition_onekey.ps1 -StopExisting -Run -AllowFixedXyFallback
```

强制 `[400,10]` mm 还需额外明确指定（该选项必须与上一个选项同时使用）：

```powershell
.\tools\deploy_competition_onekey.ps1 -StopExisting -Run -AllowFixedXyFallback -ForceFixedTarget
```

## 单事务顺序

`ROS 1 goal3_right → 头部反馈 → ROS 2 C++ capture + CUDA depth + ground plane + YOLO + pen feature + competition target node → topic snapshot adapter 得到 competition_pick_target/v1 单目标 JSON → pick → success=true 且 navigation_permitted=true → 关闭视觉 → ROS 1 goal4_back → competition_venue_65cm.yaml place`

视觉必须保持到抓起验证完成。导航、桌高冲突、串口、机械臂反馈、target 或抓起验证任一失败都会停止，且不自动重试。ROS 1/ROS 2 每次切换都会清理另一套环境并重新 source。

每次比赛日志位于 `~/temp/deyes/competition/<transaction_id>/`，至少包含 `target.json`、`admission.json`、`engine_manifest.json`、`engine_validation.json`、`grasp_verification.json`、`trace.jsonl` 和各步骤日志。

## 故障矩阵

| 故障 | 行为 | 机械臂/导航后续 |
|---|---|---|
| ONNX、engine 或 sidecar SHA/ABI 不符 | 部署或 runner 停止 | 不动作 |
| CUDA/OpenCV 模块、aarch64/7.2 架构或 `ldd` 隔离检查失败 | 部署停止 | 不动作 |
| dry-run depth < 12 Hz 或 skew > 10 ms | 部署停止 | 不动作 |
| goal3、头部命令或反馈失败 | runner 停止 | 不抓取 |
| ground plane 缺失/质量差 | 固定 650 mm 继续，记录 `fixed_height_unverified` | 仅按 target 合同继续 |
| 健康 ground plane 与 650 mm 偏差 > 25 mm | runner 停止 | 不抓取 |
| bbox center 不可用 | target 停止，除非实现允许的其他正常路径 | 不抓取 |
| 固定 XY 未显式允许 | target 停止 | 不抓取 |
| 固定 XY 已允许但未 force | 仅 target 合同判定退化 | 不会自动改为 `[400,10]` |
| pick 串口/反馈失败，或 GraspVerifier 未同时给出 `success=true`、`navigation_permitted=true` | runner 停止，视觉仍由清理流程关闭 | 不去 goal4、不 place |
| goal4 或 place 串口/反馈失败 | runner 停止 | 不重试 |

`-StopExisting` 只向明确命名的旧 Deyes 视觉/target 节点发送 `SIGINT`。部署不删除远端目录、不覆盖系统 OpenCV，也不操作机械臂。

## 现场待验

以下项目不能在 ROS-free 开发机伪造通过，必须在 Jetson/机器人现场确认：

1. Jetson TensorRT 能从固定 ONNX 构建 FP16 engine，sidecar 版本与设备实际一致。
2. CUDA OpenCV probe 和构建后 `ldd` 均指向隔离前缀。
3. 30 秒视觉 dry-run 的 depth ≥ 12 Hz、pair P95 skew ≤ 10 ms。
4. 当前 `competition_pick_target_node.py` 的 `waiting_for_exact_stamp_projector_adapter` 仍是占位结果；snapshot adapter 会明确拒绝它，绝不伪造 target 或声称全链已通过。主分支必须补齐 ROS live projector/target adapter，使 `/x1/competition/pick_target` 发布 `valid=true`、`trusted_for_venue_execution=true` 的 `competition_pick_target/v1`，再做现场执行。
5. 正式 profile 为 `competition_venue_65cm.yaml`；pick 使用 `--x-mm/--y-mm/--venue-profile/--result-json`，place 使用 `--venue-profile`。GraspVerifier 必须同时输出 `success=true` 与 `navigation_permitted=true` 才能去 goal4。
6. goal3/goal4、头部、右臂串口反馈与急停现场可用；真实抓起能输出 `success=true` 与 `navigation_permitted=true`。
