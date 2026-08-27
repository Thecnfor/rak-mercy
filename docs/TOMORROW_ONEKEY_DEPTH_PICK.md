# 明日一键部署：导航 + Deyes CUDA 深度 + YOLO + 固定桌面抓放

## 当前链路

`goal3_right → 固定头部姿态 → 双 IMX219 → 隔离版 OpenCV 4.8 CUDA StereoBM → 32FC1 深度 → YOLO 单笔检测 → 同时间戳深度准入 → 右臂固定 BaseCoords 抓取 → goal4_back → 放置`

当前快速双目标定是现场五姿态结果，`validated:false`。它只允许深度作为固定场景动作的准入证据，不提供正式手眼 TF；抓取 XY/Z 使用 2026-08-27 现场调出的固定 BaseCoords。正式标定门禁没有被绕过。

## 一键部署

Windows PowerShell（仓库根目录）：

```powershell
$env:ROBOT_IP="机器人IP"
$env:ROBOT_PASSWORD="SSH密码"
.\tools\deploy_competition_onekey.ps1 -StopExisting
```

该命令增量上传四个 ROS 2 包、配置和比赛脚本，检查 `/home/elephant/opencv-4.8.0-cuda` 的 CUDA 模块，使用该隔离 OpenCV 编译，然后只跑无动作 dry-run。若隔离版缺失，会从工作区外的受控 SHA-256 归档自动恢复；不会覆盖系统 OpenCV，也不会删除远端目录。

部署和 dry-run 成功后，人工清场、确认急停和右臂工作区，再执行：

```powershell
.\tools\deploy_competition_onekey.ps1 -StopExisting -Run
```

`-Run` 表示操作者明确允许：深度/YOLO 主链失败时，使用已调好的固定场地姿态退化。若比赛要求感知失败必须停机，直接在机器人执行：

```bash
ALLOW_DEGRADED=0 ~/scripts/race_onekey_try.sh
```

强制跳过视觉仅用于裁判/操作者明确指令：

```bash
FORCE_DEGRADED=1 ~/scripts/race_onekey_try.sh
```

## CUDA OpenCV 也属于项目

- C++ CUDA 几何源码：`Deyes/src/deyes_capture_cpp`；
- 固定版本、模块和 SHA-256：`depends/README.md`、`depends/ASSETS.md`；
- 能力预检：`depends/probe_opencv_cuda.sh`；
- 隔离恢复/源码构建：`depends/install_opencv_cuda_isolated.sh`。

二进制和源码压缩包体积大，不进入 Git。若目标机的隔离安装丢失，从 `E:/a_robot/temp/rak-mercy-baseline/offline-assets/depends/` 恢复清单中的归档到 `depends/`，再运行安装脚本。整个过程仅写 `/home/elephant/opencv-4.8.0-cuda`。

## 现场只检查四件事

1. 笔只有一支并位于已调好的右臂区域，桌面和相机支架未变化。
2. `/dev/right_arm` 无其他控制程序持有；导航和相机无重复节点。
3. 日志中的 `admission_mode.txt` 为 `depth_yolo_accepted`；若为 fallback，必须知道具体失败原因。
4. 抓取仍未获得“笔实际离桌”的视觉验证，首轮必须有人在急停旁观察，动作结束后再决定是否继续比赛运行。
