# Mercury X1 / RXAK 双臂机器人

本项目围绕 **Elephant Robotics Mercury X1**（及 RXAK 系列）双臂协作机器人展开。主开发方向为 ROS 2 双目视觉、抓取规划与安全门禁。

## 文档

工作区级开发资料统一保存在仓库外层的 `docs/`，避免规则材料、临时分析和正式源码混放。官方中文文档：https://docs.elephantrobotics.com/docs/Mercury_X1_cn/

## ROS 2 抓取系统

当前 ROS 2 系统优先完成：

- 实机硬件与 ROS 2 接口盘点
- 可取消、可反馈的双臂 Action 驱动
- Nav2、视觉定位与抓放状态机
- 安全互锁、异常恢复与全流程记录

任何物理抓取须遵守 `docs/EMERGENCY_HANDOFF_20260821_SINGLE_SHOT_PICK.md` 的标定、dry-run 与串口占用门禁。

## 导航—抓取统一入口

统一 dry-run 启动入口为：

```bash
ros2 launch deyes_bringup navigation_single_shot_pick.launch.py
```

它不会直接控制底盘；只有收到同一 `mission_id/nav_epoch` 的导航成功、
定位误差和连续静止证据后，才允许 Deyes 冻结一帧。事务身份会继续贯穿
深度、TF、规划和执行状态，失败后锁定，必须显式 reset 或重启。

当前底盘导航仍是 ROS 1 `move_base`。默认禁用的适配器
`scripts/pick_navigation_adapter_ros1.py` 通过 `ros1_bridge` 只向 ROS 2
提供可审计的导航证据；它不直接发布 `cmd_vel`、不自动重试，并要求现场
确认的目标白名单。部署步骤见
`Deyes/tools/navigation_integration_workflow.md`。

## ROS 1 导航资产（已从 GitHub 合并）

`maps/` 与 `scripts/goals_launcher.py` 是历史 ROS 1 Noetic 导航资产，包含已记录的地图、目标点和离线对比图。固定目标脚本不属于统一事务链，不得用于自动抓取；统一链只接受上述白名单适配器产生的到位证据。

不要使用强制终止 ROS、硬件桥或串口占用进程的命令；运行导航或移动底盘前，须由现场负责人确认机器人状态、当前控制权和安全姿态。

## 比赛现场 race-day 脚本（`scripts/`）

`scripts/` 现在还包含比赛当天 30 分钟冷启动流程所需的四个辅助脚本：

| 脚本 | 用途 |
|---|---|
| `start_competition_pipeline.sh` | 一键脚本，stage 0..6（`0` 清场，`1` ROS1 nav，`2` adapter，`3` ros1_bridge，`4` ROS2 Deyes，`5` T5 send_mission，`6` T6 race_monitor，`test` 单跑 Deyes dry-run） |
| `race_monitor.py` | 终端 6 健康面板：订阅 `/scan` `/amcl_pose` `/x1/pick/nav_mission` `/x1/pick/navigation_evidence`，subprocess 查 ROS 2 节点与 bridge topics。T1..T6 状态每秒刷新 |
| `send_mission.py` | Python 发 mission，pose 直接从 site_yaml allowlist 读保证 byte-exact；subscribe-before-publish + 二次 publish 避开临时 publisher 与常驻 subscriber 的 TCPROS 握手 race |
| `m2_calibrate.sh` | M2 物理双目标定（9×6 内角点 + 50 帧 capture + 3 项人工确认 compute）；候选 YAML 在 `/home/elephant/temp/deyes/calibration/<utc>/`，**不**自动入 `config/camera/` |

设计上 stage 号 = Terminal 号 = T 号（`./script.sh 5` = Terminal 5 = T5 = send_mission），现场口令无歧义。`start_competition_pipeline.sh help` 在机器人本机可看完整使用说明。赛前 Mac → X1 同步：

```bash
scp scripts/{start_competition_pipeline,race_monitor,send_mission,m2_calibrate}.sh \
    scripts/{pick_navigation_adapter_ros1,race_monitor,send_mission}.py \
    elephant@192.168.0.121:~/scripts/
```

## 目录结构

```
.
├── README.md
├── admin_gui/         # 独立管理前端与轻量后端
├── depends/           # 仅保留依赖说明、资产清单和校验值
├── maps/              # 已导入的 ROS 1 导航地图和对比图
├── scripts/           # 已导入的 ROS 1 导航脚本
└── Deyes/             # 双目视觉工程唯一源码根
    ├── src/           # 三个 ROS 2 包；colcon 的 base path
    ├── config/        # 可提交配置的唯一真源
    ├── test/          # 自动化与契约测试
    └── tools/         # 可复用部署、标定和验收工具
```

其中 `depends/README.md` 记录了 `Deyes` 在比赛现场机上恢复 CUDA OpenCV 环境所需的离线包和安装步骤。构建、日志、rosbag、标定原图和部署归档不得写入仓库，统一保存到工作区外层 `E:/a_robot/temp/`。

## License

待定。
