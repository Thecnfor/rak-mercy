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

## ROS 1 导航资产（已从 GitHub 合并）

`maps/` 与 `scripts/goals_launcher.py` 是历史 ROS 1 Noetic 导航资产，包含已记录的地图、目标点和离线对比图。它们与 ROS 2 抓取系统独立，导入仅供版本保存、离线审阅与经现场负责人批准后的验证。

不要使用强制终止 ROS、硬件桥或串口占用进程的命令；运行导航或移动底盘前，须由现场负责人确认机器人状态、当前控制权和安全姿态。

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
