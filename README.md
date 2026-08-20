# Mercury X1 / RXAK 双臂机器人

本项目围绕 **Elephant Robotics Mercury X1**（及 RXAK 系列）双臂人形协作机器人展开。

## 简介

**Mercury X1** 是一款基于 ROS 的双臂协作机器人，配备两套六轴机械臂（A/B 型），采用模块化结构设计，可用于科研、教学与产业开发。RXAK 为其双臂机器人平台的相关型号/配置。

## 文档

工作区级开发资料统一保存在仓库外层的 `docs/`，避免规则材料、临时分析和正式源码混放。官方中文文档：https://docs.elephantrobotics.com/docs/Mercury_X1_cn/

## 快速开始

当前开发方向为 ROS 2 六轴双臂实机，优先完成：

- 实机硬件与 ROS 2 接口盘点
- 可取消、可反馈的双臂 Action 驱动
- Nav2、视觉定位与抓放状态机
- 安全互锁、异常恢复与全流程记录

## 目录结构

```
.
├── README.md
├── admin_gui/         # 独立管理前端与轻量后端
├── depends/           # 仅保留依赖说明、资产清单和校验值
└── Deyes/             # 双目视觉工程唯一源码根
    ├── src/           # 三个 ROS 2 包；colcon 的 base path
    ├── config/        # 可提交配置的唯一真源
    ├── test/          # 自动化与契约测试
    └── tools/         # 可复用部署、标定和验收工具
```

其中 `depends/README.md` 记录了 `Deyes` 在比赛现场机上恢复 CUDA OpenCV 环境所需的离线包和安装步骤。
构建、日志、rosbag、标定原图和部署归档不得写入仓库，统一保存到工作区外层 `E:/a_robot/temp/`。

## License

待定。

---

更多信息请参考 [Elephant Robotics 官方文档](https://docs.elephantrobotics.com/docs/Mercury_X1_cn/)。
