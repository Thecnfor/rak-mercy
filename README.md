# Mercury X1 — 决赛 30 分钟作战手册

**队伍**: ROBOTAC
**比赛日期**: 2026-08-21
**机器人**: Elephant Robotics Mercury X1, ROS1 Noetic

> **仅供参考**。本手册基于 2026-08-19/20 两天调试实战,记录了**已验证可跑通**的最小流程。现场若有变动,以机器人和实际环境为准。

---

## 0. 30 秒速览

```bash
# 在 X1 终端
source ~/mercury_x1_ros/devel/setup.bash
roslaunch turn_on_mercury_robot navigation.launch
# RViz 弹出 → 给一次 2D Pose Estimate → 跑脚本:
python3 ~/goals_launcher.py
```

机器人自动 1→3→4 走完。

---

## 1. SSH 连接

```bash
# 用户名 elephant, 密码 Elephant
ssh elephant@<X1_IP># Mac 装了 sshpass 的话:
sshpass -p 'Elephant' ssh elephant@<X1_IP>
```

> 现场机器 IP 由工作人员分配。`ifconfig` 或触摸屏 Terminal 可查。

---

## 2. 验证 ROS1 环境

```bash
source ~/mercury_x1_ros/devel/setup.bash
echo $ROS_DISTRO  # 必须输出 noetic
```

---

## 3. 杀其他团队的 ROS2 残留(多团队共享机器人)

```bash
for k in roslaunch roscore rosmaster mercury_robot_node lslidar_driver \
         gmapping slam_gmapping keyboard_teleop turtlebot_teleop \
         slider_control rviz amcl move_base map_server \
         _ros2_daemon hardware_bridge robot_pose_ekf robot_state_publisher \
         static_transform_publisher; do
  pkill -9 -f "$k" 2>/dev/null
done
sleep 2
ps -ef | grep -E "ros|mercury|hardware" | grep -v grep | wc -l
# 必须输出 0
```

---

## 4. 修复 USB 串口(USB 重新枚举后可能丢 symlink)

```bash
for i in 0 1 2 3; do
  d=/dev/ttyACM$i
  [ -e $d ] && echo "$d $(udevadm info $d | grep ID_SERIAL_SHORT | head -1)"
done
# 雷达序列号 0001, STM32 序列号 0004
# 重建 symlink:
sudo ln -sf /dev/ttyACM0 /dev/wheeltec_lidar
sudo ln -sf /dev/ttyACM1 /dev/wheeltec_controller
```

---

## 5. 决赛现场启动(主流程,30 分钟)

### 5.1 终端 1:启动 navigation

```bash
source ~/mercury_x1_ros/devel/setup.bash
roslaunch turn_on_mercury_robot navigation.launch
```

**预期**(等 5-8 秒):
- 终端打印节点启动
- RViz 窗口自动弹出
- RViz 显示我们的地图 (192×160 像素, 9.6m × 8m)
- 一个机器人 marker 出现

如果 RViz 没弹:
```bash
rosrun rviz rviz -d $(rospack find turn_on_mercury_robot)/rviz/CobotX.rviz
```

### 5.2 加载 RViz 配置

> `File → Open Config → ~/mercury_x1_ros/src/turn_on_mercury_robot/rviz/xrak_navigation.rviz`

### 5.3 添加 RobotModel display

> 左下角 Displays → Add → RobotModel → 描述 topic: `/robot_description`

显示真正的 3D 机器人模型,跟着 amcl 实时更新。

### 5.4 2D Pose Estimate(关键!只一次)

> 工具栏 → `2D Pose Estimate` 按钮 → 在地图上机器人**实际位置**点 + 按住左键拖箭头 → 松手

**预期**: 红色激光点云**啪**地贴住墙。

### 5.5 终端 2:跑脚本

```bash
source ~/mercury_x1_ros/devel/setup.bash
python3 ~/goals_launcher.py
```

**预期 log**:
```
[INFO] waiting for move_base ...
[INFO] [goal1_start] try 1/3 -> (0.030,-0.033)
[INFO] [goal1_start] OK
[INFO] [goal3_right] try 1/3 -> (2.733,-1.939)
[INFO] [goal3_right] OK
[INFO] [goal4_back] try 1/3 -> (2.640,-1.970)
[INFO] [goal4_back] OK
[INFO] all goals done
```

机器人依次走 3 个点。

---

## 6. 调试指南

### 6.1 启动后 /scan 没数据 / RViz 没激光

```bash
ls -la /dev/wheeltec_*
# 错了重建:
sudo ln -sf /dev/ttyACM0 /dev/wheeltec_lidar
sudo ln -sf /dev/ttyACM1 /dev/wheeltec_controller
```

### 6.2 2D Pose Estimate 无效

amcl 默认不接收 /initialpose,需要修复:

```bash
rosparam set /amcl/set_initial_pose true
rosservice call /amcl/set_parameters \
  "config: {doubles: [], ints: [], strs: [], bools: [{name: set_initial_pose, value: true}]}" 2>&1
rosservice call /global_localization
sleep 1
# 再用 RViz 工具栏 2D Pose Estimate
```

### 6.3 机器人不走 / /cmd_vel 空

```bash
rostopic hz /cmd_vel  # 应 5-20 Hz
rostopic hz /odom     # 应 30-50 Hz
# STM32 死了(bootloader 异常,黄绿 RGB):急停松开、电源重启
```

### 6.4 走走停停(DWA recovery)

DWA 局部规划器遇到未建图区会触发 recovery(原地转 360°)。正常现象,不严重可忽略。频繁卡时让机器人贴墙走获取 scan-match 数据。

### 6.5 目标点要修改

```bash
nano ~/goals_launcher.py
# 改 GOALS 列表的 (x, y, z, w)
# z, w 是四元数,采法:
#   键盘控制走到目标位置 → RViz Add → TF → base_link → 抄 Position X/Y + Orientation Z/W
```

---

## 7. 随机障碍物避障

> **navigation 默认已支持随机障碍物避障**,无需额外代码。

**原理**:
- 局部规划器 DWA 每 0.1 秒重规划一次
- 雷达每秒扫描 10 次,实时更新 costmap
- 新检测到的障碍物自动加入 costmap,DWA 自动绕开
- 比赛方随机放的箱子/椅子 → 机器人实时看到 → 实时绕行

**急停脚本(可选,作为最后安全网)**: DWA 反应时间约 0.1 秒,极端情况(车前 < 0.3m 突然出现)反应可能不够。可以加一个监听 /scan 的 Python,前向距离 < 0.3m 立刻发 /cmd_vel=0。决赛现场如果觉得 DWA 不够再加。

---

## 8. 关键文件清单(全部 git tracked)

| 文件 | 作用 |
|---|---|
| `maps/team_rak_20260820.{pgm,yaml}` | 我们自己的图(192×160) |
| `maps/x1_competition_left_only_map.{png,yaml}` | 别家的图(下下策) |
| `scripts/goals_launcher.py` | **比赛直接用** |
| `maps/map_comparison.png` + `maps/map_3way_comparison.png` | 三方图对比 |

X1 端(不在 git):
- `~/mercury_x1_ros/src/turn_on_mercury_robot/rviz/xrak_navigation.rviz` — RViz 配置
- `~/mercury_x1_ros/src/turn_on_mercury_robot/launch/navigation.launch` — 已指向 team_rak_20260820

---

## 9. 比赛流程时间线(30 分钟)

| 时间 | 动作 |
|---|---|
| T+0-1min | SSH 上 X1,杀进程,验证 ROS1 环境 |
| T+1-3min | 启动 navigation.launch |
| T+3-4min | RViz 加载 xrak_navigation.rviz,加 RobotModel |
| T+4-5min | 2D Pose Estimate 一次(机器人放回起点) |
| T+5-25min | 跑 `python3 ~/goals_launcher.py`,机器人自动走完 |
| T+25-30min | 收尾、评分、备份 |

---

## 10. 经验总结

### 10.1 为什么不用 RViz MultiNaviGoalsPanel

- 目标点列表**不持久化**:每次重启 RViz 都要重新设点
- 比赛现场如果 RViz 崩溃,目标点全丢

**改用 Python 脚本直接发 move_base goal**,目标点写在文件里持久化,比赛现场可改。

### 10.2 为什么删了 goal_2

move_base 每个点都"必须完全到达" → 中转点必然有停顿(0.5-1 秒)。如果中转点不需要必到,直接删除。

### 10.3 速度参数 0.15/0.4

gmapping 在 5-15 Hz 雷达下容忍速度上限约 0.3 m/s。0.15 m/s 室内建图最优。0.4 rad/s ≈ 23°/s,转 90° 约 4 秒,既不太慢也不打滑。

### 10.4 关键突破(为什么这 4 天这么难)

1. **amcl 默认 set_initial_pose=false** → 2D Pose Estimate 无效 → 必须手动 `rosparam set`
2. **MultiNaviGoalsPanel 不持久化** → 改用 Python 脚本
3. **STM32 bootloader 异常** → 黄绿 RGB、机器人不动 → 充电 + 物理重启
4. **USB 重新枚举** → symlink 会丢,需要重建

---

## 11. 应急联系清单

> 仅供现场参考。

| 问题 | 解决 |
|---|---|
| SSH 连不上 | 等 10 秒重试 |
| 机器人不动 | 看急停、RGB 灯黄绿 = 物理重启 |
| 全部不工作 | Ctrl+C navigation 重启一次 |