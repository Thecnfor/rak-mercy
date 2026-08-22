# Mercury X1 单帧右臂抓笔闭环

## 行为边界

`single_shot_pick.launch.py` 每次启动最多生成一个事务：稳定检测、冻结同时间戳图像/深度/CameraInfo/桌面平面、单次 YOLO、完整抓取几何 TF2 转换、计划，然后在门禁允许时调用右臂 Action。零支或多支笔、不同时间戳、缺少状态、串口占用或身份不一致都会终止，不会自动重试。

默认 `dry_run=true`、`enable_live_execution=false`。当前 `8x7` 物理标定报告为 `validated:false`，不得用于动作放行。

## 构建和 dry-run

```bash
cd ~/rak-mercy/Deyes
source /opt/ros/galactic/setup.bash
colcon build --symlink-install --packages-up-to deyes_bringup
source install/setup.bash

export ROBOT_IP=192.168.255.121
ros2 launch deyes_bringup single_shot_pick.launch.py \
  model_path:=/absolute/path/pencil.engine \
  model_id:=pencil-yolov8 \
  model_sha256:=<sha256> \
  log_root:=/home/elephant/temp/deyes/single_shot_pick
```

预期 `/x1/pick/execution_status` 终止于 `dry_run_complete`，且 `hardware_commands_emitted:false`。人工重新采样使用：

```bash
ros2 service call /x1/pick/reset std_srvs/srv/Trigger '{}'
```

## Live 放行材料

Live 启动前必须同时具备：

- `640x360` 双目标定 `validated:true`，RMS 与极线 P95 均不超过 `0.50 px`；
- 手眼外参 `validated:true`，RMS 不超过 `5 mm`、P95 不超过 `10 mm`；
- 仓库外右臂现场 YAML，由模板 `right_arm_execution.site.template.yaml` 填写实测六关节限位、工作空间、夹爪方向和姿态约定；
- `/odom`、右臂反馈、双目配对诊断均持续新鲜；
- `/dev/right_arm` 没有其他持有者，尤其不得同时运行 `slider_control_turing`。

只有上述材料完成后才允许显式传入：

```bash
ros2 launch deyes_bringup single_shot_pick.launch.py \
  dry_run:=false enable_live_execution:=true operator_confirmed:=true \
  site_profile_path:=/home/elephant/temp/deyes/right_arm_site.yaml \
  stereo_calibration_path:=/home/elephant/temp/deyes/stereo_validated.yaml \
  extrinsics_path:=/home/elephant/temp/deyes/base_link_T_left_camera.yaml \
  model_path:=/absolute/path/pencil.engine model_id:=pencil-yolov8 model_sha256:=<sha256> \
  log_root:=/home/elephant/temp/deyes/single_shot_pick
```

首次现场验证仍按单关节点动、预抓取悬停、空夹、抓取抬升原位放回四次独立启动执行。任何失败后人工排障并 reset/relaunch，禁止自动重试。

本地回放和报告目录为 `E:/a_robot/temp/deyes/single_shot_pick/<transaction_id>/`；Jetson 使用上例对应路径。每个事务保存校正图、米制深度、CameraInfo、平面 manifest 和执行 JSONL trace。
