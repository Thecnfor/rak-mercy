# 明日实机视觉与抓取联调清单

代码分支：`integration/navigation-system-20260821`

基线提交：`ffa0399`

统一工程：`rak-mercy-integration-navigation`

## 目标

当天按以下顺序跑通：

```text
双目标定 → 手眼标定 → 笔识别与深度 → base_link 坐标
→ 抓取 dry-run → 预抓取悬停 → 单次低速抓取
```

不要先开机械臂动作再补标定。任一门禁失败就停在当前阶段。

## 1. 接收代码与只读检查

```bash
git checkout integration/navigation-system-20260821
git pull --ff-only
git rev-parse --short HEAD
git status --short
```

确认机器人 ROS 版本、话题、TF 和控制权：

```bash
ls /opt/ros
ros2 node list
ros2 topic list -t
ros2 action list -t
fuser -v /dev/right_arm
ros2 run tf2_ros tf2_echo base_link left_camera_optical_frame
```

记录相机、导航、硬件桥和右臂串口占用者。不要 kill 官方进程来抢串口。

## 2. 构建与导入检查

```bash
source /opt/ros/<实际版本>/setup.bash
cd /home/elephant/deyes_ws
colcon build --symlink-install --packages-select \
  deyes_interfaces deyes_capture_cpp deyes_stereo deyes_bringup
source install/setup.bash

python3 -c "import deyes_stereo.single_shot_snapshot_node,deyes_stereo.pick_nav_coordinator_node,deyes_stereo.single_shot_pick_planner_node,deyes_stereo.single_shot_pick_executor_node; print('IMPORT_OK')"
```

预期输出 `IMPORT_OK`。失败时先补同步文件或依赖，不启动动作。

## 3. 准备四项现场资产

必须准备：

1. 物理双目标定 YAML；
2. `base_link ← left_camera_optical_frame` 手眼外参 YAML；
3. 右臂工作空间、关节限位、TCP、夹爪方向安全 YAML；
4. Jetson 可加载的笔识别模型及 SHA256。

仓库中的 `stereo_calib.yaml` 和 `right_arm_execution.site.template.yaml` 是 `validated:false` 模板，不能直接解除动作门禁。

## 4. 物理双目标定

- 固定相机支架后不得再移动；
- 运行分辨率固定 `640×360@30`；
- 棋盘为 `9×6` 内角点；
- 方格约 2 cm，但必须现场实测并换算成米；
- 采集 40–60 组，建议 50 组，覆盖九宫格、近中远和不同倾角。

```bash
ros2 launch deyes_bringup imx219_stereo.launch.py \
  use_cpp_capture:=true width:=640 height:=360 fps:=30 enable_cuda_depth:=false

ros2 run deyes_stereo physical_stereo_calibration capture \
  --session-dir /home/elephant/temp/deyes/calibration/<session> \
  --board-cols 9 --board-rows 6 \
  --square-size-m <实测米数> --samples 50

ros2 run deyes_stereo physical_stereo_calibration compute \
  --session-dir /home/elephant/temp/deyes/calibration/<session> \
  --robot-id <robot-id> --camera-pair-id <pair-id> \
  --board-cols 9 --board-rows 6 --square-size-m <相同米数> \
  --confirm-left-right --confirm-baseline-sign --confirm-scale
```

通过标准：

- 重投影 RMS `≤0.50 px`；
- 校正后极线误差 P95 `≤0.50 px`；
- 左右顺序、基线符号、距离尺度正确。

未通过时保持 `validated:false`，重新采集，不降低标准。

## 5. 深度与识别检查

```bash
ros2 topic hz /x1/stereo/debug/left_rect
ros2 topic hz /x1/stereo/depth
ros2 topic echo /x1/stereo/pair_diagnostics
ros2 topic echo /x1/stereo/left/camera_info_rect --once
ros2 topic echo /x1/ground/plane
```

要求：

- 左右时间差每帧 `≤10 ms`；
- depth 为米制 `32FC1`；
- depth 与 CameraInfo 的 stamp、尺寸、frame 一致；
- 桌面平面有效；
- 一支完整可见的笔只输出一个检测；
- 检测框、笔轴和深度位置在 RViz/调试图中一致。

模型配置必须填写真实 `model_path/model_id/SHA256`。模型加载失败、0 支、多支、截断或深度质量不足都不得抓取。

## 6. 手眼标定与坐标验证

至少采集 6 个不共线点，建议 8 个以上，覆盖 `≥80 mm` 空间范围。用同一物理标记获得：

- 相机光学坐标点；
- 右臂已确认 TCP 对应的 `base_link` 坐标点。

```bash
ros2 run deyes_stereo handeye_calibration \
  --input /home/elephant/temp/deyes/calibration/<session>/correspondences.json \
  --output /home/elephant/temp/deyes/calibration/<session>/base_link_T_left_camera.yaml
```

通过标准：RMS `≤5 mm`、P95 `≤10 mm`，且 robot、camera pair、stereo calibration ID 一致。

发布并检查 TF：

```bash
ros2 launch deyes_bringup validated_extrinsics_tf.launch.py \
  extrinsics_path:=/path/base_link_T_left_camera.yaml \
  stereo_calibration_path:=/path/physical_stereo.yaml

ros2 topic echo /x1/stereo/extrinsics_status
```

先让机械臂只到计算点上方悬停，人工测量误差；禁止直接下降抓取。

## 7. 全链路 dry-run

```bash
ros2 launch deyes_bringup navigation_single_shot_pick.launch.py \
  dry_run:=true enable_live_execution:=false operator_confirmed:=false \
  model_path:=/path/pen.engine model_id:=<id> model_sha256:=<sha256> \
  stereo_calibration_path:=/path/physical_stereo.yaml \
  extrinsics_path:=/path/base_link_T_left_camera.yaml \
  site_profile_path:=/path/right_arm_execution.yaml \
  log_root:=/home/elephant/deyes_logs/single_shot
```

检查状态顺序：

```text
PICK_ARMED → snapshot_frozen → 单次检测 → 坐标结果
→ dry_run_ready → dry_run_complete
```

同时查看：

```bash
ros2 topic echo /x1/pick/transaction_status
ros2 topic echo /x1/detection/boxes
ros2 topic echo /x1/coordinate_chain/result
ros2 topic echo /x1/pick/dry_run_plan
ros2 topic echo /x1/pick/execution_status
```

确认同一 `mission_id/nav_epoch/transaction_id/candidate_id/calibration_id`，并且 `hardware_commands_emitted=false`。

## 8. 机械臂分级动作

仅在标定、工作空间、TCP、夹爪方向、串口控制权和急停全部确认后执行：

1. 单关节 `0.5°` 点动；
2. 预抓取点上方 120 mm 悬停；
3. 空夹开合；
4. 无笔完整轨迹；
5. 单笔低速抓起、抬升 100 mm、原位放回。

首轮笛卡尔速度 `≤0.02 m/s`，vendor speed `≤5`。一次启动只执行一次，失败不自动重试。

## 9. 立即停止条件

- 串口被其他进程持有；
- TF、frame、时间戳或标定 ID 不一致；
- 导航到位后底盘仍在移动；
- 深度尺度错误、重影、分层或无效；
- 检测不是恰好一支完整笔；
- 悬停误差超过 10 mm；
- 反馈中断超过 250 ms；
- 超速、越界、错误码、夹爪方向错误；
- 人员进入运动区域或急停不可用。

停止后保存日志和快照，人工 reset 或重启；不要自动换帧、换目标或重试动作。

## 10. 当天必须保存

- Git commit、ROS 版本、节点/话题/action 清单；
- 模型 ID 与 SHA256；
- 双目和手眼标定 YAML/报告；
- 右臂安全配置及夹爪方向确认；
- 快照、深度、CameraInfo、坐标、计划、执行 trace；
- 每阶段通过/失败结论。

所有运行数据放 `/home/elephant/temp/deyes` 或外部 `temp`，不要提交进仓库。
