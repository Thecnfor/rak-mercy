# M1 实机在线检查清单

目标：在继续使用当前 `spec` 标定占位参数的前提下，确认 `IMX219 -> ROS 2 image_raw + camera_info -> deyes_sync_monitor` 这条链路稳定可用。

## 启动

```bash
ros2 launch deyes_bringup imx219_stereo.launch.py enable_monitor:=true
```

## 必查项

1. 图像话题存在：

```bash
ros2 topic list -t | grep -E '/x1/(left_camera|right_camera)'
```

2. 帧率接近目标值：

```bash
ros2 topic hz /x1/left_camera/image_raw
ros2 topic hz /x1/right_camera/image_raw
```

3. `frame_id` 与尺寸存在且不为空：

```bash
ros2 topic echo /x1/left_camera/image_raw | head -n 20
ros2 topic echo /x1/left_camera/camera_info | head -n 30
```

4. 同步 gate 与失败原因：

```bash
ros2 topic echo /deyes_sync_monitor/depth_gate_ok
ros2 topic echo /deyes_sync_monitor/failure_reason
ros2 topic echo /deyes_sync_monitor/diagnostics
```

## 记录要求

- 记录实测启动时间、分辨率、帧率和 `hard_sync_max_ms` 配置值。
- 记录左右图像实际时间戳差的统计结果。
- 记录是否出现以下异常：
  - `camera_info_missing`
  - `camera_info_size_mismatch`
  - `stereo_out_of_sync`
  - `left_image_stale`
  - `right_image_stale`

## 输出文件

将本轮检查结果写入：

- `E:/a_robot/temp/deyes/reports/m1_sync_report_<yyyymmdd_hhmm>.md`
- `E:/a_robot/temp/deyes/reports/m1_sync_metrics_<yyyymmdd_hhmm>.csv`

## 判定

- 满足以下条件时，视为 `M1` 链路联调通过：
  - 左右图像与 `CameraInfo` 都存在
  - 图像尺寸与 `CameraInfo` 一致
  - `frame_id` 非空
  - `depth_gate_ok=true`
  - 左右时间戳差进入配置阈值

- 若 `depth_gate_ok=false`，必须保留 `failure_reason` 原文，不得只写“启动失败”。
