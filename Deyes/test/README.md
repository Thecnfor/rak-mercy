# Test Notes

本目录放不依赖实机大数据的轻量测试和回归检查。

当前提供：

- `test_sync_policy.py`：验证同步监控逻辑在以下场景的行为：
  - 正常同步
  - 左右图像时间差超阈值
  - `CameraInfo` 尺寸与图像不一致
  - 图像流超时
  - 允许静态软同步时的告警行为

运行方式：

```bash
python3 E:/a_robot/rak-mercy/Deyes/test/test_sync_policy.py
```

后续接入 rosbag 后，再补离线回放与回归基线测试。

当前基线运行入口：

- 在线图像链路：`ros2 launch deyes_bringup imx219_stereo.launch.py`
- 官方几何基线：`ros2 launch deyes_bringup stereo_image_proc_baseline.launch.py`
- SGBM 对照基线：`ros2 launch deyes_bringup sgbm_baseline.launch.py`

注意：

- 当前 Jetson `ROS 2 Galactic` 环境已确认默认**未安装** `stereo_image_proc`，因此官方几何基线在安装该包前无法直接运行。
- `sgbm_baseline` 可以在现有 `IMX219 -> image_raw + camera_info` 链路上直接做 debug 级别验证。
