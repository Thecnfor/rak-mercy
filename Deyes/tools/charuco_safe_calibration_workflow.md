# ChArUco 安全标定（现场待验）

离线代码不会打开串口、连接机器人或发布运动命令。仅在现场确认相机、头部和急停状态后，先生成可审计板图：

```bash
ros2 run deyes_stereo charuco_board_generator stereo --output-dir /var/workspace/temp/charuco-stereo
```

双目板为 `8x6` squares、`30 mm` square、`22 mm` marker、`DICT_5X5_1000`。打印时必须 100% 缩放并用卡尺核对尺寸条；采集只接受 640x360、左右 skew 不超过 10 ms、40--60 组（默认 50）和每对至少 12 个共同 ChArUco ID。YAML 必须记录板元数据、SHA256、共同角点统计、覆盖、RMS/P95、人工左右顺序/基线符号/尺度确认；只有 `physical_charuco` 与 `validated:true` 可被 CUDA 深度入口接纳。

手眼采集为无接触人工拖动：`save` 仅用于放松、`pause` 用于锁定。每次锁定后才允许读取稳定的 `get_base_coords` 和同步校正左图；异常或退出必须保持 pause/clean。采集 12--16 姿态、平移跨度至少 100 mm、两个旋转轴各至少 30 度。拖动中采样、串口占用、反馈过期、头部偏差超过 0.5 度或姿态退化必须拒绝。生成的结果始终绑定 `robot_id`、`camera_pair_id`、`stereo_calibration_id`；在 Shah/Li 现场交叉验证和人工确认前 `trusted_for_execution:false`。

Jetson 上还须确认 OpenCV 4.8 隔离构建同时带有 `aruco`、`objdetect` 和 Python `cv2.aruco`，以及 `calibrateRobotWorldHandEye`。这些均为现场待验，不得把离线测试当作设备通过。
