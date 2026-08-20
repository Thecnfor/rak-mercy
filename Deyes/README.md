# Deyes 双目摄像头可信测距

本目录用于 Mercury X1 的双目摄像头实机开发，当前优先目标不是“生成一张深度图”，而是在 `0.20-1.00 m` 范围内输出带坐标定义、标定版本、时间戳、有效性和失败原因的可信距离结果。

## 当前范围

第一轮实现覆盖 `M0`、`M1` 和两条 `M3` 深度基线的起步链路：

- `M0`：设备与 ROS 2 接口盘点。
- `M1`：实现 `deyes_sync_monitor`，对左右图像和 `CameraInfo` 做时间、尺寸、帧率与新鲜度诊断。
- `M3-baseline-a`：跑通 `stereo_image_proc` 官方几何基线。
- `M3-baseline-b`：跑通 `OpenCV StereoSGBM` 对照基线。
- 不在未接入实机时宣称完成 M0/M1 验收。
- 不在未知话题名和未知同步方式下写死驱动代码。

## 距离定义

- `z_m`：目标相对于校正后左目光学坐标系 `left_camera_optical_frame` 的 Z 方向距离。
- `range_m`：从左目光心到目标点的三维欧氏距离。
- 结果必须绑定 `frame_id`、采集时间戳与标定版本。
- 可信度不足时必须输出 `valid=false` 和明确原因，而不是默认 0 或无意义数值。

## 目录结构

```text
Deyes/
├── README.md
├── config/
│   ├── camera/
│   └── stereo/
├── src/
│   ├── deyes_bringup/
│   ├── deyes_capture_cpp/
│   └── deyes_stereo/
├── test/
└── tools/
```

后续 `deyes_interfaces`、双目标定、视差、深度质量评估和目标距离估计将在后续里程碑补齐；本轮先把接口、同步和离线回放基线立住。

## 第一轮交付

- 最小 ROS 2 包：`deyes_stereo`
- 启动与参数：`deyes_bringup`
- 参数文件：`config/stereo/sync_monitor.defaults.yaml`、`config/stereo/imx219_publisher.yaml`
- 只读盘点脚本：`tools/m0_inventory_commands.sh`
- 回放与测试说明：`tools/m1_replay_commands.sh`、`test/test_sync_policy.py`

## 当前链路状态

- 当前已验证的 ROS 2 图像接口统一为：
  - `/x1/left_camera/image_raw`
  - `/x1/right_camera/image_raw`
  - `/x1/left_camera/camera_info`
  - `/x1/right_camera/camera_info`
- 当前主链已切换为 Jetson 专用 C++ 节点 `deyes_capture_cpp/imx219_stereo_capture_node`，Python `imx219_stereo_publisher` 保留为调试/回退链。
- 自 `2026-08-18` 起，正式双目标定采样也必须复用这条 `C++/ROS` 主链，统一订阅 `/x1/left_camera/image_raw` 与 `/x1/right_camera/image_raw`；不再接受 `mercury_grasp/grab_stereo.py` 这类直连 `nvarguscamerasrc` 的旁路采集。
- 当前发布链路的目标是稳定 `30 Hz`，默认按中等分辨率优先做高帧率修复，而不是长期停留在最低分辨率 debug 档。
- `deyes_sync_monitor` 当前以 `sensor_data` QoS 订阅这些话题，默认阈值由 `config/stereo/sync_monitor.defaults.yaml` 提供。
- launch 默认读取 `deyes_bringup` 安装的 `config/camera/stereo_calib.yaml`；该文件为 `validated=false` 的规格推导调试参数。物理标定完成后必须通过 `calib_path:=<physical_yaml>` 显式切换。
- `/x1/stereo/depth` 的投影真源是 CUDA 节点同步发布的
  `/x1/stereo/left/camera_info_rect`，不是原始 `/x1/left_camera/camera_info`。两者具有相同
  header、分辨率与 `left_camera_optical_frame`；下游深度、融合和点云必须订阅 rectified 话题。
- 采集节点保留左右各自真实 capture stamp，并以标准
  `diagnostic_msgs/DiagnosticArray` 发布 `/x1/stereo/pair_diagnostics`。静态可信深度的
  `max_sync_diff_ms` 固定默认 10 ms，超限帧会被拒绝而不是伪造共同时间戳。
- 在完成物理双目标定前，任何 `depth/disparity` 结果都视为 debug 基线结果，不作为 `0.20-1.00 m` 可信测距验收依据。
- 当前已识别的主要瓶颈是 Python 图像消息构造与发布路径，而不是 IMX219 硬件本身；若执行期临时降分辨率，只作为达到稳定 `30 Hz` 的工程手段。
- C++ 主链继续沿用 `nvarguscamerasrc + NVMM + appsink` 路线，在节点内完成左右独立采集、最近帧配对、stale/skew 门控和 `CameraInfo` 同步发布。
- 当前新增 CUDA 深度链入口 `deyes_capture_cpp/cuda_stereo_depth_node`，用于在保留主链双目图输出的同时，额外生成正式 `/x1/stereo/disparity` 和 `/x1/stereo/depth` 话题。

## 实机前提

- 先读取现场相机型号、序列号、USB/CSI 拓扑、实际分辨率和 ROS 2 话题。
- 左右图像、`CameraInfo`、深度/点云必须使用一致时间基准。
- 标定文件按机器人编号、相机序列号、分辨率和日期版本化。
- 感知结果必须带坐标系和时间戳，并通过 TF2 转换后再交给机械臂。
- 原始数据、标定图片、rosbag、报告和构建产物全部写入 `E:/a_robot/temp/deyes`。

## 首次上机命令

```bash
lsusb
v4l2-ctl --list-devices
echo "$ROS_DISTRO"
ros2 launch turn_on_mercury_robot mercury_camera.launch.py --show-args
ros2 topic list -t | grep -E 'image|camera_info|depth|points'
ros2 topic info /your/left/image -v
ros2 topic hz /your/left/image
ros2 topic hz /your/right/image
ros2 run tf2_tools view_frames
```

确认实际话题、时间戳能力和 `CameraInfo` 一致性后，再进入标定、极线校正和视差/深度阶段。

## 当前启动入口

在线图像链路与同步诊断：

```bash
ros2 launch deyes_bringup imx219_stereo.launch.py enable_monitor:=true use_cpp_capture:=true
```

在线图像链路 + CUDA 深度输出：

```bash
ros2 launch deyes_bringup imx219_stereo.launch.py \
  enable_monitor:=true \
  use_cpp_capture:=true \
  enable_cuda_depth:=true \
  calib_path:=/path/to/your/stereo_calib.yaml
```

M1 收敛回归推荐命令（先在 shell 外设置 `ROBOT_IP=192.168.166.121`）：

```bash
ros2 launch deyes_bringup imx219_stereo.launch.py \
  enable_monitor:=true \
  use_cpp_capture:=true \
  target_publish_hz:=30.0 \
  pair_max_skew_ms:=20.0 \
  frame_stale_sec:=0.2 \
  history_size:=8 \
  monitor_expected_min_rate_hz:=20.0 \
  monitor_hard_sync_max_ms:=3.0 \
  monitor_soft_sync_max_ms:=10.0 \
  monitor_allow_soft_sync:=false
```

M1 收敛阶段默认使用上面的 monitor 阈值组合，不再沿用宽松的静态回放阈值；如果需要分析离线或静态软同步样本，必须显式传入 `monitor_allow_soft_sync:=true`，且该结果不能作为移动目标可信输出依据。

如需回退到 Python 调试链：

```bash
ros2 launch deyes_bringup imx219_stereo.launch.py enable_monitor:=true use_cpp_capture:=false
```

后续将补充两条基线：

- `stereo_image_proc_baseline.launch.py`
- `sgbm_baseline.launch.py`

独立验证 CUDA 深度链：

```bash
ros2 launch deyes_bringup cuda_depth.launch.py \
  calib_path:=/path/to/your/stereo_calib.yaml
```

## M1 回归标准

- 固定采样时长：每轮至少运行 `30 s`，连续执行两轮。
- 关注日志字段：
  - `publish_hz`
  - `drop_skew` 及 `(+delta)`
  - `drop_stale` / `wait_pair`
  - `skew_window={min=... median=... p95=...}`
  - `left_failures` / `right_failures`
- 通过判据：
  - 采集失败计数保持 `0`。
  - 发布速率稳定接近目标 `30 Hz`，不出现持续性掉到约 `10-13 Hz` 的退化段。
  - `drop_skew` 允许存在少量累计，但其增量必须低且可解释，不能持续快速增长。
  - `sync_monitor` 不应反复出现 `stereo_soft_sync_only` 或 `stereo_out_of_sync`。
- 不通过处理：
  - 若出现持续退化段或 `drop_skew` 快速累积，优先继续调试 C++ 配对策略，禁止进入 M2 标定。
  - 只有 M1 回归稳定后，才允许开始物理标定并将标定 YAML 落库到 `config/camera/`。

## 基线说明

### stereo_image_proc 官方几何基线

- 启动文件：`src/deyes_bringup/launch/stereo_image_proc_baseline.launch.py`
- 目标：在保持 `/x1/left_camera/*`、`/x1/right_camera/*` 接口不变的前提下，复用 ROS 2 官方 stereo 处理链，验证校正、视差和点云输出。
- 依赖：目标 Jetson 需要安装 `stereo_image_proc`。当前已知实机 `ROS 2 Galactic` 环境默认**未安装**此包，若 launch 报 `Package not found`，优先补安装，不要误判为 `Deyes` 代码错误。

### SGBM 对照基线

- 启动文件：`src/deyes_bringup/launch/sgbm_baseline.launch.py`
- 节点：`src/deyes_stereo/deyes_stereo/sgbm_baseline_node.py`
- 输出：
  - `/x1/stereo/debug/left_rect`
  - `/x1/stereo/debug/right_rect`
  - `/x1/stereo/debug/disparity`
  - `/x1/stereo/debug/depth`
  - `/x1/stereo/debug/valid_mask`
- 该节点只输出 debug 基线结果，不提供最终 `valid/confidence` 结论。

### CUDA StereoBM 深度链

- 启动文件：
  - `src/deyes_bringup/launch/cuda_depth.launch.py`
  - `src/deyes_bringup/launch/imx219_stereo.launch.py` 配合 `enable_cuda_depth:=true`
- 节点：`src/deyes_capture_cpp/src/cuda_stereo_depth_node.cpp`
- 目标：参考 `docs/Jetson-Stereo-CSI-Camera-main` 的 CUDA 处理方式，把校正、预处理和视差匹配迁移到 C++/CUDA 路线，同时保持现有 `/x1/left_camera/*` 与 `/x1/right_camera/*` 主链接口不变。
- 当前输出：
  - `/x1/stereo/disparity`
  - `/x1/stereo/depth`
  - `/x1/stereo/debug/left_rect`
  - `/x1/stereo/debug/right_rect`
  - `/x1/stereo/debug/valid_mask`
- 当前限制：
  - 当前默认采用稳定优先的 `CUDA StereoBM` 主链；Jetson 风格的 `WLS` 增强已接入，但在 Xavier NX 上需按实时预算谨慎开启。
  - 第一版仍以当前主链图像输出形态为输入，不强制切换到 `bgr8`。
  - 在完成物理标定前，该链路输出仍只作为 debug/基线结果。
- 现场恢复前置条件：
  - 目标机已安装 `OpenCV 4.8.0 CUDA`，并可通过 `/home/elephant/opencv-4.8.0-cuda` 提供 `cudaimgproc`、`cudawarping`、`cudafilters`、`cudastereo`。
-  - 启动前必须保证 `LD_LIBRARY_PATH` 同时包含 `/opt/ros/galactic/lib`、`/opt/ros/galactic/lib/aarch64-linux-gnu` 和 `/home/elephant/opencv-4.8.0-cuda/lib`，否则 `ros2 launch` 可能因找不到 `librcl_action.so`、`libddsc.so.0` 或 OpenCV CUDA 动态库而直接失败。
  - 当前 `stereo_calib.yaml` 至少要包含 `img_size`、`K1`、`D1`、`K2`、`D2`、`R`、`T`，其中 `T` 必须能解析成 3 维平移向量。
- 推荐现场启动方式：

```bash
export LD_LIBRARY_PATH=/opt/ros/galactic/lib:/opt/ros/galactic/lib/aarch64-linux-gnu:/home/elephant/opencv-4.8.0-cuda/lib:$LD_LIBRARY_PATH
export OpenCV_DIR=/home/elephant/opencv-4.8.0-cuda/lib/cmake/opencv4
export PKG_CONFIG_PATH=/home/elephant/opencv-4.8.0-cuda/lib/pkgconfig:$PKG_CONFIG_PATH
source /opt/ros/galactic/setup.bash
source /home/elephant/temp/deyes/install/setup.bash
ros2 launch deyes_bringup imx219_stereo.launch.py \
  enable_monitor:=true \
  use_cpp_capture:=true \
  enable_cuda_depth:=true
```

- 稳定实时深度流推荐启动命令：

```bash
export LD_LIBRARY_PATH=/opt/ros/galactic/lib:/opt/ros/galactic/lib/aarch64-linux-gnu:/home/elephant/opencv-4.8.0-cuda/lib:$LD_LIBRARY_PATH
export OpenCV_DIR=/home/elephant/opencv-4.8.0-cuda/lib/cmake/opencv4
export PKG_CONFIG_PATH=/home/elephant/opencv-4.8.0-cuda/lib/pkgconfig:$PKG_CONFIG_PATH
source /opt/ros/galactic/setup.bash
source /home/elephant/temp/deyes/install/setup.bash
ros2 launch deyes_bringup imx219_stereo.launch.py \
  enable_monitor:=true \
  use_cpp_capture:=true \
  enable_cuda_depth:=true \
  width:=640 \
  height:=360 \
  fps:=30 \
  target_publish_hz:=30.0 \
  pair_max_skew_ms:=20.0 \
  frame_stale_sec:=0.2 \
  history_size:=8 \
  monitor_expected_min_rate_hz:=20.0 \
  monitor_hard_sync_max_ms:=3.0 \
  monitor_soft_sync_max_ms:=10.0 \
  monitor_allow_soft_sync:=false \
  cuda_depth_max_sync_diff_ms:=10.0 \
  cuda_depth_publish_period_sec:=0.07 \
  cuda_depth_min_depth_m:=0.20 \
  cuda_depth_max_depth_m:=1.00 \
  cuda_depth_enable_wls_filter:=false \
  cuda_depth_wls_lambda:=8000.0 \
  cuda_depth_wls_sigma_color:=2.0 \
  cuda_depth_texture_threshold:=0 \
  cuda_depth_uniqueness_ratio:=0 \
  cuda_depth_speckle_window_size:=0 \
  cuda_depth_speckle_range:=0 \
  cuda_depth_disp12_max_diff:=0 \
  cuda_depth_publish_debug_rect:=false \
  cuda_depth_publish_debug_mask:=false
```

- 上面这组参数默认追求“先稳定实时，再细调深度细节”：
  - `width/height/fps`：先锁定输入负载；现场优先从 `640x360@30` 跑稳，再尝试回升到更高分辨率。
  - `target_publish_hz`：C++ 主链目标发布速率；它不稳定时，先不要怀疑 CUDA 匹配器。
  - `pair_max_skew_ms`、`frame_stale_sec`、`history_size`：主链左右配对窗口、过期门限和缓存深度；它们决定深度节点能否拿到新鲜且可配对的左右帧。
  - `cuda_depth_max_sync_diff_ms`：深度节点自己的左右时间窗；过小会频繁 `pair_out_of_window`，过大则可能把配对误差带进深度结果。
-  - `cuda_depth_publish_period_sec`：深度节点计算与发布节拍；当前 Jetson Xavier NX 实测推荐默认值为 `0.07`，对应约 `14 Hz` 的稳定深度流；若后续优化了算子链，再尝试往 `0.05` 或更低回推。
  - `cuda_depth_min_depth_m`、`cuda_depth_max_depth_m`：深度有效范围裁剪；范围过窄时容易表现为“有视差但深度大面积 NaN”。
  - `cuda_depth_enable_wls_filter`、`cuda_depth_wls_lambda`、`cuda_depth_wls_sigma_color`：Jetson 风格的左右向 disparity + WLS 平滑增强开关与参数；当前默认关闭，因为 Xavier NX 上开启后实测会明显增加单帧处理时间，只有在现场确认算力余量足够时才建议打开。
  - `cuda_depth_publish_debug_rect`、`cuda_depth_publish_debug_mask`：调试图像开关；默认关闭，避免把 `left_rect/right_rect/valid_mask` 下载与发布开销常态化。
- 若需要继续细调 `StereoBM` 本体参数，不要直接改默认文件；先复制 `config/stereo/cuda_depth.defaults.yaml` 到现场 YAML，再通过 `cuda_depth_config:=/path/to/your.yaml` 注入，重点参数含义如下：
  - 当前默认组合偏向“近距覆盖优先”：
    - `block_size: 11`
    - `median_ksize: 3`
    - `texture_threshold: 0`
    - `uniqueness_ratio: 0`
    - `speckle_window_size: 0`
    - `speckle_range: 0`
    - `disp12_max_diff: 0`
  - `num_disparities`：搜索视差范围，必须能被 `16` 整除；越大覆盖越远但算力与误匹配风险越高。
  - `block_size`：匹配窗口大小，必须为正奇数；越大越稳但边缘会更糊。
  - `median_ksize`：中值滤波核，必须为大于 `1` 的奇数；越大越稳但细节损失更明显。
  - `texture_threshold`、`uniqueness_ratio`、`speckle_window_size`、`speckle_range`、`disp12_max_diff`：匹配质量约束；优先小步微调，不要和实时性参数同轮大改。
  - `frame_queue_size`：深度节点内部左右帧缓存；只在上游偶发抖动时小步增大，不要把它当成长期掩盖同步问题的手段。
  - `processing_overrun_factor`：处理超时判定倍数；只用于诊断预算是否过紧，不应用它长期掩盖真实算力不足。
- 推荐调参顺序：
  1. 先看采集与 monitor 是否稳定：确认 `target_publish_hz≈30`、`drop_skew` 不持续累加，且 `deyes_sync_monitor` 不反复报 `stereo_out_of_sync`。
  2. 再看深度节点配对是否稳定：观察 `/cuda_stereo_depth_node/status` 与 `/cuda_stereo_depth_node/status_detail`，只有长期保持 `ok` 才进入下一步。
  3. `cuda_depth_max_sync_diff_ms` 在可信模式固定为 `10 ms`；任何超过 `10 ms` 的配对都必须拒绝。若为临时 debug 放宽该值，标定与输出必须保持 `validated=false`，不得用于真值验收或抓取。
  4. 仅在 `processing_overrun` 明显时再调负载：优先关闭 debug 输出，其次降低分辨率，最后才围绕 `cuda_depth_publish_period_sec:=0.07` 做小步回推，验证是否还能继续逼近更高深度帧率。
  5. 主链和实时性稳定后，再调 `num_disparities`、`block_size`、`median_ksize` 改善有效区域与噪声，不要把图像质量参数和实时性参数混在同一轮里改。
- 快速排查：
  - 无深度输出：
    - 先查输入是否齐全：`ros2 topic hz /x1/left_camera/image_raw`、`ros2 topic hz /x1/right_camera/image_raw`。CUDA 深度节点只以左右图像和已加载的物理标定为几何输入，不依赖原始 `CameraInfo`；其校正后的 `CameraInfo` 由节点同步发布。
    - 再看节点状态：`ros2 topic echo /cuda_stereo_depth_node/status` 与 `ros2 topic echo /cuda_stereo_depth_node/status_detail`；若为 `missing_input`，优先修复左右图像输入；若为 `pair_out_of_window`，先核对左右时间戳差并修主链，可信模式不得放宽 `10 ms` 门限。
    - 若节点启动即退出或始终无输出，优先检查 `calib_path` 是否存在且至少包含 `img_size/K1/D1/K2/D2/R/T`，并确认输入编码是 `mono8`、`bgr8` 或 `rgb8`。
  - 频繁跳帧：
    - 先读 `depth_stats` 日志和状态话题；`pair_out_of_window` 多说明左右帧时间差超过窗口，`stale_frame` 多说明上游发布抖动或深度节点来不及消费。
    - 先调整主链：确认 `pair_max_skew_ms:=20.0`、`frame_stale_sec:=0.2`、`history_size:=8` 仍有效，必要时先回到 `640x360@30` 复核主链是否稳定。
    - 主链已稳定但仍偶发错过配对时，先修正采集时间基准、队列或负载；可信模式不得把 `cuda_depth_max_sync_diff_ms` 放宽超过 `10 ms`。仅 debug 可单独调整队列，并必须保持 `validated=false`。
  - 性能退化：
    - 若 `/cuda_stereo_depth_node/status` 反复变成 `processing_overrun`，说明单轮处理时间已经超过当前发布预算；先保持 debug 输出关闭。
    - 用 `ros2 topic hz /x1/stereo/depth`、`ros2 topic hz /x1/stereo/disparity` 和节点日志里的 `last_processing_ms` 对照确认是否为纯算力瓶颈；必要时把 `cuda_depth_publish_period_sec` 临时改到 `0.04-0.05` 做验证。
    - 若放宽发布周期到 `0.07` 后恢复稳定，优先降分辨率或降低匹配负载；不要直接长期增大 `processing_overrun_factor` 来“消音”告警。

## 1m 可信深度与点云验收（当前目标）

- 目标距离范围：`0.20-1.00m`
- 验收模式：优先采用 `balanced` 参数组合，兼顾连续性、误差和实时性
- 真值点位与每点样本数：`0.30m`、`0.50m`、`0.80m`、`1.00m`，每点至少 `100` 帧；从左目光心到参考平面测量，禁止用滤波后的样本替代原始样本。
- 固定误差门槛：
  - `0.20–0.60m`：`MAE <= 0.010m`，`P95 绝对误差 <= 0.020m`；
  - `0.60–1.00m`：`MAE <= 0.020m`，`P95 绝对误差 <= 0.030m`。
- 任一距离样本不足、字段无效或门槛失败时，验收报告必须为 `validated=false`；不得降低门槛或以滤波掩盖偏差。
- 稳定性目标：
  - 连续运行 `10min`，采集失败计数为 `0`，所有静态配对 `<=10ms`；
  - 左右图像各 `>=28Hz`，`/x1/stereo/depth` 与 `/x1/stereo/points` 各 `>=12Hz`；
  - 中心 ROI 有效覆盖率 `>=85%`，且无持续性 `processing_overrun`；
  - RViz 人工确认平面无明显弯曲/重影/分层，且光学坐标为 X 右、Y 下、Z 前。
- 近距覆盖目标：
  - 中心 ROI 内 `0.20-1.00m` 有效深度覆盖率 `>= 85%`
  - 可连续辨识地面、挡板/墙面和中等尺寸障碍物

## YOLO + 深度坐标探测

- 目标：在双目主链稳定输出深度后，引入目标检测，实现 `2D 检测 + 深度融合 + base_link 坐标输出`。
- 历史实测机：`192.168.137.17`（Jetson Xavier NX，JetPack R35.3.1）。这不是当前部署目标；当前机器人地址只通过 shell 环境变量 `ROBOT_IP=192.168.166.121` 注入。

### 节点拆分

- `yolo_detector`：`src/deyes_stereo/deyes_stereo/yolo_detector_node.py`
  - 输入：`/x1/left_camera/image_raw`
  - 输出：`/x1/detection/boxes`、`/x1/detection/boxes_status`、`/x1/detection/debug_image`
- `object_fusion`：`src/deyes_stereo/deyes_stereo/object_fusion_node.py`
  - 输入：`/x1/detection/boxes`、`/x1/stereo/depth`、`/x1/stereo/left/camera_info_rect`、TF
  - 输出：`/x1/detection/objects_3d`、`/x1/detection/objects_3d_status`

### 检测后端历史实测结论（192.168.137.17，非当前目标）

| 后端 | 状态 | 结论 |
| --- | --- | --- |
| `ultralytics` | 未安装 | 只作为离线/算法初版路线 |
| `opencv_dnn` | `cv2 4.2.0` 无法导入 `yolov5s.onnx` | 不作为正式路线 |
| `tensorrt` | `tensorrt 8.5.2.2` 成功 parse `yolov5s.onnx` | 正式部署路线 |

- 实测输入/输出：`images [1, 3, 640, 640]` → `output0 [1, 25200, 85]`。
- `yolo_detector_node` 的 `tensorrt` 后端使用 `torch.cuda` 作为 binding buffer，不依赖 `pycuda`（当前 Jetson 未安装 `pycuda`/`cuda-python`）。
- `object_fusion` 已在实机跑通：模拟检测框注入 `/x1/detection/boxes` 后可输出 `/x1/detection/objects_3d`。

### TensorRT engine 现场构建

```bash
python3 - <<'PY'
import tensorrt as trt
logger = trt.Logger(trt.Logger.WARNING)
builder = trt.Builder(logger)
network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
parser = trt.OnnxParser(network, logger)
with open('/path/to/yolov5s.onnx', 'rb') as f:
    parsed = parser.parse(f.read())
config = builder.create_builder_config()
config.max_workspace_size = 1 << 30
if builder.platform_has_fast_fp16:
    config.set_flag(trt.BuilderFlag.FP16)
engine = builder.build_engine(network, config)
with open('/path/to/yolov5s.engine', 'wb') as f:
    f.write(engine.serialize())
PY
```

- TensorRT engine 属于机器相关构建产物，保存到 `E:/a_robot/temp/deyes/models/`；仓库 `depends/ASSETS.md` 只记录来源、版本和校验值。

### 一体化启动（检测 + 融合）

```bash
ros2 launch deyes_bringup imx219_stereo.launch.py \
  enable_cuda_depth:=true \
  enable_depth_coordinate:=true \
  enable_detector:=true \
  detector_backend:=tensorrt \
  detector_model_path:=/path/to/yolov5s.engine \
  enable_object_fusion:=true
```

## 模式建议

- `safe`
  - 适合现场恢复或算力紧张时
  - 可关闭 WLS，保守使用热力图密度
- `balanced`
  - 当前默认模式
  - `WLS off`
  - 适合比赛前主链联调和 `1m` 内可靠深度视图验收
- `dense_debug`
  - 用于在线校正和近距观察
  - 保持主深度链不变，优先通过 `depth_coordinate_node` 的 `sample_step/max_points` 提高热力图细节
  - 若要尝试 `WLS on`，必须重新检查 `processing_ms` 是否仍满足当前发布预算

## 校正深度点云（Phase 3）

`stereo_pointcloud_node` 是不依赖 PCL 的正式点云投影节点。它只接受 CUDA 深度节点成对发布的：

- `/x1/stereo/depth`：`32FC1`，单位米；
- `/x1/stereo/left/camera_info_rect`：同一 `header.stamp`、尺寸与 `left_camera_optical_frame` 的校正后 CameraInfo。

节点拒绝 stamp、尺寸、frame、编码、数据长度或 `P[0]/P[5]/P[2]/P[6]` 不合法的整帧。输出 `/x1/stereo/points` 为有组织的 `x/y/z FLOAT32` PointCloud2；默认 `sample_step=2`、`0.20–1.00 m`，无效深度写入 NaN。`/x1/stereo/points_status` 给出有效覆盖率、丢弃/拒绝计数、标定 ID 与处理耗时。

集成启动（默认关闭）：

```bash
ros2 launch deyes_bringup imx219_stereo.launch.py \
  enable_cuda_depth:=true \
  enable_pointcloud:=true \
  pointcloud_calibration_id:=<physical-calibration-id> \
  pointcloud_calibration_validated:=false
```

也可在 CUDA 深度已运行时单独启动：

```bash
ros2 launch deyes_bringup pointcloud.launch.py \
  pointcloud_calibration_id:=<physical-calibration-id> \
  pointcloud_calibration_validated:=false
```

在物理棋盘格标定及 0.30/0.50/0.80/1.00 m 真值验收都通过以前，`calibration_validated` 必须保持 `false`。此时点云只允许 RViz/debug 使用，抓取、轨迹与任何执行器消费者不得订阅或消费它；诊断会以 `WARN debug_only` 标示。只有通过标定门禁的报告所对应、非 `unassigned` 的标定 ID 才能设置 `calibration_validated:=true`。

## 物理标定替换路径

- 当前 `spec` 参数只用于链路联调和 baseline 启动。
- 真实物理双目标定完成后，应把标定文件放入 `config/camera/`，并将各 launch 的 `calib_path` 切换到仓库内文件。
- 物理标定流程见：
  - `tools/m2_calibration_workflow.md`
  - `tools/m2_calibration_commands.sh`

## M2 物理标定

- 本轮目标：在 `ROBOT_IP=192.168.166.121` 的实机上按 `checkerboard 9x6 + 640x360@30` 执行物理双目标定。
- 执行前提：
  - 相机支架固定，不再重装。
  - `640x360@30` 下主链预热后进入 `publish_hz≈30`，且 `drop_skew` 不持续增长。
  - 标定板尺寸已用卡尺复测并记录。
  - 机器人标识与双目对标识可用于 YAML 命名。
- 当前执行路径：
  - 远端 `deyes_stereo physical_stereo_calibration` 使用**棋盘格** `capture/compute` 流程。
  - 现场应使用 `inner corners: 9 x 6`、`print at 100% scale` 的棋盘格标定板，并记录单格边长。
- 停止条件：
  - `640x360@30` 下主链无法满足标定采集前提。
  - 现场无法确认机器人标识、双目对标识或板尺寸。
  - 标定结果 `reproj_error > 0.50 px` 或极线误差 `P95 > 0.50 px`。

## M3 标定后复验

- 只有在生成真实物理标定 YAML 并落库到 `config/camera/` 后，才允许切换默认 `calib_path`。
- 复验顺序：
  - `ros2 launch deyes_bringup imx219_stereo.launch.py calib_path:=<repo_calib_yaml> width:=640 height:=360 fps:=30`
  - `ros2 launch deyes_bringup stereo_image_proc_baseline.launch.py calib_path:=<repo_calib_yaml>`
  - `ros2 launch deyes_bringup sgbm_baseline.launch.py calib_path:=<repo_calib_yaml>`
- 通过判据：
  - `CameraInfo` 与图像分辨率一致。
  - `M1` monitor 不引入新的同步或尺寸错误。
  - `stereo_image_proc` 与 `SGBM` 都能基于新标定稳定启动并输出预期结果。

## Admin GUI

- 正式入口切换到项目根目录的 `admin_gui/`
- 旧版 `Deyes/admin_gui` 已移至工作区归档 `E:/a_robot/temp/rak-mercy-baseline/archive/Deyes-admin_gui-legacy/`，仅供迁移参考
- `Deyes -> 标定` 页面当前承接：
  - `1280 预检`
  - `720 预检`
  - `采集棋盘格`
  - `计算标定`
  - `stereo_image_proc` 基线
  - `SGBM` 基线
  - 双目视觉反馈
  - 验收备注与复验状态
- 机器人启动方式：

```bash
cd "${DEYES_ADMIN_GUI_ROOT:?Set DEYES_ADMIN_GUI_ROOT to the deployed admin_gui directory}/backend"
python3 server.py --host 0.0.0.0 --port 8765
```

- 浏览器访问：

```text
http://${ROBOT_IP}:8765
```
