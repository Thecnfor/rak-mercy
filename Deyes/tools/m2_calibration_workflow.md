# M2 物理双目标定流程（640×360 实体棋盘格）

目标：用当前实机 `IMX219` 双目在**真实安装状态**下生成可审计、可复现的双目标定文件，替换当前仅用于联调的 `spec` 参数，并联动完成 `M3` baseline 复验。

## 1. 标定前提

- 相机支架固定完毕，不再继续松动或重装。
- 采集分辨率与后续可信运行分辨率一致；本轮固定 `640x360@30fps`。不同分辨率只能做 debug，不得使候选结果 validated。
- 左右图像链路、`CameraInfo` 和同步诊断已通过 `M1` 检查。
- 标定前先做 `640x360@30` 预热检查；只有在 `publish_hz≈30` 且 `drop_skew` 不持续增长时才开始采集。
- 正式工具为仓库包的 `physical_stereo_calibration`，它只订阅正式话题并在保存前拒绝真实 stamp 差超过 `10 ms` 的帧对。
- 正式采集必须订阅 `/x1/left_camera/image_raw` 与 `/x1/right_camera/image_raw`，禁止再走 `grab_stereo.py` 直连 `nvarguscamerasrc` 的旁路采样。

## 2. 标定板要求

- 本轮目标板型为尺寸经过卡尺复测的 `checkerboard 9x6` 板。
- 记录以下信息：
  - 内角点规格（`9 x 6`）
  - 单个方格边长
  - 板子编号或打印批次

## 3. 采集要求

- 工具固定接受 `40–60` 组有效同步图，建议采集 `50` 组；少于 40 或超过 60 组不能验证。
- 视角覆盖：
  - 九宫格位置
  - 近、中、远距离
  - 正视、轻微偏航、轻微俯仰
- 删除样本必须记录原因：
  - 角点不全
  - 运动模糊
  - 左右不同步
  - 反光/遮挡
  - 重复姿态（棋盘中心、尺度、角度近似相同）
- 工具记录 3×3 画面位置覆盖；九宫格不完整时 `validated=false`。

## 4. 求解与产物

- 使用 `physical_stereo_calibration capture` 采集，再使用 `physical_stereo_calibration compute` 求解。
- 标定输出至少包含：
  - `K1/D1/K2/D2`
  - `R/T`
  - `P1/P2`
  - `Q`
  - `reproj_rms_px`
  - `epipolar_p95_px`（校正后全部角点的垂直像素误差 P95）
- 标定文件复制到仓库：

```text
E:/a_robot/temp/deyes/calibration/<session>/stereo_calib_candidate.yaml
```

## 5. 报告要求

每次物理标定都要在会话目录内生成 JSON 和 Markdown 报告（推荐会话根目录 `E:/a_robot/temp/deyes/calibration/`），至少包含：

- 标定日期
- 使用分辨率
- 有效样本数
- 删除样本原因统计
- `reproj_rms_px` 和 `epipolar_p95_px`
- 抽检图像的极线误差
- 生成该标定的代码版本和命令

## 5.1 Admin GUI 向导映射

当前正式入口 `E:/a_robot/rak-mercy/admin_gui` 中的 `Deyes -> 标定` 模块按以下步骤组织：

1. `步骤 1 / 预检`
- 对应任务：`640 预检`
- 目标：确认 `640x360@30` 下发布链稳定

2. `步骤 2 / 板信息`
- 用户动作：录入板编号、单方格边长
- 固定条件：`checkerboard 9 x 6`、`print at 100% scale`

3. `步骤 3 / 采集`
- 对应任务：`采集棋盘格`
- 用户动作：根据双路视觉反馈移动棋盘格，覆盖不同位置和姿态

4. `步骤 4 / 计算`
- 对应任务：`计算标定`
- 系统动作：从报告中提取 `reproj_rms_px`、`epipolar_p95_px`、候选 YAML 路径和核心矩阵信息

5. `步骤 5 / 验收`
- 用户动作：结合两项误差、人工确认和停止条件，标记“验收通过”或“需重采”

6. `步骤 6 / 复验`
- 对应任务：
  - `官方基线`
  - `SGBM 基线`
- 目标：在新标定完成后验证几何与 debug 链路

## 6. 替换规则

- 将 `imx219_capture_cpp.yaml`、`imx219_publisher.yaml`、`stereo_image_proc_baseline.launch.py` 和 `sgbm_baseline.launch.py` 的 `calib_path` 切换到仓库内标定文件。
- 切换后必须重新执行：
  - `M1` 同步诊断
  - `stereo_image_proc` 基线
  - `SGBM` 基线

## 7. 停止条件

出现以下任一情况，候选 YAML 必须保持 `validated: false`，并停止进入点云/抓取验收：

- `reproj_rms_px > 0.50 px`
- 校正后极线误差 `P95 > 0.50 px`
- 标定分辨率与运行分辨率不一致
- 相机支架重装后仍沿用旧标定
- 现场无法确认 `robot_id` 或相机标识，无法形成可审计文件名
- 标定板规格与 `CHECKERBOARD=(9, 6)` 不一致
- 操作者没有确认左右目、基线符号和尺度三项

## 现场命令（人工检查点）

先启动正式 C++ 采集链，**不**启动第二个相机消费者：

```bash
source /opt/ros/galactic/setup.bash
source /home/elephant/deyes_ws/install/setup.bash
ros2 launch deyes_bringup imx219_stereo.launch.py use_cpp_capture:=true \
  width:=640 height:=360 fps:=30 enable_cuda_depth:=false
```

实体棋盘必须为 `9x6` 内角点；用卡尺量单格边长并换算成米。操作者确认相机支架固定、左右话题未交换后执行：

```bash
ros2 run deyes_stereo physical_stereo_calibration capture \
  --session-dir /home/elephant/temp/deyes/calibration/<utc-session> \
  --square-size-m <caliper_value_m> --samples 50
```

移动棋盘覆盖九宫格、近中远距离和倾角，工具到 50 个有效帧后自动结束。核对候选点云方向、`T[0]` 符号和已知格长尺度后，才可显式确认并求解：

```bash
ros2 run deyes_stereo physical_stereo_calibration compute \
  --session-dir /home/elephant/temp/deyes/calibration/<utc-session> \
  --robot-id <confirmed_robot_id> --camera-pair-id <confirmed_pair_id> \
  --confirm-left-right --confirm-baseline-sign --confirm-scale
```

不得把候选 YAML 复制进仓库 `config/camera/`；先由负责人审阅报告和现场确认记录。工具不会替代实体板、卡尺读数或这三项人工确认。
