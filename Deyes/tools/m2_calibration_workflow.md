# M2 物理双目标定流程

目标：用当前实机 `IMX219` 双目在**真实安装状态**下生成可审计、可复现的双目标定文件，替换当前仅用于联调的 `spec` 参数，并联动完成 `M3` baseline 复验。

## 1. 标定前提

- 相机支架固定完毕，不再继续松动或重装。
- 采集分辨率与后续运行分辨率一致；本轮按 `1280x720@30fps` 规划。
- 左右图像链路、`CameraInfo` 和同步诊断已通过 `M1` 检查。
- 标定前先做 `1280x720@30` 预热检查；只有在 `publish_hz≈30` 且 `drop_skew` 不持续增长时才开始采集。
- 当前远端 `mercury_grasp.calibrate_stereo.py` 已切换为**棋盘格 + ROS/C++ 主链采样**流程，本轮正式按 `checkerboard 9x6` 执行。
- 正式采集必须订阅 `/x1/left_camera/image_raw` 与 `/x1/right_camera/image_raw`，禁止再走 `grab_stereo.py` 直连 `nvarguscamerasrc` 的旁路采样。

## 2. 标定板要求

- 本轮目标板型为尺寸经过卡尺复测的 `checkerboard 9x6` 板。
- 记录以下信息：
  - 内角点规格（`9 x 6`）
  - 单个方格边长
  - 板子编号或打印批次

## 3. 采集要求

- 至少采集 30 组有效同步图。
- 视角覆盖：
  - 九宫格位置
  - 近、中、远距离
  - 正视、轻微偏航、轻微俯仰
- 删除样本必须记录原因：
  - 角点不全
  - 运动模糊
  - 左右不同步
  - 反光/遮挡

## 4. 求解与产物

- 直接复用实机现有 `mercury_grasp/calibrate_stereo.py` 产出初版。
- 标定输出至少包含：
  - `K1/D1/K2/D2`
  - `R/T`
  - `P1/P2`
  - `Q`
  - `reproj_error`
- 标定文件复制到仓库：

```text
E:/a_robot/rak-mercy/Deyes/config/camera/<robot_id>_<camera_serial_or_sensorpair>_<width>x<height>_<yyyymmdd>.yaml
```

## 5. 报告要求

每次物理标定都要在 `E:/a_robot/temp/deyes/reports/` 留一份报告，至少包含：

- 标定日期
- 使用分辨率
- 有效样本数
- 删除样本原因统计
- `reproj_error`
- 抽检图像的极线误差
- 生成该标定的代码版本和命令

## 5.1 Admin GUI 向导映射

当前正式入口 `E:/a_robot/rak-mercy/admin_gui` 中的 `Deyes -> 标定` 模块按以下步骤组织：

1. `步骤 1 / 预检`
- 对应任务：`1280 预检`
- 目标：确认 `1280x720@30` 下发布链稳定

2. `步骤 2 / 板信息`
- 用户动作：录入板编号、单方格边长
- 固定条件：`checkerboard 9 x 6`、`print at 100% scale`

3. `步骤 3 / 采集`
- 对应任务：`采集棋盘格`
- 用户动作：根据双路视觉反馈移动棋盘格，覆盖不同位置和姿态

4. `步骤 4 / 计算`
- 对应任务：`计算标定`
- 系统动作：从日志中提取 `reproj_error`、YAML 路径和核心矩阵信息

5. `步骤 5 / 验收`
- 用户动作：结合 `reproj_error` 与停止条件，标记“验收通过”或“需重采”

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

出现以下任一情况，停止继续使用本次标定结果：

- `reproj_error > 0.50 px`
- 校正后极线误差 `P95 > 0.50 px`
- 标定分辨率与运行分辨率不一致
- 相机支架重装后仍沿用旧标定
- 现场无法确认 `robot_id` 或相机标识，无法形成可审计文件名
- 标定板规格与 `CHECKERBOARD=(9, 6)` 不一致
