# Camera Config Notes

提交到仓库的相机标定文件需要同时绑定以下信息：

- 机器人编号或机身唯一标识
- 相机序列号
- 左右目分辨率与帧率
- 标定日期
- 生成标定时所用代码提交号

本轮 `M2+M3` 约定：

- 目标主机：`192.168.166.121`（通过环境变量 `ROBOT_IP` 使用，不写入 ROS 参数或标定 YAML）
- 目标分辨率：`640x360@30`
- 目标板型：`checkerboard 9x6`
- 若执行现场无法确认 `robot_id` 或 `camera_serial_or_sensorpair`，则禁止把 YAML 提交到本目录。

建议命名形式：

```text
<robot_id>_<camera_serial_or_sensorpair>_<width>x<height>_<yyyymmdd>.yaml
```

例如：

```text
elephant_imx219stereo_640x360_20260820.yaml
```

建议同时保存一份同名或同日期的标定报告到 `E:/a_robot/temp/deyes/reports/`，报告至少包含：

- `reproj_rms_px`
- `epipolar_p95_px`
- 有效样本数
- 删除样本原因统计
- `K1/D1/K2/D2/R/T/P1/P2/Q`
- 抽检图像的极线误差

本轮复用 `mercury_grasp/calibrate_stereo.py` 的棋盘格流程时，执行前必须确认：

- 标定板内角点规格为 `9 x 6`
- 打印比例为 `100%`
- 单个方格边长已经实测并记录到报告

在未完成实机盘点前，不要预先生成虚假的内参、外参或 `Q` 矩阵文件。

当前 `/home/elephant/mercury_grasp/config/stereo_calib.yaml` 仅作为链路联调占位参数，不应直接复制进本目录冒充物理标定结果。

## 校准契约（深度真源）

所有供 `cuda_stereo_depth_node` 使用的 YAML 必须含有 `calibration_id`、`robot_id`、
`camera_pair_id`、`img_size`、`board_inner_corners: [9, 6]`、`square_size_m`、
`reproj_rms_px`、`epipolar_p95_px`、`date`、`source` 与 `validated`。物理标定前上述测量
字段必须保留为 `null`，且 `validated: false`；不得填入推导或猜测的误差数值。只有
`source: physical_checkerboard` 的实测 YAML 才可以设置 `validated: true`。

`validated: true` 时运行分辨率必须严格等于 `img_size`（本轮为 `640x360`）。未验证 YAML 仅可用于 debug，节点会
按 `scale_x/scale_y` 缩放原始 K 后重新计算校正映射，不能作为抓取距离真源。

> 历史记录：该占位参数曾从 `192.168.137.17` 备份到本目录 `stereo_calib.yaml`，只用于链路联调；并非当前机器人或物理标定结果。
> 注意：它是 `spec-based` 理论值（`fx=fy=864.91`、`baseline=0.06m`、`R=单位阵`、`D=0`），
> **不是物理标定结果**，不能作为可信测距/抓取的标定依据。
