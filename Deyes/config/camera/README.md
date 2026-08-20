# Camera Config Notes

提交到仓库的相机标定文件需要同时绑定以下信息：

- 机器人编号或机身唯一标识
- 相机序列号
- 左右目分辨率与帧率
- 标定日期
- 生成标定时所用代码提交号

本轮 `M2+M3` 约定：

- 目标主机：`192.168.0.121`
- 目标分辨率：`1280x720@30`
- 目标板型：`checkerboard 9x6`
- 若执行现场无法确认 `robot_id` 或 `camera_serial_or_sensorpair`，则禁止把 YAML 提交到本目录。

建议命名形式：

```text
<robot_id>_<camera_serial_or_sensorpair>_<width>x<height>_<yyyymmdd>.yaml
```

例如：

```text
elephant_imx219stereo_1280x720_20260818.yaml
```

建议同时保存一份同名或同日期的标定报告到 `E:/a_robot/temp/deyes/reports/`，报告至少包含：

- `reproj_error`
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

> 已从现场 `192.168.137.17` 备份该占位参数到本目录 `stereo_calib.yaml`，用于现场恢复与链路联调。
> 注意：它是 `spec-based` 理论值（`fx=fy=864.91`、`baseline=0.06m`、`R=单位阵`、`D=0`），
> **不是物理标定结果**，不能作为可信测距/抓取的标定依据。
