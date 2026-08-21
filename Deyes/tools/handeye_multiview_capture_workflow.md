# 手眼标定多姿态采集合同

`handeye_multiview_contract.py` 只检查物理采集证据；没有 ROS 命令发布、SDK、串口或机械臂动作路径。它将合格会话转换为既有 `handeye_calibration` 的点对应输入，最终外参仍须通过 RMS/P95、身份绑定和人工确认才可被抓取链使用。

每条采样要求：

- 相机侧：`left_camera_optical_frame` 中同一个、明确命名棋盘角点的 `camera_checkerboard_pose`（位置与四元数、时间戳）。
- 机械臂侧：同一物理角点的工具端 `base_link` 坐标、工具姿态、六轴关节反馈、左右臂标识与时间戳。
- 相机与末端时间戳差 `<=20 ms`；未同步即拒绝，不允许事后改写 stamp。

会话最低 8 条，要求相机和工具位置跨度均 `>=80 mm`、相机和工具姿态跨度均 `>=15°`，并拒绝近似重复视角。可混用左右臂，但每条只表示一只实际反馈的臂；不需要、也禁止双臂同时运动。

典型离线调用：

```python
from deyes_stereo.handeye_multiview_contract import build_handeye_multiview_session, build_handeye_solver_payload

session = build_handeye_multiview_session(operator_collected_json)
assert session["ready_for_solver"] and not session["validated"]
solver_input = build_handeye_solver_payload(session, operator_confirmation=True)
```

采集 JSON、session report、求解 YAML 均保存到仓库外 `E:/a_robot/temp/deyes/calibration/`。在 `handeye_calibration` 结果显示 `validated:true` 前，`pen_grasp` 仍会保持 `trusted_for_grasp:false`。
