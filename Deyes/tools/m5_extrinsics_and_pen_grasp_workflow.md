# M5：相机到机器人外参与办公笔抓取候选

本流程将双目内部标定和 `left_camera_optical_frame -> base_link` 外参分开管理。前者只说明深度尺度；后者才允许发布可供双臂规划使用的抓取点。任何一项未验证时，笔节点只输出 camera-frame debug 信息，绝不输出 `grasp_point_base_m`。

## 现场前提与文件位置

- 先完成 `640x360@30` 物理双目标定、10 分钟稳态和四距离真值验收；使用的 stereo YAML 必须为 `source: physical_checkerboard` 且 `validated: true`。
- 固定相机支架、机器人本体和桌面后再开始外参；其间不得移动相机。
- 标定图片、对应点 JSON、候选 YAML 和报告全部放到 `E:/a_robot/temp/deyes/calibration/<session>/` 或机器人端同等 `temp/deyes` 路径。仓库只接收最终经审核的 YAML 和源码。
- 任何 SSH/实机操作仅从 shell 环境读取地址：`export ROBOT_IP=192.168.255.121`。该值不进入 ROS 参数、TF 或标定 YAML。

## 物理外参：点对应法

在固定桌面上放置具有至少六个可区分标记的板。对每个同一标记：

1. 记录其在 `left_camera_optical_frame` 的三维坐标（由物理标定后的深度、校正 CameraInfo 和稳健局部采样获得）。
2. 以低速、人工监护方式用已知 TCP 逐点触碰该标记，记录同一点的 `base_link` 坐标。
3. 点应覆盖至少 `0.08m` 的范围，不能共线；请覆盖桌面中心、四角和不同深度/高度位置。

输入 JSON（示例坐标是格式样例，不能上机复用）：

```json
{
  "calibration_id": "x1-<robot-id>-<pair-id>-handeye-YYYYMMDD",
  "robot_id": "<confirmed-robot-id>",
  "camera_pair_id": "<confirmed-camera-pair-id>",
  "stereo_calibration_id": "<validated-stereo-calibration-id>",
  "operator_confirmation": true,
  "correspondences": [
    {"camera_point_m": [0.10, -0.03, 0.55], "base_point_m": [0.42, 0.10, 0.21]}
  ]
}
```

填满至少六项真实对应点后，在目标机已 source ROS 环境的 shell 执行：

```bash
ros2 run deyes_stereo handeye_calibration \
  --input /home/elephant/temp/deyes/calibration/<session>/correspondences.json \
  --output /home/elephant/temp/deyes/calibration/<session>/base_link_T_left_camera.yaml
```

工具用 Kabsch 刚体求解 `base_point = R * camera_point + t`，且仅在对应点至少 6 个、跨距至少 `0.08m`、RMS `<=5mm`、P95 `<=10mm` 时写 `validated: true`。它还绑定 robot ID、camera pair ID 与 stereo calibration ID。由人工复测保留点的误差与方向后，才可将最终 YAML 归档/评审。

通过两份身份一致的 YAML 发布静态 TF：

```bash
ros2 launch deyes_bringup validated_extrinsics_tf.launch.py \
  extrinsics_path:=/path/to/base_link_T_left_camera.yaml \
  stereo_calibration_path:=/path/to/physical_stereo.yaml
```

失败时该节点发布 `/x1/stereo/extrinsics_status` 的 `trusted_for_grasp:false`，不会广播 TF。

## 当前单目标笔的输入与输出契约

`pen_grasp` 不修改也不依赖具体 YOLO 实现。上游的检测/分割组件向 `/x1/detection/pen_features` 发布 JSON，使用显式 mask 像素和二维长轴端点：

```json
{
  "stamp_sec": 0,
  "stamp_nanosec": 0,
  "features": [
    {"id": "pen-left", "label": "pen", "confidence": 0.92, "axis_complete": true,
     "mask_pixels_px": [[120, 88], [121, 88]],
     "axis_endpoints_px": [[116, 88], [198, 91]]}
  ]
}
```

实际 `mask_pixels_px` 每支至少 12 点，且上游必须发送 `axis_complete: true`。节点以 mask 内深度的中值/MAD 稳健采样，并使用 `/x1/ground/plane` 的 RANSAC 桌面平面剔除桌面像素；再输出每支的 `target_id`、camera-frame 位置、轴、端点、40% 中段抓取区间及质量项。任何 mask 或轴端点进入图像边缘 `12px` 区，或 `axis_complete` 不为 true 时，输出 `target_visibility: edge_truncated/unknown_or_incomplete` 并关闭抓取。物理外参和双目标定均验证后，才增加同一候选的 `grasp_point_base_m`、`axis_base_unit`、`approach_normal_base_unit` 与 `grasp_interval_base_m`。

- 0 支：`reason: waiting/no_target`；
- 恰好 1 支：输出该 `target_id` 的候选；
- 超过 1 支、重复 ID 或几何无法区分：`ambiguous_multi_target` 或 `geometric_conflict_or_indistinguishable`，关闭全部 base-frame 抓取点，绝不按置信度选择一支。

启动（应同时启动 CUDA 深度、校正 CameraInfo 与 `ground_plane`）：

```bash
ros2 launch deyes_bringup pen_grasp.launch.py \
  extrinsics_path:=/path/to/base_link_T_left_camera.yaml \
  stereo_calibration_path:=/path/to/physical_stereo.yaml
```

未完成现场标定时仍可将两个路径保留为空做画面/深度 pilot；候选会显式标记 `trusted_for_grasp:false`，只能用于 camera-frame 调试。

## 动态桌面平面不是抓取坐标系

`ground_plane` 只消费 `/x1/stereo/depth` 和同 stamp、同尺寸、同 frame 的
`/x1/stereo/left/camera_info_rect`；反投影只使用该校正 CameraInfo 的
`P[0]`、`P[5]`、`P[2]`、`P[6]`。它输出相机相对的桌面法向、中心、距离、内点率、
RMS/P95 残差与相邻帧法向变化，供桌面剔除和相对高度判断。

动态平面绝不替代 `base_link_T_left_camera`：默认不发布 TF。若人工排障显式设置
`ground_plane_publish_debug_tf:=true`，输出帧仍名为 `table_plane_dynamic_debug`，只可在
RViz 调试，状态和数据都会标记 `dynamic_table_plane_camera_relative_only` 及
`trusted_for_grasp:false`。法向突变时节点只发布 `degraded` 的旧平面回退，
`valid_for_table_removal:false`；笔抓取节点将拒绝该帧。
