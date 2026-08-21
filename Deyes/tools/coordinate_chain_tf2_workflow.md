# Mercury X1 坐标链：同一 TF2 接口、物理门禁

坐标图唯一入口是 ROS 2 TF2：`left_camera_optical_frame -> base_link -> <官方实际的 left/right arm base、tool、gripper frame>`。仿真与实机均通过同一个 `lookup_transform(target, source, stamp)`；仿真场景变换只能用于仿真验证，绝不能发布为物理手眼标定或令执行器放行。

仓库当前没有 Mercury X1 官方 URDF/Xacro 副本。因此不猜测 `left_tool_frame`、`right_gripper_frame` 等名字或静态尺寸。现场必须从已启动的官方 robot_state_publisher/驱动确认实际 frame 名称，填入 `required_end_effector_frames`（左右 arm base、tool/TCP、gripper 全部列出）并作为请求的 `target_frame`。`tf_chain_audit` 在 `/x1/coordinate_chain/tf_audit` 发布缺失 frame；配置为空也会明确报 invalid。若任一 frame 未出现在 TF2 树，转换网关拒绝该请求；不得添加手填静态 TF。

## 发布和查询

先只对经物理验证、身份一致的外参发布相机挂载 TF：

```bash
ros2 launch deyes_bringup validated_extrinsics_tf.launch.py \
  extrinsics_path:=/path/to/base_link_T_left_camera.yaml \
  stereo_calibration_path:=/path/to/physical_stereo.yaml
ros2 launch deyes_bringup coordinate_chain_tf2.launch.py
```

第二个节点订阅 `/x1/stereo/extrinsics_status`，在其明确包含 `trusted_for_grasp:true`、`physical_validated:true`、`tf_published:true` 前不输出结果。它不创建 action、串口、机械臂或夹爪客户端。请求为 JSON/String：

```json
{"kind":"pose","source_frame":"left_camera_optical_frame","target_frame":"<official-left-tool-frame>","stamp_ns":0,"position_m":[0.1,0.0,0.5],"quaternion_xyzw":[0,0,0,1]}
```

输出仍保留输入 `source_frame`/`target_frame`，并增加 `frame_id`（目标 frame）、`transform_interface:"tf2"` 和 `trusted_for_execution:true`。下游执行器必须同时要求后两个字段，缺失或 false 一律拒绝。

## RViz / 现场验收（不移动机器人）

1. 启动官方 Mercury X1 的描述/驱动和 TF 后运行 `ros2 run tf2_tools view_frames`；在生成的图中确认相机到 `base_link` 只有验证节点发布的边，且左右 arm base、tool、gripper 都经官方 URDF 连到 `base_link`。
2. `ros2 topic echo --once /x1/stereo/extrinsics_status` 必须显示三个 true；空路径、`validated:false`、仿真来源或身份不匹配必须显示 `trusted_for_grasp:false` 且没有该 TF。
3. RViz 固定坐标设为 `base_link`，添加 TF 和 Pose；发送一个**不交给执行器**的网关请求，确认 pose 在对应左/右工具 frame 可见。对同一 stamp 执行 `ros2 run tf2_ros tf2_echo <official-tool-frame> left_camera_optical_frame`，结果应与网关一致。
4. 以记录但不运动的多姿态采样生成 session；只有手眼 YAML 与 stereo YAML 都审查通过后，才允许第三步的 `trusted_for_execution:true`。现场不完成这些步骤时，系统保持 fail-closed。
