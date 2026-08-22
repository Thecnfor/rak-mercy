# Socl_ous Isaac 感知桥（仿真专用）

已知运行环境为 ROS 2 Jazzy、domain 45：`/left_camera/rgb` 是
`1280x720 rgb8`、frame `Left_camera`；`/left_camera/depth` 是
`1280x720 32FC1`；`/left_camera/camera_info` 的 `P` 使用
`fx=fy=857.731902, cx=640, cy=360`，但 frame 为 `sim_camera` 且时间戳独立。

`socl_isaac_perception_contract.py` 是 ROS-free 的输入门禁。它只接收
`source=isaac_sim`，要求深度编码、尺寸、有限且有效的 12 项投影矩阵、时间偏差和
CameraInfo 新鲜度均通过。通过后，它把 **CameraInfo 的输出 header 精确重写为当前
depth header**，从而满足正式深度链关于 stamp/frame/size 的配对约束；原始 stamp、
frame 和偏差仍写入 metadata，不能丢失。

这不是物理标定桥：输出一律 `physical_validated=false`、
`command_consumption_allowed=false`、`physical_consumption_allowed=false`。因此它只能供
Isaac 的 RViz、算法回放和集成测试使用，不能接到真实抓取或作为实机标定证据。

建议 ROS 包装节点按每个 depth 帧调用：缓存最新 CameraInfo，传入接收时钟、50 ms 的
原始 stamp 偏差上限和 500 ms 新鲜度上限。失败时不要发布派生 CameraInfo，并同时发布
失败原因和原始 metadata。
