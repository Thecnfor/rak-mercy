# Isaac 双笔运输候选桥（仅仿真）

`sim_dual_pen_candidate` 将 RGB 派生的笔特征、Isaac 深度、同帧 `CameraInfo`、桌面平面和 TF
严格汇合，发布 **恰好两支** `table_1` 笔的抓取候选。它用于今晚的任务：机器人抵达黄色抓取位后，
双臂从 ① 桌面各取一支笔，再运输到 ② 桌面。

这个节点不能替代 `pen_grasp`，也不能输出可供实机执行的几何。所有成功消息都明确带有：

```json
{
  "source": "isaac_sim",
  "simulation_validated": true,
  "trusted_for_grasp": true,
  "trusted_for_grasp_scope": "simulation_only",
  "physical_validated": false,
  "physical_execution_eligible": false
}
```

## 输入契约

默认话题和世界校验值见 `config/stereo/sim_dual_pen_candidate.defaults.yaml`。默认值故意拒绝：
`expected_world_id`、`expected_scene_sha256`、`expected_seed` 和 `initial_scene_phase` 都为空/未绑定。
运行前必须从 **当前 Isaac 已加载 USD 的** `manifest.json` 复制 world ID、seed 和 USD SHA；不能复用
此前 v1 场景 SHA。当前夜间任务的初始相位必须显式设置为
`table_1_loaded_table_2_empty`，同时将 `assign_visible_pens_to_pickup_table:=true`。

`/left_camera/pen_features` 为 `std_msgs/String` JSON，必须是 RGB 派生结果，并包含：

```json
{
  "stamp_sec": 123,
  "stamp_nanosec": 456,
  "frame_id": "Left_Camera",
  "image_width": 1280,
  "image_height": 720,
  "simulation": {
    "source": "isaac_sim",
    "world_id": "team_rak_finals_20260820",
    "scene_sha256": "11d59b9fff96304d263d2d6df4e4958b876b30fe0a1b03ca461098944a419cd6",
    "seed": 20260820,
    "initial_scene_phase": "table_1_loaded_table_2_empty",
    "physical_validated": false,
    "physical_execution_eligible": false
  },
  "features": [{"label":"pen", "id":"pen_001", "source_table_id":"table_1", "axis_complete":true, "mask_pixels_px":[[1,2]], "axis_endpoints_px":[[1,2],[30,2]]}]
}
```

深度必须是 `/left_camera/depth` 的 `32FC1`（米），并和特征、CameraInfo、桌面平面严格
同 stamp、`Left_Camera`、1280×720。节点按该 stamp 查询 `base_link <- Left_Camera`。
桌面平面仍使用 `dynamic_table_plane_camera_relative_only` 契约，只作为笔/桌面分离证据。

以下任一项会拒绝整帧：世界 ID、scene SHA、seed、frame/stamp/size 不一致；任何 physical
claim；缺失 TF；退化平面；`table_1` 上不足两笔；或无法生成两个几何有效且 ID 唯一的候选。

## 启动

先由 Socl Isaac adapter 把原始 `sim_camera` 统一为 `Left_Camera`，并将 CameraInfo reheader
到 depth stamp。启动参数必须绑定实际场景；以下 `<>` 是当前 manifest 的值而非占位默认值：

```bash
ros2 run deyes_stereo sim_dual_pen_candidate --ros-args \
  --params-file <deyes_bringup_share>/config/stereo/sim_dual_pen_candidate.defaults.yaml \
  -p expected_world_id:=<manifest-world-id> \
  -p expected_scene_sha256:=<loaded-usd-sha256> \
  -p expected_seed:=<manifest-seed> \
  -p initial_scene_phase:=table_1_loaded_table_2_empty \
  -p assign_visible_pens_to_pickup_table:=true
```

输出 `/sim/grasp/dual_pen_candidates` 只能接入仿真版双臂规划/回放，适合为 ①→② 的运输链路
提供目标。不得把该话题重映射到实机 `pen_grasp` 或运动执行器。
