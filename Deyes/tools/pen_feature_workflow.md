# YOLO 框到办公笔像素特征

`pen_feature` 是一个可选的、默认关闭的 ROS 2 节点。它只消费与深度几何一致的校正左图
`/x1/stereo/debug/left_rect`，以及 `/x1/detection/boxes` 的 JSON；它不读取 raw 左图。

它只在检测 JSON 的 `stamp_sec/stamp_nanosec` 与图像 header stamp **完全相等**，且
`frame_id`、`image_width`、`image_height` 完全一致时处理。两侧消息可乱序到达；缓存默认
8 项、0.50 秒，超时的未配对消息直接丢弃，不存在近似时间匹配或“取最新图”的回退。

节点会发布 `/x1/detection/pen_features`，格式符合 `m5_extrinsics_and_pen_grasp_workflow.md`
中的 `pen_grasp` 输入契约，并保留检测 stamp、frame、图像尺寸、`target_id`、`det_index`
和 confidence。`/x1/detection/pen_features_status` 始终给出拒绝原因。

## 运行门禁

- 0 个 `pen`：`waiting_for_one_pen`；
- 恰好 1 个 `pen`：运行确定性经典 CV 像素提取；
- 多于 1 个 `pen` 或上游 already ambiguous：`ambiguous_multi_target`，不按 confidence 选取；
- stamp/frame/size 不匹配、过期、无细长组件或边缘截断：不产生可抓取候选。

像素提取仅在 YOLO bbox 的局部区域工作：局部背景差分与 Canny 边缘、形态学闭合、连通域
筛选（细长度与中心约束），最后用 PCA 计算长轴。`axis_complete` 只有在掩码面积、长度、
细长度、PCA 主/次轴比及图像边缘门禁全部通过时才为 true。可提取但不合格的细长度/方向
会发布 `axis_complete:false`；没有满足最小掩码像素的组件则不发布 feature，只给 status，
因为它不能满足 `pen_grasp` 的最少 12 像素输入契约。`mask_pixels_px` 为了消息大小可
确定性限到 `max_mask_pixels`，但始终覆盖掩码的首尾扫描位置。

这是一条几何候选链路，而不是经过像素真值验证的分割模型；离线报告只能称“几何可用率”
和轴方向一致性，不能宣称 segmentation accuracy。

## 启动与离线 QA

```bash
ros2 launch deyes_bringup imx219_stereo.launch.py \
  enable_detector:=true enable_pen_features:=true
```

在线启动前须确认 detector 的 `detector_image_topic` 与 `pen_feature_image_topic` 都是同一
校正左图话题。参数在 `config/stereo/pen_feature.defaults.yaml` 中，所有阈值显式可审计。

使用已经人工标注的 bbox 做离线几何 QA（不会衡量 detector 精度，也不会改数据）：

```powershell
python Deyes/tools/pen_feature_offline_qa.py `
  --ground-truth E:/a_robot/temp/deyes/datasets/pen/annotation_v2/ground_truth_index.jsonl `
  --output E:/a_robot/temp/deyes/reports/pen_features_qa.json
```

报告按 capture batch 隔离统计正样本的 `axis_complete` 几何可用率和横/竖/斜方向一致性，
并列出失败图片。当前“near-vertical”摆放以相机像素 `90° ±35°` 判定（透视下实拍约为
60--70°）；该阈值会写入 QA 脚本，不能误读为世界坐标角度。空桌面及多实例图片会单独
记为“不适用”，不会被伪装为单笔成功。
