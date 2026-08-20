# YOLO + Depth Coordinate Fusion Feasibility

## Goal

- 在双目主链已经稳定输出 `/x1/stereo/depth` 的前提下，引入目标检测模型，对目标进行 `2D 检测 + 深度融合 + base_link 坐标输出`。
- 优先目标不是“先把 UI 做漂亮”，而是先做一条在 Jetson Xavier NX 上能跑、能同步、能输出可用坐标的生产链路。

## Current Base

- 左目图像：`/x1/left_camera/image_raw`
- 深度图：`/x1/stereo/depth`
- 左目内参：`/x1/left_camera/camera_info`
- 基座系热力图：`/x1/stereo/base_heatmap`
- 坐标化参考实现：`src/deyes_stereo/deyes_stereo/depth_coordinate_node.py`
- 主启动入口：`src/deyes_bringup/launch/imx219_stereo.launch.py`

## Feasibility Conclusion

- 结论：可行，且推荐作为下一阶段主线。
- 约束：不建议在 Jetson Xavier NX 上长期直接跑 PyTorch 原生 YOLO 作为正式链路。
- 推荐方案：TensorRT engine 推理 + ROS2 融合节点。

## Jetson 192.168.137.17 Evidence

- 实机型号：`Jetson Xavier NX`
- JetPack 版本：`R35.3.1`
- Python 环境探测结果：
  - `torch = 2.0.0+nv23.05`
  - `tensorrt = 8.5.2.2`
  - `ultralytics = missing`
  - `onnxruntime = missing`
  - `cv2 = 4.2.0`
- 现成模型文件：
  - `/home/elephant/mystudio/resources/machine_vision/yolov5s.onnx`
- 已完成的实测结论：
  - `TensorRT 8.5.2.2` 可以成功 `parse` 这份 `yolov5s.onnx`
  - 网络输入为 `images: [1, 3, 640, 640]`
  - 网络输出为 `output0: [1, 25200, 85]`
- 当前明确失败点：
  - `OpenCV 4.2.0` 的 `cv2.dnn.readNetFromONNX()` 无法导入该模型
  - 失败报错来自 `onnx_importer.cpp` 的 `getMatFromTensor` 断言

## Actual Recommendation On This Machine

- 对 `192.168.137.17` 来说，最短正式路线不是 `opencv_dnn`
- 当前最合适的正式路线是：
  - `TensorRT engine` 作为 detector 主推理后端
  - `ROS2 yolo_detector_node` 发布 `/x1/detection/boxes`
  - `object_fusion_node` 继续负责深度融合与 `base_link` 坐标输出
- 若需要临时验证：
  - 可以补装 `ultralytics` 作为过渡方案
  - 但不建议把 `ultralytics/PyTorch` 作为比赛现场正式链路

## Deployment Options

### Option A: PyTorch / Ultralytics

- 优点：
  - 开发最快
  - 调试和导出最方便
- 缺点：
  - Jetson Xavier NX 上持续运行的吞吐和时延不稳定
  - Python 端推理占用偏高
- 适用：
  - 只做离线验证或算法初版

### Option B: ONNX Runtime

- 优点：
  - 比 PyTorch 更轻
  - 导出和接口比较标准
- 缺点：
  - 在 Jetson 上通常不如 TensorRT 稳
  - 后续还要再走一轮性能优化
- 适用：
  - 中间验证阶段
- 本机状态：
  - 当前 `onnxruntime` 未安装，不是最短路径

### Option B-2: OpenCV DNN + ONNX

- 优点：
  - 依赖面相对小
  - 可以复用现有 `cv2` 运行环境
- 缺点：
  - 强依赖本机 Python `cv2` 版本和 ONNX importer 兼容性
  - 当前 `192.168.137.17` 上的 `cv2 4.2.0` 已被实测证明无法正确导入 `yolov5s.onnx`
- 适用：
  - 仅当 Python `cv2` 升级到兼容版本后再考虑

### Option C: TensorRT Engine

- 优点：
  - 最符合 Jetson / NVIDIA 主线
  - 时延、吞吐、显存占用最可控
  - 适合比赛现场与长期运行
- 缺点：
  - 模型导出、算子兼容、engine 构建更复杂
- 适用：
  - 正式部署

## Recommended Architecture

### Detector Node

- 节点名建议：`deyes_detector_node`
- 输入：
  - `/x1/left_camera/image_raw`
- 输出：
  - `/x1/detection/boxes`
  - `/x1/detection/debug_image`
- 输出内容建议至少包含：
  - `class_id`
  - `class_name`
  - `confidence`
  - `bbox_xyxy`
  - `stamp`
  - `frame_id`

### Fusion Node

- 节点名建议：`deyes_object_fusion_node`
- 输入：
  - `/x1/detection/boxes`
  - `/x1/stereo/depth`
  - `/x1/left_camera/camera_info`
  - TF `base_link <- left_camera_optical_frame`
- 输出：
  - `/x1/detection/objects_3d`
  - `/x1/detection/objects_debug`
- 输出内容建议至少包含：
  - `class_name`
  - `confidence`
  - `bbox_xyxy`
  - `center_px`
  - `center_camera_m`
  - `center_base_m`
  - `depth_median`
  - `valid_ratio`
  - `status`

## Depth Fusion Method

- 不直接使用 bbox 中心点单像素深度，避免噪声过大。
- 推荐方法：
  - 在 bbox 内缩小 ROI
  - 仅保留 `0.20-1.00m` 的有效深度
  - 对 ROI 有效点做中位数和 MAD 去噪
  - 取稳定深度值做反投影
- 若有效点比例过低：
  - 输出 `status=low_depth_confidence`
  - 不给出最终坐标，避免误抓取

## Coordinate Computation

- 使用左目内参 `(fx, fy, cx, cy)`：
  - `X = (u - cx) * Z / fx`
  - `Y = (v - cy) * Z / fy`
  - `Z = depth`
- 先得到 `left_camera_optical_frame` 下坐标，再通过 TF 转到 `base_link`
- 这部分可直接复用 `depth_coordinate_node.py` 里的投影和 TF 处理逻辑

## Launch Integration

- 在 `src/deyes_bringup/launch/imx219_stereo.launch.py` 中新增：
  - `enable_detector`
  - `enable_object_fusion`
  - `detector_engine_path`
  - `detector_conf_threshold`
  - `detector_input_width`
  - `detector_input_height`
- 推荐结构：
  - 深度链保持现状
  - 检测与融合作为并行节点挂载

## Recommended Rollout

### Phase 1

- 保持当前深度链稳定
- 先做 YOLO 离线推理验证
- 输入本地左目图片，验证目标检测准确性

### Phase 2

- 接入 ROS2 `deyes_detector_node`
- 输出 2D 检测框和 debug image

### Phase 3

- 接入 `deyes_object_fusion_node`
- 实现 bbox ROI 深度融合与 `base_link` 坐标输出

### Phase 4

- 在 `admin_gui` 加一个目标检测调试页
- 同屏显示：
  - 左目图
  - 检测框
  - 深度估计
  - 基座系坐标

## Acceptance Criteria

- 检测节点持续在线
- 目标框可在左目图上稳定显示
- 常见目标能够输出稳定的 `base_link` 坐标
- 深度有效点比例过低时能够拒绝输出错误坐标
- 在 Jetson Xavier NX 上运行时不破坏当前深度链稳定性

## Recommendation

- 正式路线选择 `TensorRT + ROS2 独立融合节点`
- 不建议把 YOLO 主推理塞进 `admin_gui/backend/server.py`
- 不建议把 YOLO 和 `depth_coordinate_node.py` 混成一个节点
- 推荐新增独立包：
  - `src/deyes_object_fusion/`
- 对 `192.168.137.17` 的下一步建议：
  - 先用 TensorRT Python 或 engine 文件把 detector 真正跑通
  - 再接入 `/x1/detection/boxes -> /x1/detection/objects_3d`
