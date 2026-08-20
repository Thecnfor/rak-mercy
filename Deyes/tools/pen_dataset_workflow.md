# 办公用笔：采集、标注与训练流程

目标是建立一个仅识别 `pen` 的单类检测模型。COCO 预训练模型没有可靠的通用“办公笔”类别，因此不能把近似类别当作比赛识别结果；应以采集到的实机画面做迁移训练。

## 1. 采集

数据和训练产物绝不能放入仓库。建议在 Jetson 创建并使用：

```bash
mkdir -p /home/elephant/temp/deyes/datasets/pen
ros2 run deyes_stereo pen_dataset_capture --ros-args \
  -p output_dir:=/home/elephant/temp/deyes/datasets/pen \
  -p image_topic:=/x1/stereo/debug/left_rect \
  -p min_interval_sec:=0.50 \
  -p max_images:=400 \
  -p jpeg_quality:=95
```

Windows 本地整理目录对应 `E:/a_robot/temp/deyes/datasets/pen`。`output_dir` 必须是绝对路径且在 `rak-mercy` 外；节点会拒绝相对路径和仓库内目录。每次运行建立一个 `pen_...` 会话目录，包含：

- `images/*.jpg`：原始采集图像；
- `session_manifest.jsonl`：每张图的原始 ROS stamp、frame、宽高、编码、清晰度和与上一张已保存图的差异；
- `session_summary.json`：保存数、按最小间隔跳过数、解码失败数及结束原因。

`min_interval_sec` 是唯一自动取样策略。清晰度和近重复差异只做报告，**不会自动删除或丢弃已经保存的原始证据**。`max_images=0` 表示不设上限；按 `Ctrl-C` 会落盘并安全写出 summary。

建议以“摆放批次”为单位采集，而不是连续挪动同一支笔：桌面/背景、笔颜色和型号、横竖/倾角、遮挡、光照、距离（尤其 0.3–1.0m）各自形成独立批次。先收集 10–20 个批次，每批 20–50 张经间隔抽样的不同姿态图；也要采集无笔和易混淆物（尺子、线缆、筷子、工具）。

## 2. 标注与划分

用 CVAT、Label Studio 或 Roboflow 标注每支可抓取笔的可见外接框。标签必须是 YOLOv5 单类格式：

```text
<class_id> <x_center> <y_center> <width> <height>
0 0.516 0.432 0.387 0.064
```

建立 `data.yaml`：

```yaml
path: /home/elephant/temp/deyes/datasets/pen/yolo
train: images/train
val: images/val
test: images/test
names: [pen]
```

必须按**摆放批次**（而非随机逐帧）划分：建议 train/val/test = 70/15/15。一个批次的所有连续帧、相同背景及同一次摆放只能进入一个 split，否则验证指标会因泄漏而虚高。保留每张图的 session/batch 对应表，不要移动或改写原始 session。

## 3. YOLOv5 训练、ONNX 与 TensorRT

训练环境和模型文件也在仓库外，例如 `/home/elephant/temp/deyes/models/pen`。以已经安装的 YOLOv5 环境为例：

```bash
python3 train.py --img 640 --batch 16 --epochs 150 \
  --data /home/elephant/temp/deyes/datasets/pen/yolo/data.yaml \
  --weights yolov5s.pt --name pen_yolov5s

python3 export.py --weights runs/train/pen_yolov5s/weights/best.pt \
  --include onnx --imgsz 640 640 --simplify
```

固定姿态 pilot 只用于检查视场、曝光和采集链，不得进入任何 split。只有笔完整入画、姿态/摆放批次改变后的会话才可标注。双笔图每张写两行 `class_id=0` 标签。

先以保留的 `test` 批次评估，再测试比赛场景中的笔。正式目标为 Precision 和 Recall 均 `>= 0.95`，并报告端到端 3D 候选成功率；不只看训练集或连续帧验证集。

Jetson TensorRT engine 与设备、TensorRT/CUDA 版本绑定。将 ONNX 拷到 Jetson 后，必须用其 **本机 TensorRT 8.5** 构建，不提交 `.pt`、`.onnx` 或 `.engine` 到 Git：

```bash
/usr/src/tensorrt/bin/trtexec \
  --onnx=/home/elephant/temp/deyes/models/pen/best.onnx \
  --saveEngine=/home/elephant/temp/deyes/models/pen/best.engine \
  --fp16
```

导出前后分别以同一批真实图做推理对比，并在目标 Jetson 上记录 TensorRT、CUDA、模型 SHA-256、输入尺寸和置信度/NMS 阈值。若输入包含 letterbox，标签和反投影坐标必须使用相同的缩放与 padding 规则。

在模型目录（仓库外）建立 `model_manifest.json`，最少记录：`model_id`、`engine_sha256`、ONNX SHA-256、训练数据版本、Jetson/TensorRT/CUDA 版本、输入 `[1,3,640,640]`、输出 `[1,N,6]`、`class_names={"0":"pen"}`、阈值和创建时间。启动时必须把 `model_id`、engine SHA-256、类别数和双目标上限显式传给节点；缺少或不匹配会 fail-closed：

```bash
ros2 launch deyes_bringup imx219_stereo.launch.py \
  enable_cuda_depth:=true \
  enable_detector:=true \
  detector_config:=/path/to/pen_detector.defaults.yaml \
  detector_model_path:=/home/elephant/temp/deyes/models/pen/best.engine \
  detector_model_id:=pen_yolov5n_v1 \
  detector_expected_model_sha256:=<64-char-engine-sha256> \
  detector_expected_class_count:=1 \
  detector_expected_max_targets:=2 \
  detector_image_topic:=/x1/stereo/debug/left_rect \
  enable_object_fusion:=true
```

`/x1/detection/boxes` 中 1–2 个笔候选按从左到右稳定标记为 `target_00`、`target_01`。超过两个、IoU `>=0.80` 的疑似重复框、模型 SHA-256 不一致或 engine 输出不匹配时，节点会输出 `ambiguous` 并不给融合/自动抓取链候选。

## 4. 持出集评估与 3D 候选证据

在仓库外建立 JSONL：完整标注索引每行至少含 `image_id`、`batch_id`、`split`（`train`/`val`/`test`）和 `boxes`（`[x0,y0,x1,y1]` 列表）；同一 `batch_id` 只能属于一个 split。运行时推理记录仅覆盖 `test` 图片，且每行含同一 `model_id`、`model_sha256`、`detections` 和从融合节点记录的 `objects_3d`（每个成功对象有 `status:"ok"` 与 `bbox_xyxy`）。

```bash
PYTHONPATH=/home/elephant/deyes_ws/src/Deyes/src/deyes_stereo \
python3 -m deyes_stereo.pen_evaluation \
  --ground-truth /home/elephant/temp/deyes/datasets/pen/ground_truth_index.jsonl \
  --predictions /home/elephant/temp/deyes/datasets/pen/test_runtime_predictions.jsonl \
  --report /home/elephant/temp/deyes/reports/pen_test_report.json
```

该工具在批次泄漏、缺失 test 推理、混用模型身份或无效 bbox 时拒绝报告；输出 Precision、Recall 与按真值笔数统计的 `end_to_end_3d_candidate.success_rate`。
