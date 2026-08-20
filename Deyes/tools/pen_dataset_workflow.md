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

先以保留的 `test` 批次评估，再测试比赛场景中的笔。推荐同时报告 per-image precision、recall、mAP@0.5、漏检率和误检率，不只看训练集或连续帧验证集。

Jetson TensorRT engine 与设备、TensorRT/CUDA 版本绑定。将 ONNX 拷到 Jetson 后，必须用其 **本机 TensorRT 8.5** 构建，不提交 `.pt`、`.onnx` 或 `.engine` 到 Git：

```bash
/usr/src/tensorrt/bin/trtexec \
  --onnx=/home/elephant/temp/deyes/models/pen/best.onnx \
  --saveEngine=/home/elephant/temp/deyes/models/pen/best.trt \
  --fp16
```

导出前后分别以同一批真实图做推理对比，并在目标 Jetson 上记录 TensorRT、CUDA、模型 SHA-256、输入尺寸和置信度/NMS 阈值。若输入包含 letterbox，标签和反投影坐标必须使用相同的缩放与 padding 规则。
