# Deyes 模型离线资产

本目录用于保存 `Deyes` 在比赛现场机上需要用到、且希望离线保留的模型资产，避免现场网络不稳定导致模型下载失败。

## 当前包含

- `yolov5s.onnx`：YOLOv5s 源模型（COCO 80 类，29.3MB）。
- `yolov5s.engine`：由 `yolov5s.onnx` 通过 `TensorRT 8.5.2.2` 在 Jetson Xavier NX 上构建得到，供 `deyes_stereo/yolo_detector_node` 的 `tensorrt` 后端使用。

## 生成方式

在目标机上，使用仓库根目录的 `remote_build_yolo_engine.py`。连接信息通过运维环境变量传入，禁止在命令或脚本中保存密码：

```bash
export ROBOT_IP=192.168.166.121
export ROBOT_USER=elephant
python remote_build_yolo_engine.py \
  --host "$ROBOT_IP" \
  --username "$ROBOT_USER" \
  --onnx-path /home/elephant/mystudio/resources/machine_vision/yolov5s.onnx \
  --engine-out /home/elephant/deyes_ws/yolov5s.engine \
  --local-out depends/models/yolov5s.engine \
  --workspace-mb 1024
```

认证应使用 SSH key 或交互式提示；不得传入 `--password`、保存明文密码或把 IP 写入算法配置。

## 现场使用

把 `yolov5s.engine` 同步到 Jetson 后，在 `imx219_stereo.launch.py` 中启用检测：

```bash
ros2 launch deyes_bringup imx219_stereo.launch.py \
  enable_cuda_depth:=true \
  enable_depth_coordinate:=true \
  enable_detector:=true \
  detector_backend:=tensorrt \
  detector_model_path:=/home/elephant/temp/deyes/models/yolov5s.engine \
  enable_object_fusion:=true
```

## 注意

- `tensorrt` 后端的 `model_path` 必须指向 `.engine` 文件，不能指向 `.onnx`。
- 当前 Jetson 未安装 `pycuda`/`cuda-python`，`yolo_detector_node` 使用 `torch.cuda` 作为 binding buffer，无需额外安装 `pycuda`。
