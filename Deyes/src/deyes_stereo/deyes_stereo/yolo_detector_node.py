from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .yolo_detector_contract import (
    filter_detections_by_allowed_class_ids,
    gate_target_detections,
    normalize_tensorrt_dtype_name,
    parse_allowed_class_ids_json,
    TensorRTBinding,
    TensorRTEngineContract,
    normalize_yolov8_predictions,
    validate_tensorrt_yolov5_contract,
    validate_tensorrt_yolov8_contract,
    verify_model_sha256,
    validate_roi,
)


COCO80_CLASS_NAMES = {
    0: "person",
    1: "bicycle",
    2: "car",
    3: "motorcycle",
    4: "airplane",
    5: "bus",
    6: "train",
    7: "truck",
    8: "boat",
    9: "traffic_light",
    10: "fire_hydrant",
    11: "stop_sign",
    12: "parking_meter",
    13: "bench",
    14: "bird",
    15: "cat",
    16: "dog",
    17: "horse",
    18: "sheep",
    19: "cow",
    20: "elephant",
    21: "bear",
    22: "zebra",
    23: "giraffe",
    24: "backpack",
    25: "umbrella",
    26: "handbag",
    27: "tie",
    28: "suitcase",
    29: "frisbee",
    30: "skis",
    31: "snowboard",
    32: "sports_ball",
    33: "kite",
    34: "baseball_bat",
    35: "baseball_glove",
    36: "skateboard",
    37: "surfboard",
    38: "tennis_racket",
    39: "bottle",
    40: "wine_glass",
    41: "cup",
    42: "fork",
    43: "knife",
    44: "spoon",
    45: "bowl",
    46: "banana",
    47: "apple",
    48: "sandwich",
    49: "orange",
    50: "broccoli",
    51: "carrot",
    52: "hot_dog",
    53: "pizza",
    54: "donut",
    55: "cake",
    56: "chair",
    57: "couch",
    58: "potted_plant",
    59: "bed",
    60: "dining_table",
    61: "toilet",
    62: "tv",
    63: "laptop",
    64: "mouse",
    65: "remote",
    66: "keyboard",
    67: "cell_phone",
    68: "microwave",
    69: "oven",
    70: "toaster",
    71: "sink",
    72: "refrigerator",
    73: "book",
    74: "clock",
    75: "vase",
    76: "scissors",
    77: "teddy_bear",
    78: "hair_drier",
    79: "toothbrush",
}


@dataclass
class ImageFrame:
    stamp_ns: int
    frame_id: str
    width: int
    height: int
    bgr: np.ndarray


def stamp_ns_from_msg(msg: Image) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def image_msg_to_bgr(msg: Image) -> np.ndarray:
    if msg.encoding == "bgr8":
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step // 3, 3)
        return row[:, : msg.width, :].copy()
    if msg.encoding == "rgb8":
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step // 3, 3)
        return cv2.cvtColor(row[:, : msg.width, :], cv2.COLOR_RGB2BGR)
    if msg.encoding == "mono8":
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        gray = row[:, : msg.width]
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    raise RuntimeError(f"unsupported image encoding: {msg.encoding}")


def bgr_to_image_msg(image: np.ndarray, *, frame_id: str, stamp_ns: int) -> Image:
    msg = Image()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
    msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
    msg.height = int(image.shape[0])
    msg.width = int(image.shape[1])
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = int(image.shape[1] * 3)
    msg.data = image.astype(np.uint8).tobytes()
    return msg


def compact_float(value: float) -> float:
    return round(float(value), 4)


def letterbox(image: np.ndarray, target_width: int, target_height: int) -> tuple[np.ndarray, float, tuple[float, float]]:
    src_h, src_w = image.shape[:2]
    scale = min(float(target_width) / float(src_w), float(target_height) / float(src_h))
    resized_w = max(1, int(round(src_w * scale)))
    resized_h = max(1, int(round(src_h * scale)))
    resized = cv2.resize(image, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((target_height, target_width, 3), 114, dtype=np.uint8)
    pad_x = (target_width - resized_w) * 0.5
    pad_y = (target_height - resized_h) * 0.5
    left = int(round(pad_x - 0.1))
    top = int(round(pad_y - 0.1))
    canvas[top : top + resized_h, left : left + resized_w] = resized
    return canvas, scale, (float(left), float(top))


def trt_dtype_to_torch_dtype(trt_module: Any, torch_module: Any, dtype: Any) -> Any:
    mapping = {
        trt_module.float32: torch_module.float32,
        trt_module.float16: torch_module.float16,
        trt_module.int32: torch_module.int32,
        trt_module.int8: torch_module.int8,
        trt_module.bool: torch_module.bool,
    }
    if dtype not in mapping:
        raise RuntimeError(f"unsupported tensorrt dtype: {dtype}")
    return mapping[dtype]


class YoloDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_detector_node")

        defaults = {
            "image_topic": "/x1/stereo/debug/left_rect",
            "output_topic": "/x1/detection/boxes",
            "status_topic": "/x1/detection/boxes_status",
            "debug_image_topic": "/x1/detection/debug_image",
            "publish_period_sec": 0.12,
            "backend": "ultralytics",
            "model_path": "",
            "device": "cuda:0",
            "conf_threshold": 0.35,
            "iou_threshold": 0.45,
            "input_width": 640,
            "input_height": 640,
            "max_detections": 20,
            "publish_debug_image": False,
            "class_names_json": "{}",
            "allowed_class_ids_json": "[]",
            "model_id": "",
            "expected_model_sha256": "",
            "expected_class_count": 80,
            "tensorrt_output_layout": "yolov5",
            "expected_max_targets": 0,
            "duplicate_iou": 0.80,
            "run_mode": "continuous",
            "release_backend_after_inference": False,
            "roi_x": 0, "roi_y": 0, "roi_width": 0, "roi_height": 0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._frame: Optional[ImageFrame] = None
        self._last_processed_stamp_ns = -1
        self._last_status_key = ""
        self._backend_name = str(self.get_parameter("backend").value).strip().lower()
        self._model_path = str(self.get_parameter("model_path").value).strip()
        self._device = str(self.get_parameter("device").value).strip()
        self._conf_threshold = float(self.get_parameter("conf_threshold").value)
        self._iou_threshold = float(self.get_parameter("iou_threshold").value)
        self._input_width = int(self.get_parameter("input_width").value)
        self._input_height = int(self.get_parameter("input_height").value)
        self._max_detections = int(self.get_parameter("max_detections").value)
        self._publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self._model_id = str(self.get_parameter("model_id").value).strip()
        self._expected_model_sha256 = str(
            self.get_parameter("expected_model_sha256").value
        ).strip()
        self._expected_class_count = int(self.get_parameter("expected_class_count").value)
        self._tensorrt_output_layout = str(
            self.get_parameter("tensorrt_output_layout").value
        ).strip().lower()
        self._expected_max_targets = int(self.get_parameter("expected_max_targets").value)
        self._duplicate_iou = float(self.get_parameter("duplicate_iou").value)
        self._run_mode = str(self.get_parameter("run_mode").value).strip().lower()
        if self._run_mode not in {"continuous", "one_shot"}:
            raise ValueError("run_mode_must_be_continuous_or_one_shot")
        self._release_after_inference = bool(self.get_parameter("release_backend_after_inference").value)
        self._roi = tuple(int(self.get_parameter(k).value) for k in ("roi_x","roi_y","roi_width","roi_height"))
        self._one_shot_complete = False
        self._class_names_override = self._load_class_names_json(
            str(self.get_parameter("class_names_json").value)
        )
        self._allowed_class_ids, self._allowed_class_ids_error = parse_allowed_class_ids_json(
            str(self.get_parameter("allowed_class_ids_json").value)
        )

        self._boxes_pub = self.create_publisher(
            String, str(self.get_parameter("output_topic").value), qos_profile_sensor_data
        )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), qos_profile_sensor_data
        )
        self._debug_image_pub = self.create_publisher(
            Image, str(self.get_parameter("debug_image_topic").value), qos_profile_sensor_data
        )

        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_timer(float(self.get_parameter("publish_period_sec").value), self._on_timer)

        self._model = None
        self._class_names = dict(COCO80_CLASS_NAMES)
        self._class_names.update(self._class_names_override)
        self._backend_ready = False
        self._backend_message = "backend not initialized"
        self._trt_module = None
        self._torch_module = None
        self._trt_runtime = None
        self._trt_engine = None
        self._trt_context = None
        self._trt_input_index = -1
        self._trt_output_index = -1
        self._trt_device = None
        self._trt_contract: Optional[TensorRTEngineContract] = None
        self._model_sha256 = ""
        if self._allowed_class_ids_error is not None:
            self._backend_message = self._allowed_class_ids_error
            self._publish_status(
                "error",
                self._backend_message,
                allowed_class_ids_json=str(self.get_parameter("allowed_class_ids_json").value),
            )
        else:
            self._load_backend()

        self.get_logger().info(
            "yolo_detector_node started: "
            f"image_topic={str(self.get_parameter('image_topic').value)} "
            f"output_topic={str(self.get_parameter('output_topic').value)} "
            f"backend={self._backend_name} "
            f"model_path={self._model_path or '<empty>'} "
            f"device={self._device} "
            f"model_id={self._model_id or '<empty>'} "
            f"allowed_class_ids={sorted(self._allowed_class_ids)}"
        )

    def _load_class_names_json(self, text: str) -> dict[int, str]:
        try:
            payload = json.loads(text or "{}")
        except json.JSONDecodeError:
            return {}
        result: dict[int, str] = {}
        if isinstance(payload, dict):
            for key, value in payload.items():
                try:
                    result[int(key)] = str(value)
                except (TypeError, ValueError):
                    continue
        return result

    def _load_backend(self) -> None:
        if not self._model_path:
            self._backend_message = "model_path is empty; configure a .pt/.onnx/.engine model"
            self._publish_status("warn", self._backend_message)
            return
        if self._backend_name == "ultralytics":
            self._load_ultralytics_backend()
            return
        if self._backend_name == "opencv_dnn":
            self._load_opencv_dnn_backend()
            return
        if self._backend_name == "tensorrt":
            self._load_tensorrt_backend()
            return
        self._backend_message = f"unsupported backend: {self._backend_name}"
        self._publish_status("error", self._backend_message)

    def _load_ultralytics_backend(self) -> None:
        try:
            from ultralytics import YOLO
        except ImportError:
            self._backend_message = "ultralytics is not installed on this machine"
            self._publish_status("error", self._backend_message)
            return
        try:
            self._model_sha256 = verify_model_sha256(
                self._model_path, self._expected_model_sha256
            )
            self._model = YOLO(self._model_path)
            names = getattr(self._model, "names", None)
            if isinstance(names, dict):
                for key, value in names.items():
                    try:
                        self._class_names[int(key)] = str(value)
                    except (TypeError, ValueError):
                        continue
            self._backend_ready = True
            self._backend_message = "backend ready"
            self._publish_status(
                "ok", f"loaded {self._model_path}", model_sha256=self._model_sha256
            )
        except Exception as exc:
            self._backend_message = f"failed to load model: {exc}"
            self._publish_status("error", self._backend_message, model_sha256=self._model_sha256)

    def _load_opencv_dnn_backend(self) -> None:
        try:
            self._model_sha256 = verify_model_sha256(
                self._model_path, self._expected_model_sha256
            )
            net = cv2.dnn.readNetFromONNX(self._model_path)
            # Prefer CUDA when this OpenCV build supports it; otherwise fall back to CPU.
            if hasattr(cv2.dnn, "DNN_BACKEND_CUDA") and "cuda" in self._device.lower():
                try:
                    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
                except cv2.error:
                    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            else:
                net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            self._model = net
            self._backend_ready = True
            self._backend_message = "backend ready"
            self._publish_status(
                "ok", f"loaded {self._model_path}", model_sha256=self._model_sha256
            )
        except Exception as exc:
            self._backend_message = f"failed to load ONNX model: {exc}"
            self._publish_status("error", self._backend_message, model_sha256=self._model_sha256)

    def _load_tensorrt_backend(self) -> None:
        try:
            import tensorrt as trt
        except ImportError:
            self._backend_message = "tensorrt is not installed on this machine"
            self._publish_status("error", self._backend_message)
            return
        try:
            import torch
        except ImportError:
            self._backend_message = "torch is required for tensorrt backend buffers"
            self._publish_status("error", self._backend_message)
            return
        if not torch.cuda.is_available():
            self._backend_message = "torch.cuda is not available on this machine"
            self._publish_status("error", self._backend_message)
            return
        if not self._model_path.endswith(".engine"):
            self._backend_message = "tensorrt backend expects model_path to point to a .engine file"
            self._publish_status("error", self._backend_message)
            return
        if not self._model_id:
            self._backend_message = "tensorrt backend requires a non-empty model_id"
            self._publish_status("error", self._backend_message)
            return
        try:
            if not self._expected_model_sha256.strip():
                raise ValueError("expected_model_sha256_must_be_configured_for_tensorrt")
            expected_sha256 = self._expected_model_sha256
        except ValueError as exc:
            self._backend_message = str(exc)
            self._publish_status("error", self._backend_message)
            return
        try:
            # verify_model_sha256 raises model_sha256_mismatch and fails closed.
            actual_sha256 = verify_model_sha256(self._model_path, expected_sha256)
            logger = trt.Logger(trt.Logger.WARNING)
            runtime = trt.Runtime(logger)
            with open(self._model_path, "rb") as handle:
                engine = runtime.deserialize_cuda_engine(handle.read())
            if engine is None:
                raise RuntimeError("deserialize_cuda_engine returned None")
            context = engine.create_execution_context()
            if context is None:
                raise RuntimeError("create_execution_context returned None")
            if bool(getattr(engine, "has_implicit_batch_dimension", False)):
                raise RuntimeError("tensorrt_engine_must_use_explicit_batch")
            bindings: list[TensorRTBinding] = []
            for index in range(engine.num_bindings):
                bindings.append(
                    TensorRTBinding(
                        index=index,
                        name=str(engine.get_binding_name(index)),
                        is_input=bool(engine.binding_is_input(index)),
                        shape=tuple(int(value) for value in engine.get_binding_shape(index)),
                        dtype=normalize_tensorrt_dtype_name(engine.get_binding_dtype(index)),
                    )
                )
            validators = {
                "yolov5": validate_tensorrt_yolov5_contract,
                "yolov8": validate_tensorrt_yolov8_contract,
            }
            validator = validators.get(self._tensorrt_output_layout)
            if validator is None:
                raise ValueError("tensorrt_output_layout_must_be_yolov5_or_yolov8")
            contract = validator(
                bindings,
                input_width=self._input_width,
                input_height=self._input_height,
                class_count=self._expected_class_count,
            )
            self._trt_module = trt
            self._torch_module = torch
            self._trt_runtime = runtime
            self._trt_engine = engine
            self._trt_context = context
            self._trt_input_index = contract.input_index
            self._trt_output_index = contract.output_index
            self._trt_device = torch.device("cuda:0" if "cuda" in self._device.lower() else self._device)
            self._trt_contract = contract
            self._model_sha256 = actual_sha256
            self._model = engine
            self._backend_ready = True
            self._backend_message = "backend ready"
            self._publish_status(
                "ok",
                f"loaded {self._model_path}",
                model_id=self._model_id,
                model_sha256=self._model_sha256,
                input_shape=list(contract.input_shape),
                output_shape=list(contract.output_shape),
                tensorrt_output_layout=contract.decoder,
                expected_class_count=self._expected_class_count,
            )
        except Exception as exc:
            self._backend_message = f"failed to load TensorRT engine: {exc}"
            self._publish_status("error", self._backend_message)

    def _publish_status(self, level: str, message: str, **extra: Any) -> None:
        payload = {
            "level": level,
            "message": message,
            "backend": self._backend_name,
            "backend_ready": self._backend_ready,
        }
        payload.update(extra)
        encoded = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        if encoded == self._last_status_key:
            return
        self._last_status_key = encoded
        msg = String()
        msg.data = encoded
        self._status_pub.publish(msg)

    def _on_image(self, msg: Image) -> None:
        if self._one_shot_complete:
            return
        try:
            bgr = image_msg_to_bgr(msg)
        except Exception as exc:
            self._publish_status("error", f"image decode failed: {exc}")
            return
        self._frame = ImageFrame(
            stamp_ns=stamp_ns_from_msg(msg),
            frame_id=msg.header.frame_id or "left_camera_optical_frame",
            width=msg.width,
            height=msg.height,
            bgr=bgr,
        )

    def _decode_yolo_predictions(
        self,
        predictions: np.ndarray,
        *,
        frame: ImageFrame,
        scale: float,
        pad_x: float,
        pad_y: float,
    ) -> list[dict[str, Any]]:
        squeezed = np.squeeze(np.asarray(predictions))
        if squeezed.ndim == 1:
            squeezed = squeezed.reshape(1, -1)
        if squeezed.ndim != 2 or squeezed.shape[1] < 6:
            return []

        boxes_xywh: list[list[int]] = []
        scores: list[float] = []
        class_ids: list[int] = []
        for row in squeezed:
            objectness = float(row[4])
            if objectness <= 0.0:
                continue
            class_scores = row[5:]
            class_id = int(np.argmax(class_scores)) if class_scores.size else 0
            class_score = float(class_scores[class_id]) if class_scores.size else 1.0
            confidence = objectness * class_score
            if confidence < self._conf_threshold:
                continue

            cx, cy, w, h = [float(value) for value in row[:4]]
            x0 = (cx - 0.5 * w - pad_x) / max(scale, 1e-6)
            y0 = (cy - 0.5 * h - pad_y) / max(scale, 1e-6)
            x1 = (cx + 0.5 * w - pad_x) / max(scale, 1e-6)
            y1 = (cy + 0.5 * h - pad_y) / max(scale, 1e-6)
            x0 = float(np.clip(x0, 0.0, frame.width - 1.0))
            y0 = float(np.clip(y0, 0.0, frame.height - 1.0))
            x1 = float(np.clip(x1, x0 + 1.0, frame.width))
            y1 = float(np.clip(y1, y0 + 1.0, frame.height))

            boxes_xywh.append(
                [
                    int(round(x0)),
                    int(round(y0)),
                    max(1, int(round(x1 - x0))),
                    max(1, int(round(y1 - y0))),
                ]
            )
            scores.append(confidence)
            class_ids.append(class_id)

        if not boxes_xywh:
            return []

        selected = cv2.dnn.NMSBoxes(boxes_xywh, scores, self._conf_threshold, self._iou_threshold)
        if selected is None or len(selected) == 0:
            return []

        detections: list[dict[str, Any]] = []
        for index in np.asarray(selected).reshape(-1).tolist()[: self._max_detections]:
            x, y, w, h = boxes_xywh[int(index)]
            class_id = class_ids[int(index)]
            detections.append(
                {
                    "class_id": class_id,
                    "class_name": self._class_names.get(class_id, f"class_{class_id}"),
                    "confidence": compact_float(scores[int(index)]),
                    "bbox_xyxy": [
                        compact_float(x),
                        compact_float(y),
                        compact_float(x + w),
                        compact_float(y + h),
                    ],
                }
            )
        return detections

    def _decode_yolov8_predictions(
        self,
        predictions: np.ndarray,
        *,
        frame: ImageFrame,
        scale: float,
        pad_x: float,
        pad_y: float,
    ) -> list[dict[str, Any]]:
        rows = normalize_yolov8_predictions(
            predictions, class_count=self._expected_class_count
        )
        boxes_xywh: list[list[int]] = []
        scores: list[float] = []
        class_ids: list[int] = []
        for row in rows:
            class_scores = row[4:]
            class_id = int(np.argmax(class_scores))
            confidence = float(class_scores[class_id])
            if confidence < self._conf_threshold:
                continue
            cx, cy, w, h = [float(value) for value in row[:4]]
            x0 = float(np.clip((cx - 0.5 * w - pad_x) / max(scale, 1e-6), 0.0, frame.width - 1.0))
            y0 = float(np.clip((cy - 0.5 * h - pad_y) / max(scale, 1e-6), 0.0, frame.height - 1.0))
            x1 = float(np.clip((cx + 0.5 * w - pad_x) / max(scale, 1e-6), x0 + 1.0, frame.width))
            y1 = float(np.clip((cy + 0.5 * h - pad_y) / max(scale, 1e-6), y0 + 1.0, frame.height))
            boxes_xywh.append([int(round(x0)), int(round(y0)), max(1, int(round(x1 - x0))), max(1, int(round(y1 - y0)))])
            scores.append(confidence)
            class_ids.append(class_id)
        if not boxes_xywh:
            return []
        selected = cv2.dnn.NMSBoxes(boxes_xywh, scores, self._conf_threshold, self._iou_threshold)
        if selected is None or len(selected) == 0:
            return []
        detections: list[dict[str, Any]] = []
        for index in np.asarray(selected).reshape(-1).tolist()[: self._max_detections]:
            x, y, w, h = boxes_xywh[int(index)]
            class_id = class_ids[int(index)]
            detections.append({
                "class_id": class_id,
                "class_name": self._class_names.get(class_id, f"class_{class_id}"),
                "confidence": compact_float(scores[int(index)]),
                "bbox_xyxy": [compact_float(x), compact_float(y), compact_float(x + w), compact_float(y + h)],
            })
        return detections

    def _infer_ultralytics(self, frame: ImageFrame) -> tuple[list[dict[str, Any]], float]:
        started = time.perf_counter()
        results = self._model.predict(
            source=frame.bgr,
            conf=self._conf_threshold,
            iou=self._iou_threshold,
            imgsz=[self._input_height, self._input_width],
            device=self._device,
            max_det=self._max_detections,
            verbose=False,
        )
        inference_ms = (time.perf_counter() - started) * 1000.0

        detections: list[dict[str, Any]] = []
        if not results:
            return detections, inference_ms

        boxes = getattr(results[0], "boxes", None)
        if boxes is None:
            return detections, inference_ms

        xyxy = boxes.xyxy.detach().cpu().numpy() if boxes.xyxy is not None else np.empty((0, 4))
        conf = boxes.conf.detach().cpu().numpy() if boxes.conf is not None else np.empty((0,))
        cls = boxes.cls.detach().cpu().numpy() if boxes.cls is not None else np.empty((0,))
        for index in range(min(len(xyxy), self._max_detections)):
            class_id = int(cls[index]) if index < len(cls) else -1
            detections.append(
                {
                    "class_id": class_id,
                    "class_name": self._class_names.get(class_id, f"class_{class_id}"),
                    "confidence": compact_float(conf[index]) if index < len(conf) else 0.0,
                    "bbox_xyxy": [compact_float(value) for value in xyxy[index].tolist()],
                }
            )
        return detections, inference_ms

    def _infer_opencv_dnn(self, frame: ImageFrame) -> tuple[list[dict[str, Any]], float]:
        padded, scale, (pad_x, pad_y) = letterbox(frame.bgr, self._input_width, self._input_height)
        blob = cv2.dnn.blobFromImage(
            padded,
            scalefactor=1.0 / 255.0,
            size=(self._input_width, self._input_height),
            mean=(0.0, 0.0, 0.0),
            swapRB=True,
            crop=False,
        )
        started = time.perf_counter()
        self._model.setInput(blob)
        outputs = self._model.forward()
        inference_ms = (time.perf_counter() - started) * 1000.0
        return self._decode_yolo_predictions(
            np.asarray(outputs),
            frame=frame,
            scale=scale,
            pad_x=pad_x,
            pad_y=pad_y,
        ), inference_ms

    def _infer_tensorrt(self, frame: ImageFrame) -> tuple[list[dict[str, Any]], float]:
        if (
            self._trt_context is None
            or self._trt_engine is None
            or self._torch_module is None
            or self._trt_contract is None
        ):
            raise RuntimeError("tensorrt backend not initialized")

        padded, scale, (pad_x, pad_y) = letterbox(frame.bgr, self._input_width, self._input_height)
        rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
        input_array = np.transpose(rgb, (2, 0, 1)).astype(np.float32) / 255.0
        input_array = np.expand_dims(np.ascontiguousarray(input_array), axis=0)

        torch_module = self._torch_module
        device = self._trt_device
        input_tensor = torch_module.from_numpy(input_array).to(device=device)
        if tuple(input_tensor.shape) != self._trt_contract.input_shape:
            raise RuntimeError("runtime_input_shape_does_not_match_validated_tensorrt_contract")
        if not self._trt_context.set_binding_shape(self._trt_input_index, tuple(input_tensor.shape)):
            raise RuntimeError("TensorRT refused validated static input shape")
        output_shape = tuple(self._trt_context.get_binding_shape(self._trt_output_index))
        if output_shape != self._trt_contract.output_shape:
            raise RuntimeError("runtime_output_shape_does_not_match_validated_tensorrt_contract")
        output_dtype = trt_dtype_to_torch_dtype(
            self._trt_module,
            torch_module,
            self._trt_engine.get_binding_dtype(self._trt_output_index),
        )
        output_tensor = torch_module.empty(size=output_shape, dtype=output_dtype, device=device)
        bindings = [0] * self._trt_engine.num_bindings
        bindings[self._trt_input_index] = int(input_tensor.data_ptr())
        bindings[self._trt_output_index] = int(output_tensor.data_ptr())

        torch_module.cuda.synchronize()
        started = time.perf_counter()
        ok = self._trt_context.execute_v2(bindings)
        torch_module.cuda.synchronize()
        inference_ms = (time.perf_counter() - started) * 1000.0
        if not ok:
            raise RuntimeError("TensorRT execute_v2 returned false")
        outputs = output_tensor.detach().cpu().numpy()
        if self._trt_contract.decoder == "yolov5":
            detections = self._decode_yolo_predictions(
                outputs, frame=frame, scale=scale, pad_x=pad_x, pad_y=pad_y
            )
        elif self._trt_contract.decoder == "yolov8":
            detections = self._decode_yolov8_predictions(
                outputs, frame=frame, scale=scale, pad_x=pad_x, pad_y=pad_y
            )
        else:  # Contract construction should make this unreachable.
            raise RuntimeError("validated_tensorrt_decoder_is_unknown")
        return detections, inference_ms

    def _draw_detections(self, frame: ImageFrame, detections: list[dict[str, Any]]) -> Image:
        canvas = frame.bgr.copy()
        for detection in detections:
            bbox = detection.get("bbox_xyxy") or [0.0, 0.0, 0.0, 0.0]
            x0, y0, x1, y1 = [int(round(float(value))) for value in bbox]
            cv2.rectangle(canvas, (x0, y0), (x1, y1), (0, 255, 255), 2)
            label = (
                f"{detection.get('class_name', 'unknown')} "
                f"{float(detection.get('confidence', 0.0)):.2f}"
            )
            label_y = max(24, y0 - 10)
            cv2.putText(
                canvas,
                label,
                (x0, label_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
        return bgr_to_image_msg(canvas, frame_id=frame.frame_id, stamp_ns=frame.stamp_ns)

    def _infer(self, frame: ImageFrame) -> tuple[list[dict[str, Any]], float]:
        if self._backend_name == "ultralytics":
            return self._infer_ultralytics(frame)
        if self._backend_name == "opencv_dnn":
            return self._infer_opencv_dnn(frame)
        if self._backend_name == "tensorrt":
            return self._infer_tensorrt(frame)
        raise RuntimeError(f"unsupported backend: {self._backend_name}")

    def _on_timer(self) -> None:
        if self._one_shot_complete:
            return
        frame = self._frame
        if frame is None:
            self._publish_status("warn", "waiting for image frame")
            return
        if frame.stamp_ns == self._last_processed_stamp_ns:
            return
        self._last_processed_stamp_ns = frame.stamp_ns

        if not self._backend_ready or self._model is None:
            self._publish_status("warn", self._backend_message, frame_id=frame.frame_id)
            return

        try:
            rx, ry, rw, rh = validate_roi(*self._roi, frame.width, frame.height)
            infer_frame = frame if (rx,ry,rw,rh)==(0,0,frame.width,frame.height) else ImageFrame(
                stamp_ns=frame.stamp_ns, frame_id=frame.frame_id, width=rw, height=rh,
                bgr=frame.bgr[ry:ry+rh, rx:rx+rw])
            detections, inference_ms = self._infer(infer_frame)
            if (rx,ry)!=(0,0):
                for detection in detections:
                    b=detection.get("bbox_xyxy", [])
                    if len(b)==4: detection["bbox_xyxy"]=[b[0]+rx,b[1]+ry,b[2]+rx,b[3]+ry]
        except Exception as exc:
            self._publish_status("error", f"inference failed: {exc}", frame_id=frame.frame_id)
            return

        # All backends return their native detections first; this is the single,
        # backend-independent class gate before messages/debug images are published.
        detections = filter_detections_by_allowed_class_ids(detections, self._allowed_class_ids)
        observed_detection_count = len(detections)
        try:
            detections, ambiguous_reason = gate_target_detections(
                detections,
                expected_max_targets=self._expected_max_targets,
                duplicate_iou=self._duplicate_iou,
            )
        except ValueError as exc:
            self._publish_status("error", f"invalid target gate configuration: {exc}")
            return
        auto_grasp_permitted = bool(detections) and ambiguous_reason is None

        payload = {
            "transaction_id": f"pick-{frame.stamp_ns}",
            "stamp_sec": frame.stamp_ns // 1_000_000_000,
            "stamp_nanosec": frame.stamp_ns % 1_000_000_000,
            "frame_id": frame.frame_id,
            "backend": self._backend_name,
            "model_id": self._model_id,
            "model_path": self._model_path,
            "model_sha256": self._model_sha256,
            "image_width": frame.width,
            "image_height": frame.height,
            "inference_ms": compact_float(inference_ms),
            "detection_count": len(detections),
            "observed_detection_count": observed_detection_count,
            "allowed_class_ids": sorted(self._allowed_class_ids),
            "expected_class_count": self._expected_class_count,
            "expected_max_targets": self._expected_max_targets,
            "auto_grasp_permitted": auto_grasp_permitted,
            "ambiguous": ambiguous_reason is not None,
            "rejection_reason": ambiguous_reason or "",
            "detections": detections,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self._boxes_pub.publish(msg)
        self._publish_status(
            "warn" if ambiguous_reason is not None else "ok",
            (
                f"ambiguous targets rejected: {ambiguous_reason}"
                if ambiguous_reason is not None
                else f"detections={len(detections)} inference_ms={inference_ms:.2f}"
            ),
            frame_id=frame.frame_id,
            inference_ms=compact_float(inference_ms),
            detection_count=len(detections),
            observed_detection_count=observed_detection_count,
            allowed_class_ids=sorted(self._allowed_class_ids),
            expected_max_targets=self._expected_max_targets,
            auto_grasp_permitted=auto_grasp_permitted,
            ambiguous=ambiguous_reason is not None,
            rejection_reason=ambiguous_reason or "",
        )

        if self._publish_debug_image:
            self._debug_image_pub.publish(self._draw_detections(frame, detections))
        if self._run_mode == "one_shot":
            self._one_shot_complete = True
            self._frame = None
            if self._release_after_inference:
                self._release_backend()
            self._publish_status(
                "ok", "one_shot_complete", frame_id=frame.frame_id,
                stamp_ns=frame.stamp_ns, inference_count=1,
                backend_released=self._release_after_inference,
            )

    def _release_backend(self) -> None:
        self._model = None
        self._trt_context = None
        self._trt_engine = None
        self._trt_runtime = None
        self._trt_contract = None
        self._backend_ready = False
        torch_module = self._torch_module
        self._torch_module = None
        if torch_module is not None:
            try:
                torch_module.cuda.empty_cache()
            except Exception:
                pass


def main() -> None:
    rclpy.init()
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
