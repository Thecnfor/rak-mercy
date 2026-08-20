"""ROS node joining YOLO boxes to rectified-left pixels by exact stamp."""
from __future__ import annotations
import json, time
import cv2, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from .pen_feature_extractor import ExtractorParams, PenFeatureJoiner, RectifiedImage, build_feature_payload, extract_one_pen

def _stamp(msg: Image) -> int: return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
def _gray(msg: Image) -> np.ndarray:
    raw=np.frombuffer(msg.data,dtype=np.uint8)
    if msg.encoding == "mono8": return raw.reshape(msg.height,msg.step)[:,:msg.width].copy()
    if msg.encoding in {"bgr8","rgb8"}:
        a=raw.reshape(msg.height,msg.step//3,3)[:,:msg.width]; return cv2.cvtColor(a,cv2.COLOR_BGR2GRAY if msg.encoding=="bgr8" else cv2.COLOR_RGB2GRAY)
    raise ValueError(f"unsupported_rectified_image_encoding:{msg.encoding}")

class PenFeatureNode(Node):
    def __init__(self) -> None:
        super().__init__("pen_feature_node")
        defaults={"detection_topic":"/x1/detection/boxes","image_topic":"/x1/stereo/debug/left_rect","output_topic":"/x1/detection/pen_features","status_topic":"/x1/detection/pen_features_status","sync_cache_capacity":8,"sync_cache_max_age_sec":.5,"min_mask_pixels":12,"max_mask_pixels":800,"min_axis_length_px":18.,"min_aspect_ratio":2.,"min_pca_ratio":2.,"edge_margin_px":12,"min_contrast":8,"morphology_kernel_px":5}
        for k,v in defaults.items(): self.declare_parameter(k,v)
        self._joiner=PenFeatureJoiner(int(self.get_parameter("sync_cache_capacity").value),int(float(self.get_parameter("sync_cache_max_age_sec").value)*1e9))
        self._params=ExtractorParams(**{k:self.get_parameter(k).value for k in ExtractorParams.__dataclass_fields__})
        self._out=self.create_publisher(String,str(self.get_parameter("output_topic").value),qos_profile_sensor_data); self._status_pub=self.create_publisher(String,str(self.get_parameter("status_topic").value),qos_profile_sensor_data)
        self.create_subscription(String,str(self.get_parameter("detection_topic").value),self._on_detection,qos_profile_sensor_data); self.create_subscription(Image,str(self.get_parameter("image_topic").value),self._on_image,qos_profile_sensor_data)
        self.create_timer(0.10, self._expire)
    def _status(self, reason: str, level: str="warn", **extra: object) -> None:
        m=String(); m.data=json.dumps({"level":level,"reason":reason,**extra},separators=(",",":")); self._status_pub.publish(m)
    def _process(self,payload:dict[str,object],image:RectifiedImage)->None:
        feature,reason=extract_one_pen(image,payload,self._params)
        result=build_feature_payload(image,feature,reason)
        m=String();m.data=json.dumps(result,separators=(",",":"));self._out.publish(m)
        if feature is None:
            self._status(reason,frame_result="empty")
            return
        self._status(reason,"ok" if feature["axis_complete"] else "warn",target_id=feature["target_id"])
    def _on_detection(self,msg:String)->None:
        try: payload=json.loads(msg.data); pair=self._joiner.add_detection(payload,time.monotonic_ns())
        except Exception as exc: self._status(f"invalid_detection_json:{exc}");return
        if pair:self._process(*pair)
    def _on_image(self,msg:Image)->None:
        try: image=RectifiedImage(_stamp(msg),str(msg.header.frame_id),int(msg.width),int(msg.height),_gray(msg));pair=self._joiner.add_image(image,time.monotonic_ns())
        except Exception as exc:self._status(f"invalid_rectified_image:{exc}");return
        if pair:self._process(*pair)
    def _expire(self)->None:
        self._joiner.expire(time.monotonic_ns())
def main(args:object=None)->None:
    rclpy.init(args=args); n=PenFeatureNode()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
