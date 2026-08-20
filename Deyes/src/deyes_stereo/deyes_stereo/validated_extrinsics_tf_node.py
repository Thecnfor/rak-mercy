"""Publish the camera attachment TF only when both physical gates pass."""

from __future__ import annotations

import json
from typing import Any

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .extrinsics_contract import load_yaml_document, validate_extrinsics


class ValidatedExtrinsicsTFNode(Node):
    def __init__(self) -> None:
        super().__init__("validated_extrinsics_tf_node")
        self.declare_parameter("extrinsics_path", "")
        self.declare_parameter("stereo_calibration_path", "")
        self.declare_parameter("status_topic", "/x1/stereo/extrinsics_status")
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), qos_profile_sensor_data
        )
        self._broadcaster = StaticTransformBroadcaster(self)
        self._publish_if_valid()

    def _status(self, level: str, **values: Any) -> None:
        message = String()
        message.data = json.dumps({"level": level, **values}, ensure_ascii=False, separators=(",", ":"))
        self._status_pub.publish(message)
        getattr(self.get_logger(), "info" if level == "ok" else "error")(message.data)

    def _publish_if_valid(self) -> None:
        try:
            extrinsics = load_yaml_document(str(self.get_parameter("extrinsics_path").value))
            stereo = load_yaml_document(str(self.get_parameter("stereo_calibration_path").value))
            result = validate_extrinsics(extrinsics, stereo_document=stereo)
        except (OSError, ValueError) as exc:
            self._status("invalid", trusted_for_grasp=False, reasons=[str(exc)])
            return
        if not result.valid or result.rotation is None or result.translation is None:
            self._status("invalid", trusted_for_grasp=False, calibration_id=result.calibration_id, reasons=list(result.reasons))
            return
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "left_camera_optical_frame"
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = [float(v) for v in result.translation]
        quaternion = extrinsics["quaternion_xyzw"]
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = [float(v) for v in quaternion]
        self._broadcaster.sendTransform(transform)
        self._status("ok", trusted_for_grasp=True, calibration_id=result.calibration_id, stereo_calibration_id=extrinsics["stereo_calibration_id"])


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = ValidatedExtrinsicsTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
