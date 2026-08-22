#!/usr/bin/env bash
set -euo pipefail

A_ROBOT_ROOT="${A_ROBOT_ROOT:-/path/to/a_robot}"
PARAMS_FILE="${A_ROBOT_ROOT}/rak-mercy/Deyes/config/stereo/sync_monitor.defaults.yaml"
BAG_PATH="${A_ROBOT_ROOT}/temp/deyes/rosbag/static_30s"
LEFT_IMAGE_TOPIC="${LEFT_IMAGE_TOPIC:-/x1/left_camera/image_raw}"
RIGHT_IMAGE_TOPIC="${RIGHT_IMAGE_TOPIC:-/x1/right_camera/image_raw}"
LEFT_INFO_TOPIC="${LEFT_INFO_TOPIC:-/x1/left_camera/camera_info}"
RIGHT_INFO_TOPIC="${RIGHT_INFO_TOPIC:-/x1/right_camera/camera_info}"

echo "Start the publisher + monitor:"
echo "ros2 launch deyes_bringup imx219_stereo.launch.py \\"
echo "  left_image_topic:=${LEFT_IMAGE_TOPIC} \\"
echo "  right_image_topic:=${RIGHT_IMAGE_TOPIC} \\"
echo "  left_info_topic:=${LEFT_INFO_TOPIC} \\"
echo "  right_info_topic:=${RIGHT_INFO_TOPIC} \\"
echo "  enable_monitor:=true"
echo
echo "Replay a bag with simulated time:"
echo "ros2 bag play ${BAG_PATH} --clock"
echo
echo "Inspect diagnostics:"
echo "ros2 topic hz ${LEFT_IMAGE_TOPIC}"
echo "ros2 topic hz ${RIGHT_IMAGE_TOPIC}"
echo "ros2 topic echo /deyes_sync_monitor/diagnostics"
echo "ros2 topic echo /deyes_sync_monitor/depth_gate_ok"
echo "ros2 topic echo /deyes_sync_monitor/failure_reason"
echo
echo "Suggested failure injection checks:"
echo "1. Replay a bag that is missing one camera_info topic."
echo "2. Remap one image topic to a stream with a different size."
echo "3. Replay a slower bag to trigger stale or low-rate warnings."
echo "4. Compare left/right header stamps and confirm they stay within the configured hard_sync_max_ms."
