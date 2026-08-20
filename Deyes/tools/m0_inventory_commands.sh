#!/usr/bin/env bash
set -euo pipefail

A_ROBOT_ROOT="${A_ROBOT_ROOT:-/path/to/a_robot}"
TEMP_ROOT="${A_ROBOT_ROOT}/temp/deyes"
REPORT_DIR="${TEMP_ROOT}/reports"
BAG_DIR="${TEMP_ROOT}/rosbag"

mkdir -p "${REPORT_DIR}" "${BAG_DIR}"

{
  echo "# Deyes Hardware Inventory"
  echo
  echo "Timestamp: $(date -Iseconds)"
  echo "Host: $(hostname)"
  echo
  echo "## ROS"
  echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
  echo
  uname -a
  lsb_release -a || true
  echo
  echo "## USB"
  lsusb
  echo
  echo "## V4L2"
  v4l2-ctl --list-devices || true
  echo
  echo "## Camera Launch Args"
  ros2 launch turn_on_mercury_robot mercury_camera.launch.py --show-args || true
  echo
  echo "## Topics"
  ros2 topic list -t | grep -E 'image|camera_info|depth|points' || true
} | tee "${REPORT_DIR}/hardware_inventory.md"

echo
echo "Record a 30-second static bag after replacing topic names as needed:"
echo "ros2 bag record -o ${BAG_DIR}/static_30s \\"
echo "  /your/left/image_raw /your/right/image_raw \\"
echo "  /your/left/camera_info /your/right/camera_info"
