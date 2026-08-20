#!/usr/bin/env bash
set -euo pipefail

: "${CALIB_PATH:?Set CALIB_PATH to the explicit debug or physical stereo calibration YAML.}"
BAG_PATH="${BAG_PATH:-/path/to/a_robot/temp/deyes/rosbag/static_30s}"

echo "在线图像链路 + 同步诊断："
echo "ros2 launch deyes_bringup imx219_stereo.launch.py calib_path:=${CALIB_PATH} enable_monitor:=true"
echo
echo "官方几何基线（要求已安装 stereo_image_proc）："
echo "ros2 launch deyes_bringup stereo_image_proc_baseline.launch.py calib_path:=${CALIB_PATH}"
echo
echo "SGBM 对照基线："
echo "ros2 launch deyes_bringup sgbm_baseline.launch.py calib_path:=${CALIB_PATH}"
echo
echo "离线回放建议："
echo "ros2 bag play ${BAG_PATH} --clock"
echo
echo "对比检查："
echo "1. ros2 topic list -t | grep -E 'image_rect|disparity|points2|/x1/stereo/debug'"
echo "2. 记录输出是否存在、frame_id 是否正确、时间戳是否继承、重复回放是否一致。"
