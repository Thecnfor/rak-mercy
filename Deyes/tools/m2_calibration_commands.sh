#!/usr/bin/env bash
# Print safe, operator-completed M2 commands. This script never stores a password.
set -euo pipefail

ROBOT_IP="${ROBOT_IP:?Set ROBOT_IP=192.168.166.121 outside this repository}"
ROBOT_USER="${ROBOT_USER:-elephant}"
REMOTE_HOST="${REMOTE_HOST:-${ROBOT_USER}@${ROBOT_IP}}"
SESSION_ID="${SESSION_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
SESSION_DIR="${SESSION_DIR:-/home/${ROBOT_USER}/temp/deyes/calibration/${SESSION_ID}}"
DEYES_INSTALL_ROOT="${DEYES_INSTALL_ROOT:-/home/${ROBOT_USER}/temp/deyes/install}"

cat <<EOF
Target: ${REMOTE_HOST}
Session directory: ${SESSION_DIR}

1. On the robot, first launch only the formal C++ capture chain:
   source /opt/ros/galactic/setup.bash
   source ${DEYES_INSTALL_ROOT}/setup.bash
   ros2 launch deyes_bringup imx219_stereo.launch.py use_cpp_capture:=true \\
     width:=640 height:=360 fps:=30 enable_cuda_depth:=false

2. After a person measures the physical 9x6-board square with calipers, capture 40-60 pairs:
   ros2 run deyes_stereo physical_stereo_calibration capture \\
     --session-dir ${SESSION_DIR} --square-size-m <measured_metres> --samples 50

3. After the operator has checked left/right order, baseline sign and measured scale:
   ros2 run deyes_stereo physical_stereo_calibration compute \\
     --session-dir ${SESSION_DIR} --robot-id <confirmed_robot_id> \\
     --camera-pair-id <confirmed_pair_id> \\
     --confirm-left-right --confirm-baseline-sign --confirm-scale

The candidate YAML and JSON/Markdown report stay in the session directory. Do not
copy a candidate with validated=false into config/camera or use it for trusted depth.
EOF
