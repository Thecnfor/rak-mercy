#!/usr/bin/env bash
set -euo pipefail

A_ROBOT_ROOT="${A_ROBOT_ROOT:-/path/to/a_robot}"
ROBOT_IP="${ROBOT_IP:?Set ROBOT_IP to the robot management address before running this helper}"
ROBOT_USER="${ROBOT_USER:-elephant}"
REMOTE_HOST="${REMOTE_HOST:-${ROBOT_USER}@${ROBOT_IP}}"
REMOTE_CALIB_TOOL="${REMOTE_CALIB_TOOL:-/home/elephant/mercury_grasp/calibrate_stereo.py}"
REPORT_DIR="${A_ROBOT_ROOT}/temp/deyes/reports"
CAMERA_CONFIG_DIR="${A_ROBOT_ROOT}/rak-mercy/Deyes/config/camera"
CALIB_RESOLUTION="${CALIB_RESOLUTION:-1280x720}"
CALIB_FPS="${CALIB_FPS:-30}"

echo "1. 先在 ${REMOTE_HOST} 上做 1280x720@30 前置检查："
echo "   ros2 launch deyes_bringup imx219_stereo.launch.py \\"
echo "     enable_monitor:=true use_cpp_capture:=true width:=1280 height:=720 fps:=30 \\"
echo "     target_publish_hz:=30.0 pair_max_skew_ms:=20.0 frame_stale_sec:=0.2 history_size:=8 \\"
echo "     monitor_expected_min_rate_hz:=20.0 monitor_hard_sync_max_ms:=3.0 \\"
echo "     monitor_soft_sync_max_ms:=10.0 monitor_allow_soft_sync:=false"
echo
echo "2. 检查远端棋盘格规格与本轮目标是否一致："
echo "   python3 - <<'PY'"
echo "   from pathlib import Path"
echo "   text = Path('${REMOTE_CALIB_TOOL}').read_text(errors='ignore').lower()"
echo "   print('findchessboardcorners' in text)"
echo "   PY"
echo
echo "3. 确认标定板是 print at 100% scale 的 checkerboard 9x6，并记录单个方格边长（毫米）。"
echo
echo "4. 执行棋盘格采集与求解："
echo "   python3 ${REMOTE_CALIB_TOOL} capture"
echo "   python3 ${REMOTE_CALIB_TOOL} compute"
echo
echo "5. 将生成的标定 YAML 复制入仓库："
echo "   cp <remote_generated_yaml> ${CAMERA_CONFIG_DIR}/<robot_id>_<camera_serial_or_sensorpair>_${CALIB_RESOLUTION}_<yyyymmdd>.yaml"
echo
echo "6. 在 ${REPORT_DIR} 生成配套报告，至少记录："
echo "   - reproj_error"
echo "   - K1/D1/K2/D2/R/T/P1/P2/Q"
echo "   - 有效样本数"
echo "   - 删除样本原因统计"
echo "   - 极线误差抽检"
echo
echo "7. 切换各 baseline 的 calib_path 到仓库内文件后重新验证："
echo "   ros2 launch deyes_bringup imx219_stereo.launch.py calib_path:=<repo_calib_yaml> width:=1280 height:=720 fps:=30"
echo "   ros2 launch deyes_bringup stereo_image_proc_baseline.launch.py calib_path:=<repo_calib_yaml>"
echo "   ros2 launch deyes_bringup sgbm_baseline.launch.py calib_path:=<repo_calib_yaml>"
