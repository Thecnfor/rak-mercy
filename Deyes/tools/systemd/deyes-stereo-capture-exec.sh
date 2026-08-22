#!/usr/bin/env bash
# Runs as the non-root robot account.  Recovery privilege is deliberately kept
# out of this process and is limited to the systemd ExecStopPost helper.
set -euo pipefail

: "${DEYES_WORKSPACE:?DEYES_WORKSPACE must be set in /etc/default/deyes-stereo-capture}"
: "${DEYES_CAPTURE_PARAMS:?DEYES_CAPTURE_PARAMS must be set in /etc/default/deyes-stereo-capture}"
: "${DEYES_CALIB_PATH:?DEYES_CALIB_PATH must be set to a physical calibration YAML}"

test -r "${DEYES_CAPTURE_PARAMS}"
test -r "${DEYES_CALIB_PATH}"
source /opt/ros/galactic/setup.bash
if [[ -n "${MERCURY_ROS_WORKSPACE:-}" ]]; then
  source "${MERCURY_ROS_WORKSPACE}/install/setup.bash"
fi
source "${DEYES_WORKSPACE}/install/setup.bash"
if [[ -n "${DEYES_OPENCV_LIB:-}" ]]; then
  export LD_LIBRARY_PATH="${DEYES_OPENCV_LIB}:${LD_LIBRARY_PATH:-}"
fi

# exec preserves the capture process exit code (75 means Argus/appsink stall).
exec ros2 run deyes_capture_cpp imx219_stereo_capture_node --ros-args \
  --params-file "${DEYES_CAPTURE_PARAMS}" \
  -p calib_path:="${DEYES_CALIB_PATH}"
