#!/usr/bin/env bash
# Safe X1 deployment helper. It never accepts, stores, or prints passwords.
set -euo pipefail

MODE="${1:---dry-run}"
case "${MODE}" in
  --dry-run|--prepare|--deploy) ;;
  *) echo "usage: $0 [--dry-run|--prepare|--deploy]" >&2; exit 64 ;;
esac

: "${ROBOT_IP:?Set ROBOT_IP outside this repository (for example 192.168.166.121).}"
: "${ROBOT_USER:?Set ROBOT_USER outside this repository.}"
: "${REMOTE_WORKSPACE:?Set REMOTE_WORKSPACE outside this repository (for example /home/robot/deyes_ws).}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LOCAL_DEYES="${LOCAL_DEYES:-$(cd -- "${SCRIPT_DIR}/.." && pwd)}"
REMOTE_HOST="${ROBOT_USER}@${ROBOT_IP}"
REMOTE_TEMP="${REMOTE_TEMP:-/home/${ROBOT_USER}/temp/deyes}"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new "${REMOTE_HOST}")

readonly_check() {
  "${SSH[@]}" 'set -eu
    echo "== host =="; hostname
    echo "== ROS processes / camera consumers =="
    (pgrep -af "imx219_stereo_capture_node|cuda_stereo_depth_node|stereo_pointcloud_node|nvarguscamerasrc" || true)
    echo "== video device consumers =="
    for device in /dev/video*; do
      [ -e "$device" ] || continue
      pids="$(fuser "$device" 2>/dev/null || true)"
      if [ -n "$pids" ]; then
        echo "CAMERA_DEVICE_OCCUPIED:${device}:${pids}"
      fi
    done'
}

echo "Target: ${REMOTE_HOST}"
echo "Mode: ${MODE}"
echo "This helper uses SSH BatchMode only; configure an SSH key/agent before deployment."
echo "Running read-only connection and camera-occupancy check..."
OCCUPANCY="$(readonly_check)" || { echo "Remote authentication or read-only preflight failed; no remote changes made." >&2; exit 3; }
printf '%s\n' "${OCCUPANCY}"
if printf '%s\n' "${OCCUPANCY}" | grep -Eq 'imx219_stereo_capture_node|cuda_stereo_depth_node|stereo_pointcloud_node|nvarguscamerasrc|CAMERA_DEVICE_OCCUPIED:'; then
  echo "Camera/vision process is occupied. Stop it manually, then rerun; this helper will not kill processes." >&2
  exit 4
fi

echo "Packages to copy (no --delete): deyes_capture_cpp, deyes_stereo, deyes_bringup; config only."
if [[ "${MODE}" != "--deploy" ]]; then
  echo "Preflight complete. --prepare intentionally performs no remote write."
  exit 0
fi
: "${I_UNDERSTAND_REMOTE_WRITE:=}"
if [[ "${I_UNDERSTAND_REMOTE_WRITE}" != "yes" ]]; then
  echo "Refusing remote write. Re-run with I_UNDERSTAND_REMOTE_WRITE=yes after reviewing the preflight." >&2
  exit 5
fi
command -v rsync >/dev/null || { echo "rsync is required on the deployment host." >&2; exit 6; }

"${SSH[@]}" "mkdir -p '${REMOTE_WORKSPACE}/src' '${REMOTE_WORKSPACE}/config' '${REMOTE_TEMP}/build' '${REMOTE_TEMP}/logs'"
rsync -a "${LOCAL_DEYES}/src/deyes_capture_cpp/" "${REMOTE_HOST}:${REMOTE_WORKSPACE}/src/deyes_capture_cpp/"
rsync -a "${LOCAL_DEYES}/src/deyes_stereo/" "${REMOTE_HOST}:${REMOTE_WORKSPACE}/src/deyes_stereo/"
rsync -a "${LOCAL_DEYES}/src/deyes_bringup/" "${REMOTE_HOST}:${REMOTE_WORKSPACE}/src/deyes_bringup/"
rsync -a "${LOCAL_DEYES}/config/" "${REMOTE_HOST}:${REMOTE_WORKSPACE}/config/"
"${SSH[@]}" "set -eu
  source /opt/ros/galactic/setup.bash
  cd '${REMOTE_WORKSPACE}'
  colcon --log-base '${REMOTE_TEMP}/logs' build --build-base '${REMOTE_TEMP}/build' \\
    --install-base '${REMOTE_TEMP}/install' --packages-select deyes_capture_cpp deyes_stereo deyes_bringup
  echo 'Deployment build completed. No launch was started.'"
echo "Deploy/build complete. Physical calibration remains required; start only debug/RViz until validated=true."
