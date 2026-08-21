#!/usr/bin/env bash
# Safe X1 deployment helper. It never accepts, stores, or prints passwords.
set -euo pipefail

MODE="${1:---dry-run}"
case "${MODE}" in
  --dry-run|--prepare|--deploy) ;;
  *) echo "usage: $0 [--dry-run|--prepare|--deploy]" >&2; exit 64 ;;
esac

: "${ROBOT_IP:?Set ROBOT_IP outside this repository for the current robot.}"
: "${ROBOT_USER:?Set ROBOT_USER outside this repository.}"
: "${REMOTE_WORKSPACE:?Set REMOTE_WORKSPACE outside this repository (for example /home/robot/deyes_ws).}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LOCAL_DEYES="${LOCAL_DEYES:-$(cd -- "${SCRIPT_DIR}/.." && pwd)}"
REMOTE_HOST="${ROBOT_USER}@${ROBOT_IP}"
REMOTE_TEMP="${REMOTE_TEMP:-/home/${ROBOT_USER}/temp/deyes}"
REMOTE_PROJECT_ROOT="${REMOTE_PROJECT_ROOT:-${REMOTE_WORKSPACE}/src/Deyes}"
REMOTE_PACKAGE_ROOT="${REMOTE_PACKAGE_ROOT:-${REMOTE_PROJECT_ROOT}/src}"
REMOTE_CONFIG_ROOT="${REMOTE_CONFIG_ROOT:-${REMOTE_PROJECT_ROOT}/config}"
REMOTE_TOOLS_ROOT="${REMOTE_TOOLS_ROOT:-${REMOTE_PROJECT_ROOT}/tools}"
REMOTE_OPENCV_PREFIX="${REMOTE_OPENCV_PREFIX:-}"
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

duplicate_layout_check() {
  "${SSH[@]}" "set -eu
    for package in deyes_capture_cpp deyes_stereo deyes_bringup; do
      if [ -f \"${REMOTE_WORKSPACE}/src/\${package}/package.xml\" ]; then
        echo \"DUPLICATE_ROS_PACKAGE:${REMOTE_WORKSPACE}/src/\${package}\"
      fi
    done"
}

echo "Target: ${REMOTE_HOST}"
echo "Mode: ${MODE}"
echo "Remote project root: ${REMOTE_PROJECT_ROOT}"
echo "This helper uses SSH BatchMode only; configure an SSH key/agent before deployment."
echo "Running read-only connection and camera-occupancy check..."
OCCUPANCY="$(readonly_check)" || { echo "Remote authentication or read-only preflight failed; no remote changes made." >&2; exit 3; }
printf '%s\n' "${OCCUPANCY}"
if printf '%s\n' "${OCCUPANCY}" | grep -Eq 'imx219_stereo_capture_node|cuda_stereo_depth_node|stereo_pointcloud_node|nvarguscamerasrc|CAMERA_DEVICE_OCCUPIED:'; then
  echo "Camera/vision process is occupied. Stop it manually, then rerun; this helper will not kill processes." >&2
  exit 4
fi
DUPLICATES="$(duplicate_layout_check)" || { echo "Remote layout check failed; no remote changes made." >&2; exit 7; }
if [[ -n "${DUPLICATES}" ]]; then
  printf '%s\n' "${DUPLICATES}" >&2
  echo "Duplicate ROS package layout detected. Archive it explicitly before deployment." >&2
  exit 8
fi

echo "Packages to copy (no --delete): deyes_capture_cpp, deyes_stereo, deyes_bringup; canonical config only."
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

"${SSH[@]}" "mkdir -p '${REMOTE_PACKAGE_ROOT}' '${REMOTE_CONFIG_ROOT}' '${REMOTE_TOOLS_ROOT}/systemd' '${REMOTE_TEMP}/build' '${REMOTE_TEMP}/logs' '${REMOTE_TEMP}/install'"
rsync -a "${LOCAL_DEYES}/src/deyes_capture_cpp/" "${REMOTE_HOST}:${REMOTE_PACKAGE_ROOT}/deyes_capture_cpp/"
rsync -a "${LOCAL_DEYES}/src/deyes_stereo/" "${REMOTE_HOST}:${REMOTE_PACKAGE_ROOT}/deyes_stereo/"
rsync -a "${LOCAL_DEYES}/src/deyes_bringup/" "${REMOTE_HOST}:${REMOTE_PACKAGE_ROOT}/deyes_bringup/"
rsync -a "${LOCAL_DEYES}/config/" "${REMOTE_HOST}:${REMOTE_CONFIG_ROOT}/"
rsync -a "${LOCAL_DEYES}/tools/systemd/" "${REMOTE_HOST}:${REMOTE_TOOLS_ROOT}/systemd/"
rsync -a "${LOCAL_DEYES}/tools/install_argus_capture_supervisor.sh" "${LOCAL_DEYES}/tools/argus_capture_supervisor.md" "${REMOTE_HOST}:${REMOTE_TOOLS_ROOT}/"

"${SSH[@]}" "set -eu
  source /opt/ros/galactic/setup.bash
  cd '${REMOTE_WORKSPACE}'
  cmake_args=()
  if [ -n '${REMOTE_OPENCV_PREFIX}' ]; then
    test -f '${REMOTE_OPENCV_PREFIX}/lib/cmake/opencv4/OpenCVConfig.cmake'
    cmake_args=(
      --cmake-args
      '-DOpenCV_DIR=${REMOTE_OPENCV_PREFIX}/lib/cmake/opencv4'
      '-DCMAKE_BUILD_RPATH=${REMOTE_OPENCV_PREFIX}/lib'
      '-DCMAKE_INSTALL_RPATH=${REMOTE_OPENCV_PREFIX}/lib'
    )
  fi
  colcon --log-base '${REMOTE_TEMP}/logs' build \\
    --base-paths '${REMOTE_PACKAGE_ROOT}/deyes_capture_cpp' '${REMOTE_PACKAGE_ROOT}/deyes_stereo' '${REMOTE_PACKAGE_ROOT}/deyes_bringup' \\
    --build-base '${REMOTE_TEMP}/build' --install-base '${REMOTE_TEMP}/install' \\
    --packages-select deyes_capture_cpp deyes_stereo deyes_bringup \"\${cmake_args[@]}\"
  echo 'Deployment build completed. No launch was started.'"
echo "Deploy/build complete. Physical calibration remains required; start only debug/RViz until validated=true."
