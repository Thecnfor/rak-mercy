#!/usr/bin/env bash
set -Eeuo pipefail

# Fail-closed one-line entry point for tomorrow's Jetson dry-run.  It builds a
# device-local TensorRT engine because engine files are not portable between
# TensorRT/CUDA/JetPack versions.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
DEYES_ROOT="${REPO_ROOT}/Deyes"
ONNX_PATH="${DEYES_ROOT}/models/pen/pen_student_01875_416_v1.onnx"
ONNX_SHA256="8916cbf25949d6e8b03c01e6ca1c7871aeac0ad105c931f1dd9881cb5d5a4c4e"
MODEL_ID="pen-yolov5-student-01875-416-v1"
WORKSPACE="${DEYES_WS:-/home/elephant/deyes_ws}"
ENGINE_DIR="${DEYES_MODEL_DIR:-/home/elephant/temp/deyes/models/pen}"
ENGINE_PATH="${ENGINE_DIR}/${MODEL_ID}.fp16.engine"
LOG_ROOT="${DEYES_LOG_ROOT:-/home/elephant/temp/deyes/single_shot_pick}"

die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
need_file() { [[ -f "$1" ]] || die "missing file: $1"; }

[[ -n "${ROBOT_IP:-}" ]] || die "set ROBOT_IP (for example 192.168.137.17)"
[[ -n "${STEREO_CALIB:-}" ]] || die "set STEREO_CALIB to the validated physical stereo YAML"
[[ -n "${HANDEYE_CALIB:-}" ]] || die "set HANDEYE_CALIB to the validated base_link camera YAML"
[[ -n "${RIGHT_ARM_SITE:-}" ]] || die "set RIGHT_ARM_SITE to the reviewed right-arm site YAML"
need_file "${ONNX_PATH}"
need_file "${STEREO_CALIB}"
need_file "${HANDEYE_CALIB}"
need_file "${RIGHT_ARM_SITE}"

actual_onnx_sha="$(sha256sum "${ONNX_PATH}" | awk '{print tolower($1)}')"
[[ "${actual_onnx_sha}" == "${ONNX_SHA256}" ]] || die "ONNX SHA256 mismatch"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  mapfile -t ros_setups < <(find /opt/ros -mindepth 2 -maxdepth 2 -name setup.bash -print 2>/dev/null | sort)
  [[ "${#ros_setups[@]}" -eq 1 ]] || die "source the intended ROS 2 setup.bash first"
  # shellcheck disable=SC1090
  source "${ros_setups[0]}"
else
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if command -v trtexec >/dev/null 2>&1; then
  TRTEXEC="$(command -v trtexec)"
elif [[ -x /usr/src/tensorrt/bin/trtexec ]]; then
  TRTEXEC=/usr/src/tensorrt/bin/trtexec
else
  die "trtexec not found; install/use the JetPack TensorRT tools"
fi

mkdir -p "${ENGINE_DIR}" "${LOG_ROOT}" "${WORKSPACE}/src"
if [[ ! -s "${ENGINE_PATH}" || ! -f "${ENGINE_PATH}.onnx.sha256" || "$(cat "${ENGINE_PATH}.onnx.sha256")" != "${ONNX_SHA256}" ]]; then
  printf 'Building Jetson-local TensorRT FP16 engine...\n'
  "${TRTEXEC}" --onnx="${ONNX_PATH}" --saveEngine="${ENGINE_PATH}" --fp16 --skipInference
  printf '%s' "${ONNX_SHA256}" > "${ENGINE_PATH}.onnx.sha256"
fi
engine_sha="$(sha256sum "${ENGINE_PATH}" | awk '{print tolower($1)}')"

# The repository may itself be the colcon workspace or may be copied beneath
# one.  Build from the repository's package source paths without deleting any
# existing workspace content.
cd "${REPO_ROOT}"
colcon build --symlink-install --base-paths "${DEYES_ROOT}/src" --packages-select \
  deyes_interfaces deyes_capture_cpp deyes_stereo deyes_bringup
# shellcheck disable=SC1091
source "${REPO_ROOT}/install/setup.bash"

printf 'Starting fail-closed dry-run; hardware execution is disabled.\n'
exec ros2 launch deyes_bringup real_robot_single_shot_dry_run.launch.py \
  calib_path:="${STEREO_CALIB}" \
  extrinsics_path:="${HANDEYE_CALIB}" \
  site_profile_path:="${RIGHT_ARM_SITE}" \
  model_path:="${ENGINE_PATH}" \
  model_id:="${MODEL_ID}" \
  model_sha256:="${engine_sha}" \
  log_root:="${LOG_ROOT}"
