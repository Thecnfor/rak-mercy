#!/usr/bin/env bash
# Competition chain: navigation -> CUDA stereo + YOLO admission -> fixed venue pick -> place.
# The unvalidated quick calibration may admit a fixed pose, but never pretends to be hand-eye TF.
set -uo pipefail

RAC_SCRIPTS="${RAC_SCRIPTS:-$HOME/scripts}"
LOG_DIR="${LOG_DIR:-$HOME/temp/deyes/competition/race_try_$(date +%Y%m%d_%H%M%S)}"
DRY_RUN="${DEGRADED_DRY_RUN:-0}"
ALLOW_DEGRADED="${ALLOW_DEGRADED:-0}"
FORCE_DEGRADED="${FORCE_DEGRADED:-0}"
DEYES_WS="${DEYES_WS:-$HOME/deyes_competition_ws}"
DEYES_INSTALL="${DEYES_INSTALL:-$DEYES_WS/install}"
DEYES_ASSETS="${DEYES_ASSETS:-$HOME/deyes_competition_assets}"
OPENCV_PREFIX="${DEYES_OPENCV_PREFIX:-$HOME/opencv-4.8.0-cuda}"
CALIB_PATH="${DEYES_CALIB_PATH:-$DEYES_ASSETS/venue_20260827_quick_stereo.yaml}"
DETECTOR_CONFIG="${DEYES_DETECTOR_CONFIG:-$DEYES_ASSETS/competition_fixed_scene.yaml}"
MODEL_PATH="${DEYES_MODEL_PATH:-$HOME/x1_real_robot/assets/models/pencil.engine}"
PERCEPTION_TIMEOUT="${PERCEPTION_TIMEOUT:-25}"
mkdir -p "$LOG_DIR"

VISION_PID=""
cleanup() {
  if [ -n "$VISION_PID" ]; then kill "$VISION_PID" 2>/dev/null || true; fi
  if [ -n "${NAV_PID:-}" ]; then kill "$NAV_PID" 2>/dev/null || true; fi
}
trap cleanup EXIT INT TERM

source_ros1() {
  unset ROS_DISTRO ROS_VERSION
  source /opt/ros/noetic/setup.bash
  source /home/elephant/mercury_x1_ros/devel/setup.bash
}

run_step() {
  name="$1"; shift
  echo "[race_try] $name: $*"
  "$@" >"$LOG_DIR/$name.log" 2>&1
  rc=$?
  if [ "$rc" -ne 0 ]; then
    echo "[race_try] STOP $name rc=$rc log=$LOG_DIR/$name.log"
    tail -n 30 "$LOG_DIR/$name.log" 2>/dev/null || true
    exit "$rc"
  fi
}

if [ "$DRY_RUN" = "1" ]; then
  run_step head_dry python3 "$RAC_SCRIPTS/set_venue_head.py" --dry-run
  run_step pick_dry python3 "$RAC_SCRIPTS/pick_pen_degraded.py" --dry-run
  run_step place_dry python3 "$RAC_SCRIPTS/place_pen_degraded.py" --dry-run
  echo "[race_try] DRY-RUN OK: goal3_right -> pick -> goal4_back -> place"
  exit 0
fi

run_perception_gate() {
  [ -f "$CALIB_PATH" ] || { echo "missing calibration: $CALIB_PATH"; return 21; }
  [ -f "$DETECTOR_CONFIG" ] || { echo "missing detector config: $DETECTOR_CONFIG"; return 22; }
  [ -f "$MODEL_PATH" ] || { echo "missing TensorRT engine: $MODEL_PATH"; return 23; }
  [ -f "$DEYES_INSTALL/setup.bash" ] || { echo "missing ROS2 install: $DEYES_INSTALL"; return 24; }
  "$DEYES_ASSETS/probe_opencv_cuda.sh" "$OPENCV_PREFIX" || return 25
  model_sha="$(sha256sum "$MODEL_PATH" | awk '{print $1}')" || return 26

  unset ROS_DISTRO ROS_VERSION ROS_PACKAGE_PATH
  source /opt/ros/galactic/setup.bash
  source "$DEYES_INSTALL/setup.bash"
  export OpenCV_DIR="$OPENCV_PREFIX/lib/cmake/opencv4"
  export PKG_CONFIG_PATH="$OPENCV_PREFIX/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
  export LD_LIBRARY_PATH="/opt/ros/galactic/lib:/opt/ros/galactic/lib/aarch64-linux-gnu:$OPENCV_PREFIX/lib:${LD_LIBRARY_PATH:-}"

  ros2 launch deyes_bringup imx219_stereo.launch.py \
    use_cpp_capture:=true width:=640 height:=360 fps:=30 \
    swap_left_right:=true \
    calib_path:="$CALIB_PATH" enable_cuda_depth:=true \
    cuda_depth_publish_debug_rect:=true cuda_depth_max_sync_diff_ms:=10.0 \
    enable_detector:=true detector_config:="$DETECTOR_CONFIG" \
    detector_backend:=tensorrt detector_model_path:="$MODEL_PATH" \
    detector_model_id:=pen-student-416-venue detector_expected_model_sha256:="$model_sha" \
    detector_input_width:=416 detector_input_height:=416 \
    detector_expected_class_count:=1 detector_expected_max_targets:=1 \
    >"$LOG_DIR/vision.log" 2>&1 &
  VISION_PID=$!
  sleep 4
  kill -0 "$VISION_PID" 2>/dev/null || { tail -n 50 "$LOG_DIR/vision.log"; return 27; }
  ros2 run deyes_stereo competition_perception_gate \
    --timeout "$PERCEPTION_TIMEOUT" --output "$LOG_DIR/perception_gate.json" \
    >"$LOG_DIR/perception_gate.log" 2>&1
  gate_rc=$?
  kill "$VISION_PID" 2>/dev/null || true
  wait "$VISION_PID" 2>/dev/null || true
  VISION_PID=""
  return "$gate_rc"
}

source_ros1
if ! rostopic list 2>/dev/null | grep -qx /move_base/goal; then
  echo "[race_try] starting ROS1 navigation"
  roslaunch turn_on_mercury_robot navigation.launch >"$LOG_DIR/navigation.log" 2>&1 &
  NAV_PID=$!
  ready=0
  for _ in $(seq 1 30); do
    if rostopic list 2>/dev/null | grep -qx /move_base/goal; then ready=1; break; fi
    sleep 2
  done
  if [ "$ready" -ne 1 ]; then
    echo "[race_try] STOP move_base unavailable"
    exit 2
  fi
fi

run_step nav_pick python3 "$RAC_SCRIPTS/send_one_goal.py" goal3_right
run_step head_profile python3 "$RAC_SCRIPTS/set_venue_head.py"

admission="forced_degraded"
if [ "$FORCE_DEGRADED" != "1" ]; then
  echo "[race_try] starting CUDA depth + YOLO admission"
  if run_perception_gate; then
    admission="depth_yolo_accepted"
  else
    rc=$?
    echo "[race_try] perception rejected rc=$rc"
    [ -f "$LOG_DIR/perception_gate.json" ] && cat "$LOG_DIR/perception_gate.json"
    if [ "$ALLOW_DEGRADED" != "1" ]; then
      echo "[race_try] STOP: fail-closed. Set ALLOW_DEGRADED=1 only for explicit venue fallback."
      exit "$rc"
    fi
    admission="explicit_fallback_after_perception_failure_rc_$rc"
  fi
fi
echo "$admission" >"$LOG_DIR/admission_mode.txt"
run_step pick python3 "$RAC_SCRIPTS/pick_pen_degraded.py"
source_ros1
run_step nav_place python3 "$RAC_SCRIPTS/send_one_goal.py" goal4_back
run_step place python3 "$RAC_SCRIPTS/place_pen_degraded.py"

echo "[race_try] COMPLETE logs=$LOG_DIR"
