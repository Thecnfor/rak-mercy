#!/usr/bin/env bash
# Fail-closed competition transaction: goal3 -> perceive once -> pick -> verify -> goal4 -> place.
set -Eeuo pipefail

RAC_SCRIPTS="${RAC_SCRIPTS:-$HOME/scripts}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
DEYES_WS="${DEYES_WS:-$HOME/deyes_competition_ws}"
DEYES_INSTALL="${DEYES_INSTALL:-$DEYES_WS/install}"
DEYES_ASSETS="${DEYES_ASSETS:-$HOME/deyes_competition_assets}"
OPENCV_PREFIX="${DEYES_OPENCV_PREFIX:-$HOME/opencv-4.8.0-cuda}"
CALIB_PATH="${DEYES_CALIB_PATH:-$DEYES_ASSETS/venue_20260827_quick_stereo.yaml}"
DETECTOR_CONFIG="${DEYES_DETECTOR_CONFIG:-$DEYES_ASSETS/competition_fixed_scene.yaml}"
SITE_METADATA_PATH="${COMPETITION_SITE_METADATA:-$DEYES_ASSETS/competition_venue_65cm.yaml}"
ENGINE_PATH="${DEYES_ENGINE_PATH:-$HOME/temp/deyes/models/pen/pen-yolov5-student-01875-416-v1.fp16.engine}"
ENGINE_MANIFEST="${DEYES_ENGINE_MANIFEST:-${ENGINE_PATH}.manifest.json}"
EXPECTED_ONNX_SHA256="8916cbf25949d6e8b03c01e6ca1c7871aeac0ad105c931f1dd9881cb5d5a4c4e"

FIXED_TABLE_HEIGHT_MM="${FIXED_TABLE_HEIGHT_MM:-650}"
ALLOW_BBOX_CENTER="${ALLOW_BBOX_CENTER:-1}"
ALLOW_FIXED_XY_FALLBACK="${ALLOW_FIXED_XY_FALLBACK:-0}"
FORCE_FIXED_TARGET="${FORCE_FIXED_TARGET:-0}"
GROUND_PLANE_MAX_DELTA_MM="${GROUND_PLANE_MAX_DELTA_MM:-25}"
TARGET_TIMEOUT_SEC="${TARGET_TIMEOUT_SEC:-30}"

TARGET_PACKAGE="${COMPETITION_TARGET_PACKAGE:-deyes_stereo}"
TARGET_EXECUTABLE="${COMPETITION_TARGET_EXECUTABLE:-competition_pick_target}"
TARGET_TOPIC="${COMPETITION_TARGET_TOPIC:-/x1/competition/pick_target}"
TARGET_ADAPTER="${COMPETITION_TARGET_ADAPTER:-$RAC_SCRIPTS/competition_target_snapshot_adapter.py}"
PICK_SCRIPT="${COMPETITION_PICK_SCRIPT:-$RAC_SCRIPTS/pick_pen_degraded.py}"
PLACE_SCRIPT="${COMPETITION_PLACE_SCRIPT:-$RAC_SCRIPTS/place_pen_degraded.py}"
NAV_GOAL3="${COMPETITION_NAV_GOAL3:-goal3_right}"
NAV_GOAL4="${COMPETITION_NAV_GOAL4:-goal4_back}"

TRANSACTION_ID="${COMPETITION_TRANSACTION_ID:-$(date -u +%Y%m%dT%H%M%SZ)_$$}"
LOG_DIR="${LOG_DIR:-$HOME/temp/deyes/competition/$TRANSACTION_ID}"
TARGET_JSON="$LOG_DIR/target.json"
ADMISSION_JSON="$LOG_DIR/admission.json"
GRASP_JSON="$LOG_DIR/grasp_verification.json"
TRACE_JSONL="$LOG_DIR/trace.jsonl"
mkdir -p "$LOG_DIR"

BASE_LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
BASE_PYTHONPATH="${PYTHONPATH:-}"
BASE_CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-}"

VISION_PID=""
NAV_PID=""
TARGET_PID=""
TARGET_SNAPSHOT_PID=""
trace() {
  local event="$1" status="$2" detail="${3:-}"
  TRACE_EVENT="$event" TRACE_STATUS="$status" TRACE_DETAIL="$detail" TRACE_TX="$TRANSACTION_ID" \
    "$PYTHON_BIN" - "$TRACE_JSONL" <<'PY'
import datetime, json, os, sys
with open(sys.argv[1], "a", encoding="utf-8") as stream:
    stream.write(json.dumps({
        "time_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "transaction_id": os.environ["TRACE_TX"], "event": os.environ["TRACE_EVENT"],
        "status": os.environ["TRACE_STATUS"], "detail": os.environ["TRACE_DETAIL"],
    }, sort_keys=True) + "\n")
PY
}

die() { trace "stop" "failed" "$*" || true; printf '[race] STOP: %s (logs=%s)\n' "$*" "$LOG_DIR" >&2; exit 1; }
bool01() { [[ "$2" == 0 || "$2" == 1 ]] || die "$1 must be 0 or 1"; }
need_file() { [[ -f "$1" ]] || die "missing file: $1"; }

stop_vision() {
  if [[ -n "$VISION_PID" ]]; then
    kill -INT "$VISION_PID" 2>/dev/null || true
    wait "$VISION_PID" 2>/dev/null || true
    VISION_PID=""
    trace "vision" "stopped" "after_grasp_verification"
  fi
}
stop_target() {
  if [[ -n "$TARGET_PID" ]]; then
    kill -INT "$TARGET_PID" 2>/dev/null || true
    wait "$TARGET_PID" 2>/dev/null || true
    TARGET_PID=""
    trace "competition_target_node" "stopped" "after_grasp_verification"
  fi
}
cleanup() {
  if [[ -n "$TARGET_SNAPSHOT_PID" ]]; then kill -INT "$TARGET_SNAPSHOT_PID" 2>/dev/null || true; wait "$TARGET_SNAPSHOT_PID" 2>/dev/null || true; fi
  stop_target || true
  stop_vision || true
  if [[ -n "$NAV_PID" ]]; then kill -INT "$NAV_PID" 2>/dev/null || true; fi
}
trap cleanup EXIT INT TERM

source_ros1() {
  unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
  unset PYTHONPATH ROS_PACKAGE_PATH CMAKE_PREFIX_PATH
  export LD_LIBRARY_PATH="$BASE_LD_LIBRARY_PATH"
  [[ -z "$BASE_PYTHONPATH" ]] || export PYTHONPATH="$BASE_PYTHONPATH"
  [[ -z "$BASE_CMAKE_PREFIX_PATH" ]] || export CMAKE_PREFIX_PATH="$BASE_CMAKE_PREFIX_PATH"
  # shellcheck disable=SC1091
  source "${ROS1_SETUP:-/opt/ros/noetic/setup.bash}"
  # shellcheck disable=SC1091
  source "${MERCURY_ROS1_SETUP:-/home/elephant/mercury_x1_ros/devel/setup.bash}"
}

source_ros2() {
  unset ROS_DISTRO ROS_VERSION ROS_PACKAGE_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH
  unset PYTHONPATH CMAKE_PREFIX_PATH
  export LD_LIBRARY_PATH="$BASE_LD_LIBRARY_PATH"
  [[ -z "$BASE_PYTHONPATH" ]] || export PYTHONPATH="$BASE_PYTHONPATH"
  [[ -z "$BASE_CMAKE_PREFIX_PATH" ]] || export CMAKE_PREFIX_PATH="$BASE_CMAKE_PREFIX_PATH"
  # shellcheck disable=SC1091
  source "${ROS2_SETUP:-/opt/ros/galactic/setup.bash}"
  # shellcheck disable=SC1091
  source "$DEYES_INSTALL/setup.bash"
  export OpenCV_DIR="$OPENCV_PREFIX/lib/cmake/opencv4"
  export PKG_CONFIG_PATH="$OPENCV_PREFIX/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
  export LD_LIBRARY_PATH="/opt/ros/galactic/lib:/opt/ros/galactic/lib/aarch64-linux-gnu:$OPENCV_PREFIX/lib:${LD_LIBRARY_PATH:-}"
}

run_step() {
  local name="$1"; shift
  trace "$name" "started" "$*"
  if "$@" >"$LOG_DIR/$name.log" 2>&1; then
    trace "$name" "ok" ""
    return 0
  else
    local rc=$?
    tail -n 40 "$LOG_DIR/$name.log" >&2 || true
    trace "$name" "failed" "rc=$rc"
    return "$rc"
  fi
}

validate_engine_manifest() {
  need_file "$ENGINE_PATH"; need_file "$ENGINE_MANIFEST"
  EXPECTED_ENGINE_PATH="$ENGINE_PATH" EXPECTED_ONNX_SHA256="$EXPECTED_ONNX_SHA256" \
    "$PYTHON_BIN" - "$ENGINE_MANIFEST" >"$LOG_DIR/engine_validation.json" <<'PY'
import hashlib, json, os, pathlib, sys
manifest_path = pathlib.Path(sys.argv[1])
manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
required = {"onnx_sha256", "engine_sha256", "tensorrt_version", "cuda_version", "input_shape", "output_layout", "precision"}
missing = sorted(required - manifest.keys())
if missing: raise SystemExit("engine manifest missing: " + ",".join(missing))
if manifest["onnx_sha256"].lower() != os.environ["EXPECTED_ONNX_SHA256"]: raise SystemExit("manifest ONNX SHA mismatch")
if manifest["precision"].lower() != "fp16": raise SystemExit("engine precision is not FP16")
if manifest["input_shape"] != [1, 3, 416, 416]: raise SystemExit("engine input shape is not [1,3,416,416]")
if manifest["output_layout"] != "yolov5:[1,N,5+C]": raise SystemExit("unsupported engine output layout")
engine = pathlib.Path(os.environ["EXPECTED_ENGINE_PATH"])
actual = hashlib.sha256(engine.read_bytes()).hexdigest()
if actual != manifest["engine_sha256"].lower(): raise SystemExit("engine SHA does not match deployment sidecar")
print(json.dumps({"validated": True, "engine_sha256": actual, "manifest": str(manifest_path)}, sort_keys=True))
PY
  cp "$ENGINE_MANIFEST" "$LOG_DIR/engine_manifest.json"
  ENGINE_SHA256="$("$PYTHON_BIN" -c 'import json,sys; print(json.load(open(sys.argv[1]))["engine_sha256"])' "$ENGINE_MANIFEST")"
  export ENGINE_SHA256
}

validate_target_and_write_admission() {
  TARGET_FILE="$TARGET_JSON" ADMISSION_FILE="$ADMISSION_JSON" FIXED_MM="$FIXED_TABLE_HEIGHT_MM" \
    MAX_DELTA_MM="$GROUND_PLANE_MAX_DELTA_MM" \
    FORCE_FIXED="$FORCE_FIXED_TARGET" "$PYTHON_BIN" - <<'PY'
import json, os, pathlib
p = pathlib.Path(os.environ["TARGET_FILE"]); data = json.loads(p.read_text(encoding="utf-8"))
if data.get("schema") not in (None, "competition_pick_target/v1"): raise SystemExit("unsupported target schema")
forced = os.environ["FORCE_FIXED"] == "1"
if data.get("schema") == "competition_pick_target/v1":
    if data.get("valid") is not True:
        raise SystemExit("v1 target is not valid")
    if data.get("trusted_for_venue_execution") is not True and not forced:
        raise SystemExit("v1 target is not trusted for normal venue execution")
elif data.get("valid") is False:
    raise SystemExit("competition target is invalid")
elif data.get("trusted_for_venue_execution") is False and not forced:
    raise SystemExit("competition target is not trusted for normal venue execution")
if data.get("accepted") is False or (isinstance(data.get("admission"), dict) and data["admission"].get("accepted") is False):
    raise SystemExit("competition target rejected")
plane = data.get("ground_plane") if isinstance(data.get("ground_plane"), dict) else {}
verification_raw = data.get("height_verification")
verification = verification_raw if isinstance(verification_raw, dict) else {}
state = str(verification.get("status", plane.get("status", data.get("ground_plane_status", "missing")))).lower()
if isinstance(verification_raw, str): state = verification_raw.lower()
healthy = plane.get("healthy", data.get("ground_plane_healthy", state in {"ok", "healthy", "valid", "verified"})) is True
healthy = verification.get("verified", verification.get("healthy", healthy)) is True
height = verification.get("measured_table_height_mm", verification.get("observed_table_height_mm",
         verification.get("measured_height_mm", verification.get("table_height_mm",
         plane.get("table_height_mm", plane.get("height_mm", data.get("table_height_mm")))))))
delta = verification.get("delta_from_fixed_mm", verification.get("delta_mm"))
fixed = float(os.environ["FIXED_MM"]); max_delta = float(os.environ["MAX_DELTA_MM"])
declared_fixed = data.get("fixed_table_height_mm", verification.get("fixed_table_height_mm"))
declared_fixed_m = data.get("fixed_table_height_m", verification.get("fixed_table_height_m"))
if declared_fixed is not None and abs(float(declared_fixed) - fixed) > 1e-6: raise SystemExit("target fixed table height is not 650mm")
if declared_fixed_m is not None and abs(float(declared_fixed_m) * 1000.0 - fixed) > 1e-6: raise SystemExit("target fixed table height is not 0.650m")
mode = "healthy_ground_plane"
if healthy:
    if delta is not None:
        if abs(float(delta)) > max_delta: raise SystemExit("healthy ground plane conflicts with fixed 650mm table")
    elif height is not None:
        if abs(float(height) - fixed) > max_delta: raise SystemExit("healthy ground plane conflicts with fixed 650mm table")
    else: raise SystemExit("healthy ground plane omitted height/delta")
else:
    mode = "fixed_height_verified" if state == "fixed_height_verified" else "fixed_height_unverified"
if forced:
    xy = data.get("target_xy_mm", data.get("base_xy_mm"))
    sdk = data.get("right_arm_sdk_target_m")
    force_ok = xy in ([400, 10], [400.0, 10.0]) or (isinstance(sdk, list) and len(sdk) >= 2 and abs(float(sdk[0])-.4)<1e-6 and abs(float(sdk[1])-.01)<1e-6)
    if not force_ok: raise SystemExit("forced target is not [400,10]mm")
admission = {"accepted": True, "fixed_table_height_mm": fixed, "ground_plane_status": state,
             "ground_plane_height_mm": height, "mode": mode,
             "force_fixed_target": os.environ["FORCE_FIXED"] == "1"}
pathlib.Path(os.environ["ADMISSION_FILE"]).write_text(json.dumps(admission, indent=2, sort_keys=True)+"\n", encoding="utf-8")
PY
}

validate_grasp() {
  GRASP_FILE="$GRASP_JSON" "$PYTHON_BIN" - <<'PY'
import json, os, pathlib
data = json.loads(pathlib.Path(os.environ["GRASP_FILE"]).read_text(encoding="utf-8"))
verified = data.get("grasp_verified") is True or data.get("success") is True
navigation_permitted = data.get("navigation_permitted", verified) is True
if not verified or not navigation_permitted: raise SystemExit("grasp success/navigation_permitted gate failed")
PY
}

# ROS-free fault-matrix hook. It invokes the exact production validators but never
# sources ROS or starts hardware. It cannot report a live-system pass.
case "${COMPETITION_VALIDATE_ONLY:-}" in
  engine) validate_engine_manifest; exit $? ;;
  target) validate_target_and_write_admission; exit $? ;;
  grasp) validate_grasp; exit $? ;;
  "") ;;
  *) die "unknown COMPETITION_VALIDATE_ONLY mode" ;;
esac

bool01 ALLOW_BBOX_CENTER "$ALLOW_BBOX_CENTER"
bool01 ALLOW_FIXED_XY_FALLBACK "$ALLOW_FIXED_XY_FALLBACK"
bool01 FORCE_FIXED_TARGET "$FORCE_FIXED_TARGET"
[[ "$FIXED_TABLE_HEIGHT_MM" == "650" ]] || die "FIXED_TABLE_HEIGHT_MM is competition-locked to 650"
need_file "$CALIB_PATH"; need_file "$DETECTOR_CONFIG"; need_file "$SITE_METADATA_PATH"
need_file "$DEYES_INSTALL/setup.bash"; need_file "$PICK_SCRIPT"; need_file "$PLACE_SCRIPT"
need_file "$TARGET_ADAPTER"
"$DEYES_ASSETS/probe_opencv_cuda.sh" "$OPENCV_PREFIX" >"$LOG_DIR/cuda_probe.log" 2>&1 || die "CUDA OpenCV probe failed"
validate_engine_manifest || die "engine sidecar validation failed"
trace "transaction" "started" "log_dir=$LOG_DIR"

source_ros1
if ! rostopic list 2>/dev/null | grep -qx /move_base/goal; then
  roslaunch turn_on_mercury_robot navigation.launch >"$LOG_DIR/navigation.log" 2>&1 & NAV_PID=$!
  ready=0
  for _ in $(seq 1 "${NAV_READY_POLLS:-30}"); do
    rostopic list 2>/dev/null | grep -qx /move_base/goal && { ready=1; break; }
    sleep 2
  done
  [[ "$ready" == 1 ]] || die "move_base unavailable"
fi
run_step nav_goal3 "$PYTHON_BIN" "$RAC_SCRIPTS/send_one_goal.py" "$NAV_GOAL3" || die "navigation goal3 failed"
run_step set_head "$PYTHON_BIN" "$RAC_SCRIPTS/set_venue_head.py" || die "head command/feedback failed"

source_ros2
ros2 launch deyes_bringup imx219_stereo.launch.py \
  use_cpp_capture:=true width:=640 height:=360 fps:=30 swap_left_right:=true \
  calib_path:="$CALIB_PATH" enable_cuda_depth:=true cuda_depth_publish_debug_rect:=true \
  cuda_depth_max_sync_diff_ms:=10.0 enable_ground_plane:=true \
  enable_detector:=true detector_config:="$DETECTOR_CONFIG" detector_backend:=tensorrt \
  detector_model_path:="$ENGINE_PATH" detector_model_id:=pen-yolov5-student-01875-416-v1 \
  detector_expected_model_sha256:="$ENGINE_SHA256" detector_input_width:=416 detector_input_height:=416 \
  detector_expected_class_count:=1 detector_expected_max_targets:=1 enable_pen_features:=true \
  >"$LOG_DIR/vision.log" 2>&1 & VISION_PID=$!
sleep "${VISION_STARTUP_SEC:-5}"
kill -0 "$VISION_PID" 2>/dev/null || { tail -n 60 "$LOG_DIR/vision.log" >&2 || true; die "vision launch exited"; }
trace "vision" "ready" "pid=$VISION_PID"

bbox_bool=false; fixed_xy_bool=false; force_bool=false
[[ "$ALLOW_BBOX_CENTER" == 1 ]] && bbox_bool=true
[[ "$ALLOW_FIXED_XY_FALLBACK" == 1 ]] && fixed_xy_bool=true
[[ "$FORCE_FIXED_TARGET" == 1 ]] && force_bool=true
trace "competition_target" "started" "topic=$TARGET_TOPIC"
"$PYTHON_BIN" "$TARGET_ADAPTER" --topic "$TARGET_TOPIC" --output "$TARGET_JSON" --timeout "$TARGET_TIMEOUT_SEC" \
  >"$LOG_DIR/competition_target.log" 2>&1 & TARGET_SNAPSHOT_PID=$!
sleep "${TARGET_SUBSCRIBER_READY_SEC:-0.5}"
ros2 run "$TARGET_PACKAGE" "$TARGET_EXECUTABLE" --ros-args \
  -p venue_profile_path:="$SITE_METADATA_PATH" -p fixed_table_height_m:="0.650" \
  -p allow_bbox_center:="$bbox_bool" -p allow_fixed_xy_fallback:="$fixed_xy_bool" \
  -p force_fixed_target:="$force_bool" >"$LOG_DIR/competition_target_node.log" 2>&1 & TARGET_PID=$!
sleep "${TARGET_STARTUP_SEC:-2}"
kill -0 "$TARGET_PID" 2>/dev/null || { tail -n 40 "$LOG_DIR/competition_target_node.log" >&2 || true; die "competition target node exited"; }
if wait "$TARGET_SNAPSHOT_PID"; then
  TARGET_SNAPSHOT_PID=""; trace "competition_target" "ok" ""
else
  rc=$?; TARGET_SNAPSHOT_PID=""; tail -n 40 "$LOG_DIR/competition_target.log" >&2 || true
  trace "competition_target" "failed" "rc=$rc"
  die "competition target snapshot failed (placeholder/waiting payloads are not executable)"
fi
need_file "$TARGET_JSON"
validate_target_and_write_admission || die "target/ground-plane admission failed"
trace "admission" "ok" "$(tr -d '\n' < "$ADMISSION_JSON")"

mapfile -t pick_xy < <(TARGET_FILE="$TARGET_JSON" "$PYTHON_BIN" - <<'PY'
import json,os,pathlib
d=json.loads(pathlib.Path(os.environ['TARGET_FILE']).read_text())
sdk=d.get('right_arm_sdk_target_m')
if isinstance(sdk,list) and len(sdk)>=2: print(float(sdk[0])*1000); print(float(sdk[1])*1000)
else:
    xy=d.get('target_xy_mm',d.get('base_xy_mm'))
    if not isinstance(xy,list) or len(xy)<2: raise SystemExit('target XY missing')
    print(float(xy[0])); print(float(xy[1]))
PY
)
[[ "${#pick_xy[@]}" == 2 ]] || die "target XY extraction failed"
run_step pick "$PYTHON_BIN" "$PICK_SCRIPT" --x-mm "${pick_xy[0]}" --y-mm "${pick_xy[1]}" --venue-profile "$SITE_METADATA_PATH" --result-json "$GRASP_JSON" || die "pick command/serial/feedback failed"
need_file "$GRASP_JSON"
validate_grasp || die "grasp verification failed"
trace "grasp_verification" "ok" "success=true navigation_permitted=true"
stop_target
stop_vision

source_ros1
run_step nav_goal4 "$PYTHON_BIN" "$RAC_SCRIPTS/send_one_goal.py" "$NAV_GOAL4" || die "navigation goal4 failed"
run_step place "$PYTHON_BIN" "$PLACE_SCRIPT" --venue-profile "$SITE_METADATA_PATH" || die "place command/serial/feedback failed"
trace "transaction" "complete" ""
printf '[race] COMPLETE transaction=%s logs=%s\n' "$TRANSACTION_ID" "$LOG_DIR"
