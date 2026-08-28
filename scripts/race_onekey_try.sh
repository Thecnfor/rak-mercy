#!/usr/bin/env bash
# Competition transaction with fail-closed hardware gates and truthful showcase continuation.
set -Eeuo pipefail

RAC_SCRIPTS="${RAC_SCRIPTS:-$HOME/scripts}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
DEYES_WS="${DEYES_WS:-$HOME/deyes_competition_ws}"
DEYES_INSTALL="${DEYES_INSTALL:-$DEYES_WS/install}"
DEYES_ASSETS="${DEYES_ASSETS:-$HOME/deyes_competition_assets}"
OPENCV_PREFIX="${DEYES_OPENCV_PREFIX:-$HOME/opencv-4.8.0-cuda}"
CALIB_PATH="${DEYES_CALIB_PATH:-$DEYES_ASSETS/venue_20260827_quick_stereo.yaml}"
PROJECTOR_PATH="${DEYES_PROJECTOR_PATH:-$DEYES_ASSETS/venue_20260827_touch_projector.yaml}"
DETECTOR_CONFIG="${DEYES_DETECTOR_CONFIG:-$DEYES_ASSETS/competition_fixed_scene.yaml}"
SITE_METADATA_PATH="${COMPETITION_SITE_METADATA:-$DEYES_ASSETS/competition_venue_65cm.yaml}"
ENGINE_PATH="${DEYES_ENGINE_PATH:-$HOME/temp/deyes/models/pen/pen-yolov5-student-01875-416-v1.fp16.engine}"
ENGINE_MANIFEST="${DEYES_ENGINE_MANIFEST:-${ENGINE_PATH}.manifest.json}"
EXPECTED_ONNX_SHA256="8916cbf25949d6e8b03c01e6ca1c7871aeac0ad105c931f1dd9881cb5d5a4c4e"

FIXED_TABLE_HEIGHT_MM="${FIXED_TABLE_HEIGHT_MM:-650}"
ALLOW_BBOX_CENTER="${ALLOW_BBOX_CENTER:-0}"
ALLOW_FIXED_XY_FALLBACK="${ALLOW_FIXED_XY_FALLBACK:-0}"
FORCE_FIXED_TARGET="${FORCE_FIXED_TARGET:-0}"
COMPETITION_SHOWCASE_CONTINUE="${COMPETITION_SHOWCASE_CONTINUE:-1}"
GROUND_PLANE_MAX_DELTA_MM="${GROUND_PLANE_MAX_DELTA_MM:-25}"
TARGET_TIMEOUT_SEC="${TARGET_TIMEOUT_SEC:-30}"

TARGET_PACKAGE="${COMPETITION_TARGET_PACKAGE:-deyes_stereo}"
TARGET_EXECUTABLE="${COMPETITION_TARGET_EXECUTABLE:-competition_pick_target}"
TARGET_TOPIC="${COMPETITION_TARGET_TOPIC:-/x1/competition/pick_target}"
TARGET_ADAPTER="${COMPETITION_TARGET_ADAPTER:-$RAC_SCRIPTS/competition_target_snapshot_adapter.py}"
GRASP_FEEDBACK_ADAPTER="${COMPETITION_GRASP_FEEDBACK_ADAPTER:-$RAC_SCRIPTS/competition_grasp_feedback_adapter.py}"
PICK_SCRIPT="${COMPETITION_PICK_SCRIPT:-$RAC_SCRIPTS/pick_pen_degraded.py}"
PLACE_SCRIPT="${COMPETITION_PLACE_SCRIPT:-$RAC_SCRIPTS/place_pen_degraded.py}"
ARM_PREPARE_SCRIPT="${COMPETITION_ARM_PREPARE_SCRIPT:-$RAC_SCRIPTS/prepare_competition_arms.py}"
NAV_GOAL3="${COMPETITION_NAV_GOAL3:-goal3_right}"
NAV_GOAL4="${COMPETITION_NAV_GOAL4:-goal4_back}"

TRANSACTION_ID="${COMPETITION_TRANSACTION_ID:-$(date -u +%Y%m%dT%H%M%SZ)_$$}"
LOG_DIR="${LOG_DIR:-$HOME/temp/deyes/competition/$TRANSACTION_ID}"
TARGET_JSON="$LOG_DIR/target.json"
RETRY_TARGET_JSON="$LOG_DIR/retry_target.json"
SHOWCASE_TARGET_JSON="$LOG_DIR/showcase_target.json"
ADMISSION_JSON="$LOG_DIR/admission.json"
RETRY_ADMISSION_JSON="$LOG_DIR/retry_admission.json"
GRASP_JSON="$LOG_DIR/grasp_verification.json"
GRASP_FEEDBACK_JSON="$LOG_DIR/grasp_feedback.json"
PICK_DECISION_JSON="$LOG_DIR/pick_decision.json"
RETRY_GRASP_JSON="$LOG_DIR/retry_grasp_verification.json"
RETRY_GRASP_FEEDBACK_JSON="$LOG_DIR/retry_grasp_feedback.json"
RETRY_PICK_DECISION_JSON="$LOG_DIR/retry_pick_decision.json"
PLACE_JSON="$LOG_DIR/place.json"
ARM_STOW_JSON="$LOG_DIR/arm_stow.json"
TRANSACTION_RESULT_JSON="$LOG_DIR/transaction_result.json"
TRACE_JSONL="$LOG_DIR/trace.jsonl"
mkdir -p "$LOG_DIR"

BASE_LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
BASE_PYTHONPATH="${PYTHONPATH:-}"
BASE_CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-}"

VISION_PID=""
NAV_PID=""
TARGET_PID=""
TARGET_SNAPSHOT_PID=""
TARGET_SOURCE="none"
EXECUTION_TARGET_JSON=""
DEGRADED_REASONS=""
COMPETITION_SUCCESS=0
PICK_COMPETITION_VERIFIED=0
SHOWCASE_COMPLETE=0
OBJECT_GRASP_VERIFIED=0
PICK_MOTION_COMPLETED=0
PICK_ATTEMPT_COUNT=0
TRANSPORT_POSE_REACHED=0
GOAL4_COMPLETED=0
PLACE_MOTION_COMPLETED=0
HARD_STOP_REASON=""
COMMANDS_EMITTED=0

resolve_deyes_python_site() {
  local python_root
  python_root="$(
    set +u
    unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH ROS_DISTRO ROS_VERSION ROS_PACKAGE_PATH CMAKE_PREFIX_PATH
    # shellcheck disable=SC1091
    source "${ROS2_SETUP:-/opt/ros/galactic/setup.bash}"
    # shellcheck disable=SC1091
    source "$DEYES_INSTALL/setup.bash"
    "$PYTHON_BIN" -c 'import pathlib, deyes_stereo; print(pathlib.Path(deyes_stereo.__file__).resolve().parent.parent)'
  )"
  [[ -n "$python_root" && -d "$python_root" ]] || die "installed deyes_stereo Python package not found under $DEYES_INSTALL"
  printf '%s\n' "$python_root"
}

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

add_degraded_reason() {
  local reason="$1"
  DEGRADED_REASONS+="${DEGRADED_REASONS:+$'\n'}$reason"
}

write_transaction_result() {
  TX_RESULT="$TRANSACTION_RESULT_JSON" TX_ID="$TRANSACTION_ID" SHOWCASE_MODE="$COMPETITION_SHOWCASE_CONTINUE" \
    COMPETITION_OK="$COMPETITION_SUCCESS" SHOWCASE_OK="$SHOWCASE_COMPLETE" OBJECT_OK="$OBJECT_GRASP_VERIFIED" \
    TARGET_KIND="$TARGET_SOURCE" PICK_OK="$PICK_MOTION_COMPLETED" TRANSPORT_OK="$TRANSPORT_POSE_REACHED" \
    GOAL4_OK="$GOAL4_COMPLETED" PLACE_OK="$PLACE_MOTION_COMPLETED" DEGRADED="$DEGRADED_REASONS" \
    HARD_STOP="$HARD_STOP_REASON" COMMANDS_SENT="$COMMANDS_EMITTED" PICK_ATTEMPTS="$PICK_ATTEMPT_COUNT" "$PYTHON_BIN" - <<'PY'
import json, os, pathlib

def flag(name): return os.environ[name] == "1"
payload = {
    "schema": "competition_transaction_result/v1",
    "transaction_id": os.environ["TX_ID"],
    "showcase_mode": flag("SHOWCASE_MODE"),
    "competition_success": flag("COMPETITION_OK"),
    "showcase_complete": flag("SHOWCASE_OK"),
    "object_grasp_verified": flag("OBJECT_OK"),
    "target_source": os.environ["TARGET_KIND"],
    "pick_motion_completed": flag("PICK_OK"),
    "transport_pose_reached": flag("TRANSPORT_OK"),
    "goal4_completed": flag("GOAL4_OK"),
    "place_motion_completed": flag("PLACE_OK"),
    "degraded_reasons": [value for value in os.environ["DEGRADED"].splitlines() if value],
    "hard_stop_reason": os.environ["HARD_STOP"] or None,
    "commands_emitted": flag("COMMANDS_SENT"),
    "pick_attempt_count": int(os.environ["PICK_ATTEMPTS"]),
}
if payload["pick_attempt_count"] not in (0, 1, 2):
    raise SystemExit("invalid pick attempt count")
if payload["competition_success"] and not (payload["showcase_complete"] and payload["object_grasp_verified"]):
    raise SystemExit("invalid competition success terminal state")
if payload["showcase_complete"] and not all(payload[name] for name in (
        "pick_motion_completed", "transport_pose_reached", "goal4_completed", "place_motion_completed")):
    raise SystemExit("invalid showcase completion terminal state")
pathlib.Path(os.environ["TX_RESULT"]).write_text(json.dumps(payload, indent=2, sort_keys=True)+"\n", encoding="utf-8")
PY
}

die() {
  HARD_STOP_REASON="$*"
  write_transaction_result || true
  trace "stop" "failed" "$*" || true
  printf '[race] STOP: %s (logs=%s)\n' "$*" "$LOG_DIR" >&2
  exit 1
}
bool01() { [[ "$2" == 0 || "$2" == 1 ]] || die "$1 must be 0 or 1"; }
need_file() { [[ -f "$1" ]] || die "missing file: $1"; }

select_showcase_target() {
  local reason="$1"
  [[ "$COMPETITION_SHOWCASE_CONTINUE" == 1 ]] || die "$reason"
  if ! SHOWCASE_REASON="$reason" SHOWCASE_OUTPUT="$SHOWCASE_TARGET_JSON" SHOWCASE_SITE="$SITE_METADATA_PATH" "$PYTHON_BIN" - <<'PY'
import json, os, pathlib
import yaml
from deyes_stereo.competition_showcase_contract import (
    build_showcase_target,
    classify_showcase_failure,
    runtime_perception_failure_code,
    validate_showcase_site,
)
failure_code = runtime_perception_failure_code(os.environ["SHOWCASE_REASON"])
if classify_showcase_failure(failure_code, showcase_enabled=True) != "continue_showcase":
    raise SystemExit(f"showcase policy hard stop:{failure_code}")
validate_showcase_site(yaml.safe_load(pathlib.Path(os.environ["SHOWCASE_SITE"]).read_text(encoding="utf-8")))
payload = build_showcase_target(os.environ["SHOWCASE_REASON"])
assert payload["schema"] == "competition_showcase_target/v1"
pathlib.Path(os.environ["SHOWCASE_OUTPUT"]).write_text(json.dumps(payload, indent=2, sort_keys=True)+"\n", encoding="utf-8")
PY
  then
    die "showcase target contract failed:$reason"
  fi
  TARGET_SOURCE="fixed_marker_showcase"
  EXECUTION_TARGET_JSON="$SHOWCASE_TARGET_JSON"
  add_degraded_reason "$reason"
  trace "showcase_target" "degraded" "reason=$reason marker_xy_mm=[400,10] sensor_target_available=false"
}

terminate_pid() {
  local pid="${1:-}" signal_name="${2:-process}"
  [[ -n "$pid" ]] || return 0
  if kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
    for _ in $(seq 1 "${PROCESS_INT_POLLS:-30}"); do kill -0 "$pid" 2>/dev/null || break; sleep .1; done
  fi
  if kill -0 "$pid" 2>/dev/null; then
    trace "$signal_name" "stop_escalated" "SIGTERM pid=$pid"
    kill -TERM "$pid" 2>/dev/null || true
    for _ in $(seq 1 "${PROCESS_TERM_POLLS:-20}"); do kill -0 "$pid" 2>/dev/null || break; sleep .1; done
  fi
  if kill -0 "$pid" 2>/dev/null; then
    trace "$signal_name" "stop_escalated" "SIGKILL pid=$pid"
    kill -KILL "$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
}

stop_vision() {
  if [[ -n "$VISION_PID" ]]; then
    terminate_pid "$VISION_PID" vision
    VISION_PID=""
    trace "vision" "stopped" "after_grasp_verification"
  fi
}
stop_target() {
  if [[ -n "$TARGET_PID" ]]; then
    terminate_pid "$TARGET_PID" competition_target_node
    TARGET_PID=""
    trace "competition_target_node" "stopped" "after_single_target_snapshot"
  fi
}
cleanup() {
  if [[ -n "$TARGET_SNAPSHOT_PID" ]]; then terminate_pid "$TARGET_SNAPSHOT_PID" target_snapshot_adapter; TARGET_SNAPSHOT_PID=""; fi
  stop_target || true
  stop_vision || true
  if [[ -n "$NAV_PID" ]]; then terminate_pid "$NAV_PID" navigation_launch; NAV_PID=""; fi
}
on_signal() {
  HARD_STOP_REASON="signal:$1"
  write_transaction_result || true
  trace "signal" "interrupted" "$1" || true
  trap - INT TERM
  exit "$2"
}
trap cleanup EXIT
trap 'on_signal INT 130' INT
trap 'on_signal TERM 143' TERM

source_ros1() {
  local deyes_site
  deyes_site="$(resolve_deyes_python_site)"
  unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
  unset PYTHONPATH ROS_PACKAGE_PATH CMAKE_PREFIX_PATH
  export LD_LIBRARY_PATH="$BASE_LD_LIBRARY_PATH"
  [[ -z "$BASE_PYTHONPATH" ]] || export PYTHONPATH="$BASE_PYTHONPATH"
  [[ -z "$BASE_CMAKE_PREFIX_PATH" ]] || export CMAKE_PREFIX_PATH="$BASE_CMAKE_PREFIX_PATH"
  # shellcheck disable=SC1091
  source "${ROS1_SETUP:-/opt/ros/noetic/setup.bash}"
  # shellcheck disable=SC1091
  source "${MERCURY_ROS1_SETUP:-/home/elephant/mercury_x1_ros/devel/setup.bash}"
  # place_pen_degraded imports the ROS-free execution contract after ROS 1 is
  # sourced; retain only the installed Deyes Python package path explicitly.
  export PYTHONPATH="$deyes_site${PYTHONPATH:+:$PYTHONPATH}"
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
required = {"onnx_sha256", "engine_sha256", "tensorrt_version", "cuda_version", "input_shape", "output_layout", "precision", "bindings"}
missing = sorted(required - manifest.keys())
if missing: raise SystemExit("engine manifest missing: " + ",".join(missing))
if manifest["onnx_sha256"].lower() != os.environ["EXPECTED_ONNX_SHA256"]: raise SystemExit("manifest ONNX SHA mismatch")
if manifest["precision"].lower() != "fp16": raise SystemExit("engine precision is not FP16")
if manifest["input_shape"] != [1, 3, 416, 416]: raise SystemExit("engine input shape is not [1,3,416,416]")
if manifest["output_layout"] != "yolov5:[1,N,5+C]": raise SystemExit("unsupported engine output layout")
bindings = manifest["bindings"]
if not isinstance(bindings, list) or len(bindings) != 2:
    raise SystemExit("engine bindings must contain exactly one input and one output")
for binding in bindings:
    if not isinstance(binding, dict): raise SystemExit("engine binding entry must be an object")
    binding_missing = sorted({"index", "name", "io", "shape", "dtype"} - binding.keys())
    if binding_missing: raise SystemExit("engine binding missing: " + ",".join(binding_missing))
    if not isinstance(binding["index"], int) or binding["index"] < 0: raise SystemExit("engine binding index is invalid")
    if not isinstance(binding["name"], str) or not binding["name"]: raise SystemExit("engine binding name is invalid")
    if binding["io"] not in ("input", "output"): raise SystemExit("engine binding io must be input or output")
    if not isinstance(binding["shape"], list) or not all(isinstance(value, int) for value in binding["shape"]):
        raise SystemExit("engine binding shape is invalid")
if len({binding["index"] for binding in bindings}) != 2 or len({binding["name"] for binding in bindings}) != 2:
    raise SystemExit("engine binding indexes and names must be unique")
inputs = [binding for binding in bindings if binding["io"] == "input"]
outputs = [binding for binding in bindings if binding["io"] == "output"]
if len(inputs) != 1 or len(outputs) != 1: raise SystemExit("engine bindings must contain exactly one input and one output")
if inputs[0]["shape"] != [1, 3, 416, 416] or inputs[0]["dtype"] != "float32":
    raise SystemExit("engine input binding ABI mismatch")
output_shape = outputs[0]["shape"]
if len(output_shape) != 3 or output_shape[0] != 1 or output_shape[1] <= 0 or output_shape[2] != 6:
    raise SystemExit("engine output binding ABI mismatch for one-class YOLOv5")
if outputs[0]["dtype"] not in ("float16", "float32"):
    raise SystemExit("engine output binding dtype mismatch")
engine = pathlib.Path(os.environ["EXPECTED_ENGINE_PATH"])
actual = hashlib.sha256(engine.read_bytes()).hexdigest()
if actual != manifest["engine_sha256"].lower(): raise SystemExit("engine SHA does not match deployment sidecar")
print(json.dumps({"validated": True, "engine_sha256": actual, "manifest": str(manifest_path), "bindings": bindings}, sort_keys=True))
PY
  cp "$ENGINE_MANIFEST" "$LOG_DIR/engine_manifest.json"
  ENGINE_SHA256="$("$PYTHON_BIN" -c 'import json,sys; print(json.load(open(sys.argv[1]))["engine_sha256"])' "$ENGINE_MANIFEST")"
  export ENGINE_SHA256
}

validate_target_and_write_admission() {
  local target_file="${1:-$TARGET_JSON}" admission_file="${2:-$ADMISSION_JSON}"
  local force_fixed="${3:-$FORCE_FIXED_TARGET}" allow_fixed="${4:-$ALLOW_FIXED_XY_FALLBACK}"
  TARGET_FILE="$target_file" ADMISSION_FILE="$admission_file" FIXED_MM="$FIXED_TABLE_HEIGHT_MM" \
    MAX_DELTA_MM="$GROUND_PLANE_MAX_DELTA_MM" \
    FORCE_FIXED="$force_fixed" ALLOW_FIXED="$allow_fixed" "$PYTHON_BIN" - <<'PY'
import json, math, os, pathlib
p = pathlib.Path(os.environ["TARGET_FILE"]); data = json.loads(p.read_text(encoding="utf-8"))
if data.get("schema") != "competition_pick_target/v1": raise SystemExit("target schema must be competition_pick_target/v1")
forced = os.environ["FORCE_FIXED"] == "1"
allow_fixed = os.environ["ALLOW_FIXED"] == "1"
if forced != allow_fixed: raise SystemExit("fixed XY fallback and forced target must be enabled together")
if data.get("valid") is not True: raise SystemExit("v1 target is not valid")
if data.get("commands_emitted") is not False: raise SystemExit("target commands_emitted must be false")
if data.get("execution_allowed") is not True: raise SystemExit("target execution_allowed must be true")
source = data.get("selection_source")
sdk = data.get("right_arm_sdk_target_m")
orientation = data.get("orientation_deg")
if not isinstance(sdk, list) or len(sdk) != 3: raise SystemExit("target right_arm_sdk_target_m missing")
sdk = [float(value) for value in sdk]
if not all(math.isfinite(value) for value in sdk): raise SystemExit("target coordinates are not finite")
if abs(sdk[2]-.135) > 1e-6: raise SystemExit("target contact Z is not 135mm")
if not isinstance(orientation, list) or len(orientation) != 3: raise SystemExit("target orientation missing")
orientation = [float(value) for value in orientation]
if not all(math.isfinite(value) for value in orientation) or any(abs(a-b)>1e-6 for a,b in zip(orientation,(179.99,-12.,0.))):
    raise SystemExit("target orientation is not [179.99,-12,0]deg")
if forced:
    if source != "fixed_xy_fallback": raise SystemExit("forced target is not an explicit fixed_xy_fallback")
    if data.get("trusted_for_venue_execution") is not False or data.get("projector_usable_and_validated") is not False:
        raise SystemExit("fixed fallback must preserve unusable/untrusted projector evidence")
    if data.get("degraded") is not True or data.get("degraded_mode") != "forced_fixed_xy_marker" or data.get("force_fixed_target") is not True:
        raise SystemExit("fixed fallback degraded metadata missing")
    if "[400,10]mm" not in str(data.get("manual_action_required", "")):
        raise SystemExit("fixed fallback marker instruction missing")
else:
    if source == "fixed_xy_fallback": raise SystemExit("fixed XY target requires explicit force")
    if data.get("trusted_for_venue_execution") is not True or data.get("projector_usable_and_validated") is not True:
        raise SystemExit("target is not trusted for normal venue execution")
    if data.get("degraded") is not False: raise SystemExit("normal target cannot be marked degraded")
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
if not math.isfinite(fixed) or not math.isfinite(max_delta) or max_delta < 0: raise SystemExit("table height policy is not finite")
declared_fixed = data.get("fixed_table_height_mm", verification.get("fixed_table_height_mm"))
declared_fixed_m = data.get("fixed_table_height_m", verification.get("fixed_table_height_m"))
if declared_fixed is not None and (not math.isfinite(float(declared_fixed)) or abs(float(declared_fixed) - fixed) > 1e-6): raise SystemExit("target fixed table height is not 650mm")
if declared_fixed_m is not None and (not math.isfinite(float(declared_fixed_m)) or abs(float(declared_fixed_m) * 1000.0 - fixed) > 1e-6): raise SystemExit("target fixed table height is not 0.650m")
if height is not None and not math.isfinite(float(height)): raise SystemExit("ground plane height is not finite")
if delta is not None and not math.isfinite(float(delta)): raise SystemExit("ground plane delta is not finite")
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
    force_ok = xy in ([400, 10], [400.0, 10.0]) or (abs(sdk[0]-.4)<1e-6 and abs(sdk[1]-.01)<1e-6)
    if not force_ok: raise SystemExit("forced target is not [400,10]mm")
    mode = "fixed_xy_degraded"
admission = {"accepted": True, "fixed_table_height_mm": fixed, "ground_plane_status": state,
             "ground_plane_height_mm": height, "mode": mode,
             "force_fixed_target": os.environ["FORCE_FIXED"] == "1"}
pathlib.Path(os.environ["ADMISSION_FILE"]).write_text(json.dumps(admission, indent=2, sort_keys=True)+"\n", encoding="utf-8")
PY
}

validate_grasp() {
  GRASP_FILE="$GRASP_JSON" "$PYTHON_BIN" - <<'PY'
import json, math, os, pathlib
data = json.loads(pathlib.Path(os.environ["GRASP_FILE"]).read_text(encoding="utf-8"))
evidence = data.get("feedback_evidence")
if data.get("schema") != "competition_grasp_verification/v1": raise SystemExit("grasp verification schema mismatch")
if data.get("success") is not True or data.get("navigation_permitted") is not True: raise SystemExit("grasp success/navigation_permitted gate failed")
if data.get("single_attempt_latched") is not True or data.get("condition_b_roi_clear_3_and_feedback_delta_5") is not True:
    raise SystemExit("grasp single-attempt sensor condition was not proven")
if not isinstance(evidence, dict) or evidence.get("schema") != "competition_grasp_feedback/v1": raise SystemExit("grasp feedback evidence missing")
if evidence.get("live") is not True or evidence.get("source") != "live_ros2_and_mercury_feedback": raise SystemExit("grasp feedback is not live")
delta = float(evidence.get("gripper_feedback_delta", float("-inf")))
if evidence.get("roi_pen_last3") != [False, False, False] or evidence.get("detector_frames_last3_ambiguous") != [False, False, False] or not math.isfinite(delta) or delta < 5.0:
    raise SystemExit("grasp feedback does not prove ROI clear and gripper delta")
PY
}

validate_place() {
  PLACE_FILE="$PLACE_JSON" "$PYTHON_BIN" - <<'PY'
import json, os, pathlib
data = json.loads(pathlib.Path(os.environ["PLACE_FILE"]).read_text(encoding="utf-8"))
if (data.get("schema") != "competition_place_execution/v1" or data.get("success") is not True
        or data.get("motion_completed") is not True or data.get("commands_emitted") is not True):
    raise SystemExit("place completion result is not successful")
state=data.get("object_state")
if state not in ("verified","unverified"): raise SystemExit("place object_state is invalid")
if data.get("object_delivery_verified") is not (state=="verified"):
    raise SystemExit("place object delivery truth mismatch")
if state=="unverified" and data.get("showcase_mode") is not True:
    raise SystemExit("unverified place requires showcase mode")
PY
}

capture_target_once() {
  local output_json="$1" label="$2" strict_retry="${3:-0}"
  local adapter_log="$LOG_DIR/${label}_target.log"
  local node_log="$LOG_DIR/${label}_target_node.log"
  local bbox_bool=false fixed_xy_bool=false force_bool=false rc=0
  [[ "$ALLOW_BBOX_CENTER" == 1 ]] && bbox_bool=true
  if [[ "$strict_retry" != 1 ]]; then
    [[ "$ALLOW_FIXED_XY_FALLBACK" == 1 ]] && fixed_xy_bool=true
    [[ "$FORCE_FIXED_TARGET" == 1 ]] && force_bool=true
  fi
  trace "$label" "started" "topic=$TARGET_TOPIC strict_retry=$strict_retry"
  "$PYTHON_BIN" "$TARGET_ADAPTER" --topic "$TARGET_TOPIC" --output "$output_json" --timeout "$TARGET_TIMEOUT_SEC" \
    >"$adapter_log" 2>&1 & TARGET_SNAPSHOT_PID=$!
  sleep "${TARGET_SUBSCRIBER_READY_SEC:-0.5}"
  ros2 run "$TARGET_PACKAGE" "$TARGET_EXECUTABLE" --ros-args \
    -p venue_profile_path:="$SITE_METADATA_PATH" -p projector_path:="$PROJECTOR_PATH" -p fixed_table_height_m:="0.650" \
    -p allow_bbox_center:="$bbox_bool" -p allow_fixed_xy_fallback:="$fixed_xy_bool" \
    -p force_fixed_target:="$force_bool" >"$node_log" 2>&1 & TARGET_PID=$!
  sleep "${TARGET_STARTUP_SEC:-2}"

  if ! kill -0 "$TARGET_SNAPSHOT_PID" 2>/dev/null; then
    if wait "$TARGET_SNAPSHOT_PID"; then rc=0; else rc=$?; fi
  elif ! kill -0 "$TARGET_PID" 2>/dev/null; then
    for _ in $(seq 1 "${TARGET_REJECTION_GRACE_POLLS:-10}"); do
      kill -0 "$TARGET_SNAPSHOT_PID" 2>/dev/null || break
      sleep .1
    done
    if kill -0 "$TARGET_SNAPSHOT_PID" 2>/dev/null; then
      terminate_pid "$TARGET_SNAPSHOT_PID" "${label}_adapter"
      rc=1
    elif wait "$TARGET_SNAPSHOT_PID"; then
      rc=0
    else
      rc=$?
    fi
  elif wait "$TARGET_SNAPSHOT_PID"; then
    rc=0
  else
    rc=$?
  fi
  TARGET_SNAPSHOT_PID=""
  stop_target
  if [[ "$rc" == 0 && -f "$output_json" ]]; then
    trace "$label" "ok" "output=$output_json"
    return 0
  fi
  trace "$label" "failed" "rc=$rc detail=$(tail -n 1 "$adapter_log" 2>/dev/null || true)"
  return 1
}

# ROS-free fault-matrix hook. It invokes the exact production validators but never
# sources ROS or starts hardware. It cannot report a live-system pass.
case "${COMPETITION_VALIDATE_ONLY:-}" in
  engine) validate_engine_manifest; exit $? ;;
  target) validate_target_and_write_admission; exit $? ;;
  grasp) validate_grasp; exit $? ;;
  place) validate_place; exit $? ;;
  "") ;;
  *) die "unknown COMPETITION_VALIDATE_ONLY mode" ;;
esac

bool01 ALLOW_BBOX_CENTER "$ALLOW_BBOX_CENTER"
bool01 ALLOW_FIXED_XY_FALLBACK "$ALLOW_FIXED_XY_FALLBACK"
bool01 FORCE_FIXED_TARGET "$FORCE_FIXED_TARGET"
bool01 COMPETITION_SHOWCASE_CONTINUE "$COMPETITION_SHOWCASE_CONTINUE"
[[ "$ALLOW_FIXED_XY_FALLBACK" == "$FORCE_FIXED_TARGET" ]] || die "fixed XY fallback and FORCE_FIXED_TARGET must be enabled together"
[[ "$FIXED_TABLE_HEIGHT_MM" == "650" ]] || die "FIXED_TABLE_HEIGHT_MM is competition-locked to 650"
need_file "$CALIB_PATH"; need_file "$PROJECTOR_PATH"; need_file "$DETECTOR_CONFIG"; need_file "$SITE_METADATA_PATH"
need_file "$DEYES_INSTALL/setup.bash"; need_file "$PICK_SCRIPT"; need_file "$PLACE_SCRIPT"; need_file "$ARM_PREPARE_SCRIPT"
need_file "$TARGET_ADAPTER"; need_file "$GRASP_FEEDBACK_ADAPTER"
"$DEYES_ASSETS/probe_opencv_cuda.sh" "$OPENCV_PREFIX" >"$LOG_DIR/cuda_probe.log" 2>&1 || die "CUDA OpenCV probe failed"
validate_engine_manifest || die "engine sidecar validation failed"
trace "transaction" "started" "log_dir=$LOG_DIR"

# The hash-bound Isaac evidence decides both the joint targets and the safe
# order.  This must finish before navigation; a missing/tampered evidence file
# therefore stops before either arm serial client is constructed.
source_ros2 || die "ROS2 environment setup failed before arm preparation"
run_step arm_stow "$PYTHON_BIN" "$ARM_PREPARE_SCRIPT" \
  --venue-profile "$SITE_METADATA_PATH" --result-json "$ARM_STOW_JSON" \
  || die "two-arm automatic stow failed"
ARM_STOW_FILE="$ARM_STOW_JSON" "$PYTHON_BIN" - <<'PY' || die "arm stow result invalid"
import json, os, pathlib
data=json.loads(pathlib.Path(os.environ["ARM_STOW_FILE"]).read_text(encoding="utf-8"))
if data.get("schema")!="competition_arm_stow_result/v1" or data.get("success") is not True:
    raise SystemExit("arm stow did not succeed")
if data.get("order") not in (["left","right"],["right","left"]):
    raise SystemExit("arm stow order missing")
if data.get("commands_emitted") is not True:
    raise SystemExit("live arm stow emitted no commands")
PY
COMMANDS_EMITTED=1
trace "arm_stow" "ok" "$(tr -d '\n' < "$ARM_STOW_JSON")"

source_ros1 || die "ROS1 environment setup failed"
if ! rostopic list 2>/dev/null | grep -qx /move_base/goal; then
  roslaunch turn_on_mercury_robot navigation.launch >"$LOG_DIR/navigation.log" 2>&1 & NAV_PID=$!
  ready=0
  for _ in $(seq 1 "${NAV_READY_POLLS:-30}"); do
    rostopic list 2>/dev/null | grep -qx /move_base/goal && { ready=1; break; }
    sleep 2
  done
  [[ "$ready" == 1 ]] || die "move_base unavailable"
fi
COMMANDS_EMITTED=1
run_step nav_goal3 "$PYTHON_BIN" "$RAC_SCRIPTS/send_one_goal.py" "$NAV_GOAL3" || die "navigation goal3 failed"
run_step set_head "$PYTHON_BIN" "$RAC_SCRIPTS/set_venue_head.py" || die "head command/feedback failed"

source_ros2 || die "ROS2 environment setup failed"
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
if kill -0 "$VISION_PID" 2>/dev/null; then
  trace "vision" "ready" "pid=$VISION_PID"
  export FORCE_FIXED_TARGET
  if capture_target_once "$TARGET_JSON" competition 0; then
    TARGET_SOURCE="live_competition_target"
    EXECUTION_TARGET_JSON="$TARGET_JSON"
  else
    target_detail="$(tail -n 1 "$LOG_DIR/competition_target.log" 2>/dev/null || true)"
    select_showcase_target "competition_target_failed:${target_detail:-no_detail}"
  fi
else
  tail -n 60 "$LOG_DIR/vision.log" >&2 || true
  wait "$VISION_PID" 2>/dev/null || true
  VISION_PID=""
  select_showcase_target "runtime_vision_launch_exited"
fi

if [[ "$TARGET_SOURCE" == "live_competition_target" ]]; then
  need_file "$TARGET_JSON"
  validate_target_and_write_admission || die "target/ground-plane admission failed"
  trace "admission" "ok" "$(tr -d '\n' < "$ADMISSION_JSON")"
  if [[ "$FORCE_FIXED_TARGET" == 1 ]]; then
    trace "degraded" "active" "operator asserted pen is manually placed at marker_xy_mm=[400,10]; random XY disabled"
  fi
else
  printf '{"accepted":true,"mode":"fixed_marker_showcase","sensor_target_available":false}\n' >"$ADMISSION_JSON"
fi
stop_target

mapfile -t pick_xy < <(TARGET_FILE="$EXECUTION_TARGET_JSON" "$PYTHON_BIN" - <<'PY'
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
pick_args=(--x-mm "${pick_xy[0]}" --y-mm "${pick_xy[1]}" --venue-profile "$SITE_METADATA_PATH" --result-json "$GRASP_JSON")
if [[ "$TARGET_SOURCE" == "fixed_marker_showcase" ]]; then
  pick_args+=(--showcase-target-json "$SHOWCASE_TARGET_JSON")
else
  pick_args+=(--target-json "$TARGET_JSON" --feedback-adapter "$GRASP_FEEDBACK_ADAPTER" --feedback-json "$GRASP_FEEDBACK_JSON")
fi
pick_rc=0
PICK_ATTEMPT_COUNT=1
run_step pick "$PYTHON_BIN" "$PICK_SCRIPT" "${pick_args[@]}" || pick_rc=$?
need_file "$GRASP_JSON"
if ! PICK_FILE="$GRASP_JSON" DECISION_FILE="$PICK_DECISION_JSON" SHOWCASE_ENABLED="$COMPETITION_SHOWCASE_CONTINUE" ATTEMPT_NUMBER=1 \
  "$PYTHON_BIN" - <<'PY'
import json, os, pathlib
from deyes_stereo.competition_showcase_contract import decide_pick_attempt
result=json.loads(pathlib.Path(os.environ["PICK_FILE"]).read_text(encoding="utf-8"))
decision=decide_pick_attempt(result, attempt_number=int(os.environ["ATTEMPT_NUMBER"]),
                             showcase_enabled=os.environ["SHOWCASE_ENABLED"]=="1")
pathlib.Path(os.environ["DECISION_FILE"]).write_text(json.dumps(decision,indent=2,sort_keys=True)+"\n",encoding="utf-8")
PY
then
  die "pick result classification failed"
fi
mapfile -t pick_decision < <(PICK_FILE="$GRASP_JSON" DECISION_FILE="$PICK_DECISION_JSON" "$PYTHON_BIN" - <<'PY'
import json,os,pathlib
result=json.loads(pathlib.Path(os.environ["PICK_FILE"]).read_text())
decision=json.loads(pathlib.Path(os.environ["DECISION_FILE"]).read_text())
print(decision["action"])
print(1 if decision["competition_success"] else 0)
print(1 if decision["object_grasp_verified"] else 0)
print(decision.get("degraded_reason") or "")
print(1 if result.get("motion_completed") is True else 0)
print(1 if result.get("transport_pose_reached") is True else 0)
PY
)
[[ "${#pick_decision[@]}" == 6 ]] || die "pick decision output malformed"
[[ "${pick_decision[0]}" != stop ]] || die "pick hard failure:rc=$pick_rc:${pick_decision[3]}"
PICK_COMPETITION_VERIFIED="${pick_decision[1]}"
OBJECT_GRASP_VERIFIED="${pick_decision[2]}"
PICK_MOTION_COMPLETED="${pick_decision[4]}"
TRANSPORT_POSE_REACHED="${pick_decision[5]}"

if [[ "${pick_decision[0]}" == retry_snapshot ]]; then
  trace "grasp_verification" "retry_requested" "attempt=1 reason=${pick_decision[3]}"
  if [[ -n "$VISION_PID" ]] && kill -0 "$VISION_PID" 2>/dev/null \
      && capture_target_once "$RETRY_TARGET_JSON" competition_retry 1; then
    need_file "$RETRY_TARGET_JSON"
    validate_target_and_write_admission "$RETRY_TARGET_JSON" "$RETRY_ADMISSION_JSON" 0 0 \
      || die "retry target/ground-plane admission failed"
    FIRST_TARGET_FILE="$TARGET_JSON" RETRY_TARGET_FILE="$RETRY_TARGET_JSON" "$PYTHON_BIN" - <<'PY' \
      || die "retry snapshot freshness/identity gate failed"
import json, os, pathlib
from deyes_stereo.competition_showcase_contract import validate_retry_snapshot
first=json.loads(pathlib.Path(os.environ["FIRST_TARGET_FILE"]).read_text(encoding="utf-8"))
retry=json.loads(pathlib.Path(os.environ["RETRY_TARGET_FILE"]).read_text(encoding="utf-8"))
valid,reason=validate_retry_snapshot(first,retry)
if not valid: raise SystemExit(reason)
PY
    mapfile -t retry_xy < <(TARGET_FILE="$RETRY_TARGET_JSON" "$PYTHON_BIN" - <<'PY'
import json,os,pathlib
d=json.loads(pathlib.Path(os.environ['TARGET_FILE']).read_text())
sdk=d.get('right_arm_sdk_target_m')
if not isinstance(sdk,list) or len(sdk)<2: raise SystemExit('retry target XY missing')
print(float(sdk[0])*1000); print(float(sdk[1])*1000)
PY
)
    [[ "${#retry_xy[@]}" == 2 ]] || die "retry target XY extraction failed"
    PICK_ATTEMPT_COUNT=2
    retry_pick_rc=0
    run_step pick_retry "$PYTHON_BIN" "$PICK_SCRIPT" \
      --x-mm "${retry_xy[0]}" --y-mm "${retry_xy[1]}" \
      --venue-profile "$SITE_METADATA_PATH" --result-json "$RETRY_GRASP_JSON" \
      --target-json "$RETRY_TARGET_JSON" --feedback-adapter "$GRASP_FEEDBACK_ADAPTER" \
      --feedback-json "$RETRY_GRASP_FEEDBACK_JSON" || retry_pick_rc=$?
    need_file "$RETRY_GRASP_JSON"
    if ! PICK_FILE="$RETRY_GRASP_JSON" DECISION_FILE="$RETRY_PICK_DECISION_JSON" \
      SHOWCASE_ENABLED="$COMPETITION_SHOWCASE_CONTINUE" ATTEMPT_NUMBER=2 "$PYTHON_BIN" - <<'PY'
import json, os, pathlib
from deyes_stereo.competition_showcase_contract import decide_pick_attempt
result=json.loads(pathlib.Path(os.environ["PICK_FILE"]).read_text(encoding="utf-8"))
decision=decide_pick_attempt(result, attempt_number=int(os.environ["ATTEMPT_NUMBER"]),
                             showcase_enabled=os.environ["SHOWCASE_ENABLED"]=="1")
pathlib.Path(os.environ["DECISION_FILE"]).write_text(json.dumps(decision,indent=2,sort_keys=True)+"\n",encoding="utf-8")
PY
    then
      die "retry pick result classification failed"
    fi
    mapfile -t pick_decision < <(PICK_FILE="$RETRY_GRASP_JSON" DECISION_FILE="$RETRY_PICK_DECISION_JSON" "$PYTHON_BIN" - <<'PY'
import json,os,pathlib
result=json.loads(pathlib.Path(os.environ["PICK_FILE"]).read_text())
decision=json.loads(pathlib.Path(os.environ["DECISION_FILE"]).read_text())
print(decision["action"])
print(1 if decision["competition_success"] else 0)
print(1 if decision["object_grasp_verified"] else 0)
print(decision.get("degraded_reason") or "")
print(1 if result.get("motion_completed") is True else 0)
print(1 if result.get("transport_pose_reached") is True else 0)
PY
)
    [[ "${#pick_decision[@]}" == 6 ]] || die "retry pick decision output malformed"
    [[ "${pick_decision[0]}" != stop ]] || die "retry pick hard failure:rc=$retry_pick_rc:${pick_decision[3]}"
    PICK_COMPETITION_VERIFIED="${pick_decision[1]}"
    OBJECT_GRASP_VERIFIED="${pick_decision[2]}"
    PICK_MOTION_COMPLETED="${pick_decision[4]}"
    TRANSPORT_POSE_REACHED="${pick_decision[5]}"
    TARGET_SOURCE="live_competition_target_retry"
    trace "grasp_retry" "completed" "attempt=2 action=${pick_decision[0]}"
  else
    pick_decision[0]=continue_showcase
    pick_decision[3]="retry_snapshot_or_reidentification_failed"
    trace "grasp_retry" "degraded" "new live target unavailable; no second motion"
  fi
fi

if [[ "${pick_decision[0]}" == continue_showcase ]]; then
  add_degraded_reason "${pick_decision[3]}"
  trace "grasp_verification" "degraded_continue" "attempts=$PICK_ATTEMPT_COUNT navigation_permitted=false showcase_continuation=true reason=${pick_decision[3]}"
else
  trace "grasp_verification" "ok" "attempts=$PICK_ATTEMPT_COUNT success=true navigation_permitted=true"
fi
stop_vision

source_ros1 || die "ROS1 environment restore failed before goal4"
run_step nav_goal4 "$PYTHON_BIN" "$RAC_SCRIPTS/send_one_goal.py" "$NAV_GOAL4" || die "navigation goal4 failed"
GOAL4_COMPLETED=1
object_state=unverified
[[ "$OBJECT_GRASP_VERIFIED" == 1 ]] && object_state=verified
place_args=(--venue-profile "$SITE_METADATA_PATH" --object-state "$object_state" --result-json "$PLACE_JSON")
[[ "$COMPETITION_SHOWCASE_CONTINUE" == 1 ]] && place_args+=(--showcase-mode)
run_step place "$PYTHON_BIN" "$PLACE_SCRIPT" "${place_args[@]}" || die "place command/serial/feedback failed"
need_file "$PLACE_JSON"
validate_place || die "place completion validation failed"
PLACE_MOTION_COMPLETED=1
SHOWCASE_COMPLETE=1
COMPETITION_SUCCESS="$PICK_COMPETITION_VERIFIED"
write_transaction_result || die "transaction result serialization failed"
trace "transaction" "complete" "competition_success=$COMPETITION_SUCCESS showcase_complete=true"
if [[ "$COMPETITION_SUCCESS" == 1 ]]; then
  printf '[race] COMPLETE transaction=%s logs=%s\n' "$TRANSACTION_ID" "$LOG_DIR"
else
  printf '[race] SHOWCASE COMPLETE transaction=%s logs=%s\n' "$TRANSACTION_ID" "$LOG_DIR"
  printf '[race] COMPETITION SUCCESS: false (object was not verified; see %s)\n' "$TRANSACTION_RESULT_JSON"
fi
