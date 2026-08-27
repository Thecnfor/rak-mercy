#!/usr/bin/env python3
"""Incrementally deploy and validate the fail-closed Deyes competition chain."""

from __future__ import annotations

import argparse
import getpass
import hashlib
import os
from pathlib import Path, PurePosixPath
import shlex
import stat
import sys


ROOT = Path(__file__).resolve().parents[1]
IGNORE_PARTS = {".git", "__pycache__", ".pytest_cache", "build", "install", "log"}
ONNX_RELATIVE = Path("Deyes/models/pen/pen_student_01875_416_v1.onnx")
ONNX_SHA256 = "8916cbf25949d6e8b03c01e6ca1c7871aeac0ad105c931f1dd9881cb5d5a4c4e"
MODEL_ID = "pen-yolov5-student-01875-416-v1"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_opencv_ldd_paths(paths: list[str], prefix: str) -> list[str]:
    """Require every resolved OpenCV dependency to live under the isolated prefix."""
    resolved_prefix = os.path.realpath(prefix)
    resolved = [os.path.realpath(path.strip()) for path in paths if path.strip()]
    if not resolved:
        raise ValueError("no OpenCV libraries were reported by ldd")
    for path in resolved:
        if os.path.commonpath((resolved_prefix, path)) != resolved_prefix:
            raise ValueError(f"OpenCV library outside isolated OpenCV prefix: {path}")
    return resolved


def local_files() -> list[tuple[Path, PurePosixPath]]:
    mappings = [
        (ROOT / "Deyes" / "src" / package, PurePosixPath("ws/src/Deyes/src") / package)
        for package in ("deyes_interfaces", "deyes_capture_cpp", "deyes_stereo", "deyes_bringup")
    ] + [
        (ROOT / "Deyes" / "config", PurePosixPath("ws/src/Deyes/config")),
        (ROOT / "scripts", PurePosixPath("scripts")),
    ]
    result: list[tuple[Path, PurePosixPath]] = []
    for source, remote in mappings:
        if not source.is_dir():
            raise FileNotFoundError(source)
        for path in source.rglob("*"):
            if path.is_file() and not any(part in IGNORE_PARTS for part in path.parts):
                result.append((path, remote / PurePosixPath(path.relative_to(source).as_posix())))
    for name in ("probe_opencv_cuda.sh", "install_opencv_cuda_isolated.sh"):
        result.append((ROOT / "depends" / name, PurePosixPath("assets/depends") / name))
    result.append((ROOT / "tools" / "competition_target_snapshot_adapter.py",
                   PurePosixPath("scripts/competition_target_snapshot_adapter.py")))
    onnx = ROOT / ONNX_RELATIVE
    if sha256_file(onnx) != ONNX_SHA256:
        raise RuntimeError(f"repository ONNX SHA256 mismatch: {onnx}")
    result.append((onnx, PurePosixPath("models/pen") / onnx.name))
    return result


def mkdir_p(sftp, path: PurePosixPath) -> None:
    current = PurePosixPath("/")
    for part in path.parts[1:]:
        current /= part
        try:
            sftp.stat(str(current))
        except OSError:
            sftp.mkdir(str(current))


def remote_sha256(ssh, path: str) -> str:
    _, stdout, _ = ssh.exec_command(f"sha256sum {shlex.quote(path)} 2>/dev/null || true")
    return stdout.read().decode().strip().split(" ", 1)[0]


def run(ssh, command: str) -> None:
    channel = ssh.get_transport().open_session()
    channel.get_pty()
    channel.exec_command(command)
    while True:
        if channel.recv_ready():
            sys.stdout.write(channel.recv(65536).decode(errors="replace")); sys.stdout.flush()
        if channel.recv_stderr_ready():
            sys.stderr.write(channel.recv_stderr(65536).decode(errors="replace")); sys.stderr.flush()
        if channel.exit_status_ready() and not channel.recv_ready() and not channel.recv_stderr_ready():
            break
    code = channel.recv_exit_status()
    if code:
        raise RuntimeError(f"remote command failed ({code})")


def remote_deploy_command(args: argparse.Namespace, base: PurePosixPath) -> str:
    home = shlex.quote(args.remote_home)
    base_q = shlex.quote(str(base))
    stop = (
        "pkill -INT -f '[i]mx219_stereo_capture_node|[c]uda_stereo_depth_node|"
        "[g]round_plane_node|[y]olo_detector_node|[p]en_feature_node|[c]ompetition_pick_target|"
        "[r]os2 launch deyes_bringup imx219_stereo.launch.py' || true"
        if args.stop_existing else ":"
    )
    duration = max(1, args.vision_dry_run_seconds)
    return f"""set -Eeuo pipefail
{stop}
mkdir -p {home}/deyes_competition_ws/src/Deyes/src {home}/deyes_competition_ws/src/Deyes/config \
  {home}/deyes_competition_assets/depends {home}/scripts {home}/temp/deyes/competition \
  {home}/temp/deyes/models/pen
for package in deyes_interfaces deyes_capture_cpp deyes_stereo deyes_bringup; do
  mkdir -p {home}/deyes_competition_ws/src/Deyes/src/$package
  cp -a {base_q}/ws/src/Deyes/src/$package/. {home}/deyes_competition_ws/src/Deyes/src/$package/
done
cp -a {base_q}/ws/src/Deyes/config/. {home}/deyes_competition_ws/src/Deyes/config/
cp -a {base_q}/scripts/. {home}/scripts/
cp {base_q}/assets/depends/*.sh {home}/deyes_competition_assets/depends/
cp {base_q}/assets/depends/*.sh {home}/deyes_competition_assets/
cp {base_q}/models/pen/pen_student_01875_416_v1.onnx {home}/temp/deyes/models/pen/
if compgen -G {base_q}/assets/depends/'*.tar.gz' >/dev/null; then
  cp {base_q}/assets/depends/*.tar.gz {home}/deyes_competition_assets/depends/
fi
cp {home}/deyes_competition_ws/src/Deyes/config/camera/venue_20260827_quick_stereo.yaml {home}/deyes_competition_assets/
cp {home}/deyes_competition_ws/src/Deyes/config/camera/venue_20260827_touch_projector.yaml {home}/deyes_competition_assets/
cp {home}/deyes_competition_ws/src/Deyes/config/stereo/competition_fixed_scene.yaml {home}/deyes_competition_assets/
site_file="$(find {home}/deyes_competition_ws/src/Deyes/config -type f -name competition_venue_65cm.yaml | head -n 1)"
[[ -f "$site_file" ]] || {{ echo 'missing competition_venue_65cm.yaml after config upload' >&2; exit 30; }}
cp "$site_file" {home}/deyes_competition_assets/competition_venue_65cm.yaml
chmod +x {home}/scripts/*.sh {home}/scripts/*.py {home}/deyes_competition_assets/*.sh

OPENCV_PREFIX={home}/opencv-4.8.0-cuda
DEYES_OPENCV_PREFIX="$OPENCV_PREFIX" {home}/deyes_competition_assets/depends/install_opencv_cuda_isolated.sh
{home}/deyes_competition_assets/depends/probe_opencv_cuda.sh "$OPENCV_PREFIX"

ONNX={home}/temp/deyes/models/pen/pen_student_01875_416_v1.onnx
ENGINE={home}/temp/deyes/models/pen/{MODEL_ID}.fp16.engine
MANIFEST="$ENGINE.manifest.json"
RUNTIME_BINDINGS="$ENGINE.runtime_bindings.json"
actual_onnx="$(sha256sum "$ONNX" | awk '{{print tolower($1)}}')"
[[ "$actual_onnx" == "{ONNX_SHA256}" ]] || {{ echo 'ONNX SHA256 mismatch' >&2; exit 31; }}
if command -v trtexec >/dev/null 2>&1; then TRTEXEC="$(command -v trtexec)";
elif [[ -x /usr/src/tensorrt/bin/trtexec ]]; then TRTEXEC=/usr/src/tensorrt/bin/trtexec;
else echo 'trtexec not found' >&2; exit 32; fi
trt_version="$(dpkg-query -W -f='${{Version}}' libnvinfer8 2>/dev/null || "$TRTEXEC" --version 2>&1 | tail -n 1)"
cuda_version="$(nvcc --version 2>/dev/null | tail -n 1 || cat /usr/local/cuda/version.txt 2>/dev/null || echo unknown)"
device_arch="$(uname -m)"
[[ -n "$trt_version" && "$trt_version" != unknown ]] || {{ echo 'unable to determine TensorRT version' >&2; exit 36; }}
[[ -n "$cuda_version" && "$cuda_version" != unknown ]] || {{ echo 'unable to determine CUDA version' >&2; exit 37; }}
[[ "$device_arch" == aarch64 ]] || {{ echo "unsupported deployment architecture: $device_arch" >&2; exit 38; }}
inspect_engine() {{
  ENGINE="$ENGINE" RUNTIME_BINDINGS="$RUNTIME_BINDINGS" python3 - <<'PY'
import json, os, pathlib
import tensorrt as trt

def dtype_name(value):
    text = str(value).strip().lower()
    aliases = {{"float": "float32", "float32": "float32", "fp32": "float32",
               "half": "float16", "float16": "float16", "fp16": "float16",
               "int32": "int32", "int8": "int8", "bool": "bool"}}
    for suffix, normalized in aliases.items():
        if text == suffix or text.endswith("." + suffix):
            return normalized
    return text

logger = trt.Logger(trt.Logger.WARNING)
runtime = trt.Runtime(logger)
engine = runtime.deserialize_cuda_engine(pathlib.Path(os.environ["ENGINE"]).read_bytes())
if engine is None:
    raise SystemExit("TensorRT could not deserialize engine")
if bool(getattr(engine, "has_implicit_batch_dimension", False)):
    raise SystemExit("TensorRT engine must use explicit batch")
bindings = []
if hasattr(engine, "num_io_tensors") and hasattr(engine, "get_tensor_name"):
    for index in range(int(engine.num_io_tensors)):
        name = str(engine.get_tensor_name(index))
        is_input = engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT
        bindings.append({{"index": index, "name": name, "io": "input" if is_input else "output",
                         "shape": [int(value) for value in engine.get_tensor_shape(name)],
                         "dtype": dtype_name(engine.get_tensor_dtype(name))}})
elif hasattr(engine, "num_bindings"):
    for index in range(int(engine.num_bindings)):
        is_input = bool(engine.binding_is_input(index))
        bindings.append({{"index": index, "name": str(engine.get_binding_name(index)),
                         "io": "input" if is_input else "output",
                         "shape": [int(value) for value in engine.get_binding_shape(index)],
                         "dtype": dtype_name(engine.get_binding_dtype(index))}})
else:
    raise SystemExit("unsupported TensorRT engine inspection API")
facts = {{"schema": "deyes_tensorrt_runtime_bindings/v1",
         "tensorrt_runtime_version": str(getattr(trt, "__version__", "unknown")),
         "explicit_batch": True, "bindings": bindings}}
pathlib.Path(os.environ["RUNTIME_BINDINGS"]).write_text(
    json.dumps(facts, indent=2, sort_keys=True) + "\\n", encoding="utf-8")
PY
}}
reuse=0
if [[ -s "$ENGINE" && -f "$MANIFEST" ]] && inspect_engine; then
  ENGINE="$ENGINE" MANIFEST="$MANIFEST" RUNTIME_BINDINGS="$RUNTIME_BINDINGS" TRT_VERSION="$trt_version" CUDA_VERSION="$cuda_version" DEVICE_ARCH="$device_arch" python3 - <<'PY' && reuse=1 || true
import hashlib,json,os,pathlib
e=pathlib.Path(os.environ['ENGINE']); m=json.loads(pathlib.Path(os.environ['MANIFEST']).read_text())
runtime=json.loads(pathlib.Path(os.environ['RUNTIME_BINDINGS']).read_text())
assert m['onnx_sha256']=='{ONNX_SHA256}'
assert m['engine_sha256']==hashlib.sha256(e.read_bytes()).hexdigest()
assert m['input_shape']==[1,3,416,416] and m['output_layout']=='yolov5:[1,N,5+C]' and m['precision']=='fp16'
assert m['model_id']=='{MODEL_ID}'
assert m['tensorrt_version']==os.environ['TRT_VERSION'] and m['cuda_version']==os.environ['CUDA_VERSION']
assert m['device_arch']==os.environ['DEVICE_ARCH']
assert m['tensorrt_runtime_version']==runtime['tensorrt_runtime_version']
assert m['bindings']==runtime['bindings']
bindings=runtime['bindings']; inputs=[b for b in bindings if b.get('io')=='input']; outputs=[b for b in bindings if b.get('io')=='output']
assert len(bindings)==2 and len(inputs)==1 and len(outputs)==1
assert inputs[0]['shape']==[1,3,416,416] and inputs[0]['dtype']=='float32'
assert len(outputs[0]['shape'])==3 and outputs[0]['shape'][0]==1 and outputs[0]['shape'][1]>0 and outputs[0]['shape'][2]==6
assert outputs[0]['dtype'] in ('float16','float32')
PY
fi
if [[ "$reuse" != 1 ]]; then
  "$TRTEXEC" --onnx="$ONNX" --saveEngine="$ENGINE" --fp16 --skipInference --shapes=images:1x3x416x416
fi
inspect_engine || {{ echo 'unable to enumerate TensorRT engine bindings' >&2; exit 39; }}
engine_sha="$(sha256sum "$ENGINE" | awk '{{print tolower($1)}}')"
ENGINE_SHA="$engine_sha" TRT_VERSION="$trt_version" CUDA_VERSION="$cuda_version" TRTEXEC="$TRTEXEC" MANIFEST="$MANIFEST" RUNTIME_BINDINGS="$RUNTIME_BINDINGS" python3 - <<'PY'
import json,os,pathlib,platform
runtime=json.loads(pathlib.Path(os.environ['RUNTIME_BINDINGS']).read_text())
bindings=runtime['bindings']; inputs=[b for b in bindings if b.get('io')=='input']; outputs=[b for b in bindings if b.get('io')=='output']
if len(bindings)!=2 or len(inputs)!=1 or len(outputs)!=1: raise SystemExit('engine ABI requires exactly one input and one output binding')
if inputs[0].get('shape')!=[1,3,416,416] or inputs[0].get('dtype')!='float32': raise SystemExit('engine input binding ABI mismatch')
shape=outputs[0].get('shape')
if not isinstance(shape,list) or len(shape)!=3 or shape[0]!=1 or shape[1]<=0 or shape[2]!=6: raise SystemExit('engine YOLOv5 output binding ABI mismatch')
if outputs[0].get('dtype') not in ('float16','float32'): raise SystemExit('engine output binding dtype mismatch')
data={{"schema_version":2,"model_id":"{MODEL_ID}","onnx_sha256":"{ONNX_SHA256}",
"engine_sha256":os.environ['ENGINE_SHA'],"precision":"fp16","tensorrt_version":os.environ['TRT_VERSION'],
"cuda_version":os.environ['CUDA_VERSION'],"device_arch":platform.machine(),"input_shape":[1,3,416,416],
"output_layout":"yolov5:[1,N,5+C]","builder":os.environ['TRTEXEC'],
"tensorrt_runtime_version":runtime['tensorrt_runtime_version'],"bindings":bindings}}
pathlib.Path(os.environ['MANIFEST']).write_text(json.dumps(data,indent=2,sort_keys=True)+'\\n')
PY

source /opt/ros/galactic/setup.bash
export OpenCV_DIR="$OPENCV_PREFIX/lib/cmake/opencv4"
export PKG_CONFIG_PATH="$OPENCV_PREFIX/lib/pkgconfig:${{PKG_CONFIG_PATH:-}}"
export LD_LIBRARY_PATH="/opt/ros/galactic/lib:/opt/ros/galactic/lib/aarch64-linux-gnu:$OPENCV_PREFIX/lib:${{LD_LIBRARY_PATH:-}}"
cd {home}/deyes_competition_ws
colcon build --symlink-install --packages-select deyes_interfaces deyes_capture_cpp deyes_stereo deyes_bringup --cmake-args -DOpenCV_DIR="$OpenCV_DIR"
source install/setup.bash
CUDA_NODE="$(ros2 pkg prefix deyes_capture_cpp)/lib/deyes_capture_cpp/cuda_stereo_depth_node"
ldd "$CUDA_NODE" | tee {home}/temp/deyes/competition/deploy_ldd.log
ldd "$CUDA_NODE" | awk '/libopencv_/ {{for (i=1;i<=NF;i++) if ($i=="=>") print $(i+1)}}' > {home}/temp/deyes/competition/deploy_opencv_ldd_paths.txt
OPENCV_PREFIX="$OPENCV_PREFIX" OPENCV_PATHS={home}/temp/deyes/competition/deploy_opencv_ldd_paths.txt python3 - <<'PY'
import os, pathlib
prefix=os.path.realpath(os.environ['OPENCV_PREFIX'])
paths=[line.strip() for line in pathlib.Path(os.environ['OPENCV_PATHS']).read_text().splitlines() if line.strip()]
if not paths: raise SystemExit('CUDA node has no resolved OpenCV dependencies')
for reported in paths:
    resolved=os.path.realpath(reported)
    if os.path.commonpath((prefix,resolved)) != prefix:
        raise SystemExit('CUDA node leaked outside isolated OpenCV: '+resolved)
PY

DRY_LOG={home}/temp/deyes/competition/deploy_vision_dry_run
mkdir -p "$DRY_LOG"
ros2 launch deyes_bringup imx219_stereo.launch.py use_cpp_capture:=true width:=640 height:=360 fps:=30 \
  swap_left_right:=true calib_path:={home}/deyes_competition_assets/venue_20260827_quick_stereo.yaml \
  enable_cuda_depth:=true cuda_depth_publish_debug_rect:=true cuda_depth_max_sync_diff_ms:=10.0 \
  enable_ground_plane:=true enable_detector:=true detector_config:={home}/deyes_competition_assets/competition_fixed_scene.yaml \
  detector_model_path:="$ENGINE" detector_expected_model_sha256:="$engine_sha" detector_input_width:=416 detector_input_height:=416 \
  detector_model_id:={MODEL_ID} detector_expected_class_count:=1 detector_expected_max_targets:=1 \
  enable_pen_features:=true >"$DRY_LOG/vision.log" 2>&1 &
vision_pid=$!
trap 'kill -INT "$vision_pid" 2>/dev/null || true; wait "$vision_pid" 2>/dev/null || true' EXIT
sleep 5
kill -0 "$vision_pid" 2>/dev/null || {{ tail -n 80 "$DRY_LOG/vision.log"; exit 35; }}
timeout {duration} ros2 topic hz /x1/stereo/depth >"$DRY_LOG/depth_hz.log" 2>&1 & hz_pid=$!
DRY_LOG="$DRY_LOG" ENGINE="$ENGINE" ENGINE_SHA="$engine_sha" DEADLINE_SECONDS={duration} python3 - <<'PY' & contract_pid=$!
import json, math, os, pathlib, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

topics = {{
    "camera_info": "/x1/stereo/left/camera_info_rect",
    "boxes": "/x1/detection/boxes",
    "boxes_status": "/x1/detection/boxes_status",
    "ground_plane": "/x1/ground/plane",
    "ground_plane_status": "/x1/ground/plane_status",
    "pen_features": "/x1/detection/pen_features",
    "pen_features_status": "/x1/detection/pen_features_status",
}}
received = {{}}
rclpy.init()
node = Node("competition_deploy_vision_contract_probe")
node.create_subscription(CameraInfo, topics["camera_info"], lambda msg: received.setdefault("camera_info", msg), qos_profile_sensor_data)
for key in ("boxes", "boxes_status", "ground_plane", "ground_plane_status", "pen_features", "pen_features_status"):
    def callback(msg, name=key):
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError) as exc:
            received[name] = {{"_parse_error": str(exc), "_raw": msg.data}}
            return
        if name == "boxes_status" and (payload.get("backend_ready") is not True or payload.get("level") == "error"):
            return
        if name == "ground_plane_status" and payload.get("level") not in ("ok", "degraded"):
            return
        received[name] = payload
    node.create_subscription(String, topics[key], callback, qos_profile_sensor_data)
deadline = time.monotonic() + max(1.0, float(os.environ["DEADLINE_SECONDS"]))
while len(received) < len(topics) and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=min(.2, max(0.0, deadline-time.monotonic())))
node.destroy_node(); rclpy.shutdown()
missing = sorted(set(topics) - set(received))
if missing:
    raise SystemExit("vision contract topics missing: " + ",".join(topics[name] for name in missing))

info = received["camera_info"]
projection = [float(value) for value in info.p]
if int(info.width) != 640 or int(info.height) != 360:
    raise SystemExit("rectified CameraInfo dimensions mismatch")
if len(projection) != 12 or not all(math.isfinite(value) for value in projection):
    raise SystemExit("rectified CameraInfo projection is malformed")
if projection[0] <= 0 or projection[5] <= 0 or abs(projection[10]-1.0) > 1e-6:
    raise SystemExit("rectified CameraInfo is not calibrated")

boxes = received["boxes"]
if boxes.get("backend") != "tensorrt" or boxes.get("model_id") != "{MODEL_ID}":
    raise SystemExit("YOLO boxes do not identify the pinned TensorRT model")
if boxes.get("model_path") != os.environ["ENGINE"] or boxes.get("model_sha256") != os.environ["ENGINE_SHA"]:
    raise SystemExit("YOLO boxes engine/model SHA chain mismatch")
if boxes.get("expected_class_count") != 1 or boxes.get("expected_max_targets") != 1:
    raise SystemExit("YOLO boxes class/target contract mismatch")
boxes_status = received["boxes_status"]
if boxes_status.get("backend") != "tensorrt" or boxes_status.get("backend_ready") is not True or boxes_status.get("level") == "error":
    raise SystemExit("YOLO TensorRT backend status is not ready")

plane = received["ground_plane"]
if plane.get("coordinate_contract") != "dynamic_table_plane_camera_relative_only":
    raise SystemExit("ground plane coordinate contract mismatch")
for key, length in (("plane_normal", 3), ("plane_center_camera_m", 3)):
    values = plane.get(key)
    if not isinstance(values, list) or len(values) != length or not all(math.isfinite(float(value)) for value in values):
        raise SystemExit("ground plane output malformed: " + key)
plane_status = received["ground_plane_status"]
if plane_status.get("level") not in ("ok", "degraded"):
    raise SystemExit("ground plane status is not an emitted fit")

features = received["pen_features"]
if features.get("image_width") != 640 or features.get("image_height") != 360 or not isinstance(features.get("features"), list):
    raise SystemExit("pen_feature output contract mismatch")
feature_status = received["pen_features_status"]
if feature_status.get("level") not in ("ok", "warn"):
    raise SystemExit("pen_feature status is invalid")

evidence = {{
    "schema": "deyes_deploy_vision_contract/v1",
    "received_topics": topics,
    "rectified_camera_info": {{"frame_id": info.header.frame_id, "width": int(info.width), "height": int(info.height), "p": projection}},
    "yolo_engine_chain": {{"backend": boxes["backend"], "model_id": boxes["model_id"], "model_path": boxes["model_path"], "model_sha256": boxes["model_sha256"]}},
    "ground_plane": plane,
    "ground_plane_status": plane_status,
    "pen_features": features,
    "pen_features_status": feature_status,
}}
pathlib.Path(os.environ["DRY_LOG"], "vision_contract.json").write_text(json.dumps(evidence, indent=2, sort_keys=True)+"\\n", encoding="utf-8")
PY
contract_rc=0; wait "$contract_pid" || contract_rc=$?
hz_rc=0; wait "$hz_pid" || hz_rc=$?
[[ "$contract_rc" == 0 ]] || {{ tail -n 80 "$DRY_LOG/vision.log"; exit "$contract_rc"; }}
[[ "$hz_rc" == 0 || "$hz_rc" == 124 ]] || exit "$hz_rc"
timeout 10 ros2 topic echo --once /x1/stereo/pair_diagnostics >"$DRY_LOG/pair_diagnostics.txt"
DRY_LOG="$DRY_LOG" python3 - <<'PY'
import os,pathlib,re
root=pathlib.Path(os.environ['DRY_LOG'])
hz=[float(x) for x in re.findall(r'average rate:\\s*([0-9.]+)',(root/'depth_hz.log').read_text())]
if not hz or hz[-1] < 12.0: raise SystemExit('depth topic below 12Hz')
diag=(root/'pair_diagnostics.txt').read_text()
m=re.search(r'key:\\s*[\\x22\\x27]?window_p95_skew_ms[\\x22\\x27]?[\\s\\S]*?value:\\s*[\\x22\\x27]?([0-9.]+)',diag)
if not m or float(m.group(1)) > 10.0: raise SystemExit('pair skew missing or above 10ms')
(root/'acceptance.txt').write_text(f'depth_hz={{hz[-1]:.3f}}\\npair_p95_skew_ms={{float(m.group(1)):.3f}}\\n')
PY
kill -INT "$vision_pid"; wait "$vision_pid" || true; trap - EXIT
echo DEPLOY_DRY_RUN_OK
"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default=os.environ.get("ROBOT_IP", "192.168.43.60"))
    parser.add_argument("--user", default=os.environ.get("ROBOT_USER", "elephant"))
    parser.add_argument("--password", default=os.environ.get("ROBOT_PASSWORD", ""))
    parser.add_argument("--remote-home", default="/home/elephant")
    parser.add_argument("--opencv-assets", type=Path, default=ROOT.parent / "temp" / "rak-mercy-baseline" / "offline-assets" / "depends")
    parser.add_argument("--stop-existing", action="store_true")
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--allow-fixed-xy-fallback", action="store_true")
    parser.add_argument("--force-fixed-target", action="store_true")
    parser.add_argument("--strict-result-gates", action="store_true")
    parser.add_argument("--vision-dry-run-seconds", type=int, default=int(os.environ.get("DEYES_DEPLOY_VISION_SECONDS", "30")))
    args = parser.parse_args()
    if args.force_fixed_target != args.allow_fixed_xy_fallback:
        parser.error("--allow-fixed-xy-fallback and --force-fixed-target must be used together")
    if not args.password:
        args.password = getpass.getpass("Jetson SSH password: ")
    try:
        import paramiko
    except ImportError as exc:
        raise SystemExit("Install local dependency: py -m pip install paramiko") from exc

    ssh = paramiko.SSHClient(); ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(args.host, username=args.user, password=args.password, timeout=10)
    sftp = ssh.open_sftp(); base = PurePosixPath(args.remote_home) / "deyes_competition_deploy"
    files = local_files()
    _, probe_stdout, _ = ssh.exec_command(f"test -f {shlex.quote(args.remote_home + '/opencv-4.8.0-cuda/lib/cmake/opencv4/OpenCVConfig.cmake')} && echo ready")
    if probe_stdout.read().decode().strip() != "ready":
        archive = args.opencv_assets / "opencv-4.8.0-cuda-xavier-nx-ubuntu20.04-cuda11.4.tar.gz"
        if archive.is_file(): files.append((archive, PurePosixPath("assets/depends") / archive.name))
        else: print(f"WARNING: isolated CUDA OpenCV archive not found: {archive}")
    uploaded = skipped = 0
    for local, relative in files:
        remote = base / relative; mkdir_p(sftp, remote.parent)
        digest = sha256_file(local)
        if remote_sha256(ssh, str(remote)) == digest: skipped += 1; continue
        sftp.put(str(local), str(remote))
        if local.suffix in {".sh", ".py"}:
            sftp.chmod(str(remote), stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR | stat.S_IRGRP | stat.S_IXGRP)
        uploaded += 1
    print(f"Incremental sync: uploaded={uploaded}, unchanged={skipped}")
    run(ssh, remote_deploy_command(args, base))
    if args.run:
        env = ["FIXED_TABLE_HEIGHT_MM=650", "ALLOW_BBOX_CENTER=0"]
        env.append(f"ALLOW_FIXED_XY_FALLBACK={int(args.allow_fixed_xy_fallback)}")
        env.append(f"FORCE_FIXED_TARGET={int(args.force_fixed_target)}")
        env.append(f"COMPETITION_SHOWCASE_CONTINUE={int(not args.strict_result_gates)}")
        run(ssh, " ".join(env) + f" DEYES_WS={shlex.quote(args.remote_home + '/deyes_competition_ws')} bash {shlex.quote(args.remote_home + '/scripts/race_onekey_try.sh')}")
    else:
        print("DEPLOY_OK: live vision dry-run passed; no arm motion was requested.")
    sftp.close(); ssh.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
