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
reuse=0
if [[ -s "$ENGINE" && -f "$MANIFEST" ]]; then
  ENGINE="$ENGINE" MANIFEST="$MANIFEST" TRT_VERSION="$trt_version" CUDA_VERSION="$cuda_version" DEVICE_ARCH="$device_arch" python3 - <<'PY' && reuse=1 || true
import hashlib,json,os,pathlib
e=pathlib.Path(os.environ['ENGINE']); m=json.loads(pathlib.Path(os.environ['MANIFEST']).read_text())
assert m['onnx_sha256']=='{ONNX_SHA256}'
assert m['engine_sha256']==hashlib.sha256(e.read_bytes()).hexdigest()
assert m['input_shape']==[1,3,416,416] and m['output_layout']=='yolov5:[1,N,5+C]' and m['precision']=='fp16'
assert m['model_id']=='{MODEL_ID}'
assert m['tensorrt_version']==os.environ['TRT_VERSION'] and m['cuda_version']==os.environ['CUDA_VERSION']
assert m['device_arch']==os.environ['DEVICE_ARCH']
PY
fi
if [[ "$reuse" != 1 ]]; then
  "$TRTEXEC" --onnx="$ONNX" --saveEngine="$ENGINE" --fp16 --skipInference --shapes=images:1x3x416x416
fi
engine_sha="$(sha256sum "$ENGINE" | awk '{{print tolower($1)}}')"
ENGINE_SHA="$engine_sha" TRT_VERSION="$trt_version" CUDA_VERSION="$cuda_version" TRTEXEC="$TRTEXEC" MANIFEST="$MANIFEST" python3 - <<'PY'
import json,os,pathlib,platform
data={{"schema_version":1,"model_id":"{MODEL_ID}","onnx_sha256":"{ONNX_SHA256}",
"engine_sha256":os.environ['ENGINE_SHA'],"precision":"fp16","tensorrt_version":os.environ['TRT_VERSION'],
"cuda_version":os.environ['CUDA_VERSION'],"device_arch":platform.machine(),"input_shape":[1,3,416,416],
"output_layout":"yolov5:[1,N,5+C]","builder":os.environ['TRTEXEC']}}
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
ldd "$CUDA_NODE" | grep -E 'libopencv_(core|cuda)' | grep -F "$OPENCV_PREFIX/" >/dev/null || {{ echo 'CUDA node is not linked to isolated OpenCV' >&2; exit 33; }}
ldd "$CUDA_NODE" | awk '/libopencv_(core|cuda)/ {{for (i=1;i<=NF;i++) if ($i=="=>") print $(i+1)}}' > {home}/temp/deyes/competition/deploy_opencv_ldd_paths.txt
if grep -E '^(/usr/lib/|/lib/)' {home}/temp/deyes/competition/deploy_opencv_ldd_paths.txt >/dev/null; then
  echo 'CUDA node leaked to system OpenCV' >&2; exit 34
fi

DRY_LOG={home}/temp/deyes/competition/deploy_vision_dry_run
mkdir -p "$DRY_LOG"
ros2 launch deyes_bringup imx219_stereo.launch.py use_cpp_capture:=true width:=640 height:=360 fps:=30 \
  swap_left_right:=true calib_path:={home}/deyes_competition_assets/venue_20260827_quick_stereo.yaml \
  enable_cuda_depth:=true cuda_depth_publish_debug_rect:=true cuda_depth_max_sync_diff_ms:=10.0 \
  enable_ground_plane:=true enable_detector:=true detector_config:={home}/deyes_competition_assets/competition_fixed_scene.yaml \
  detector_model_path:="$ENGINE" detector_expected_model_sha256:="$engine_sha" detector_input_width:=416 detector_input_height:=416 \
  detector_expected_class_count:=1 detector_expected_max_targets:=1 enable_pen_features:=true >"$DRY_LOG/vision.log" 2>&1 &
vision_pid=$!
trap 'kill -INT "$vision_pid" 2>/dev/null || true; wait "$vision_pid" 2>/dev/null || true' EXIT
sleep 5
kill -0 "$vision_pid" 2>/dev/null || {{ tail -n 80 "$DRY_LOG/vision.log"; exit 35; }}
timeout {duration} ros2 topic hz /x1/stereo/depth >"$DRY_LOG/depth_hz.log" 2>&1 & hz_pid=$!
sleep {duration}
wait "$hz_pid" || [[ "$?" == 124 ]]
timeout 10 ros2 topic echo --once /x1/stereo/pair_diagnostics >"$DRY_LOG/pair_diagnostics.txt"
DRY_LOG="$DRY_LOG" python3 - <<'PY'
import os,pathlib,re
root=pathlib.Path(os.environ['DRY_LOG'])
hz=[float(x) for x in re.findall(r'average rate:\\s*([0-9.]+)',(root/'depth_hz.log').read_text())]
if not hz or hz[-1] < 12.0: raise SystemExit('depth topic below 12Hz')
diag=(root/'pair_diagnostics.txt').read_text()
m=re.search(r'key:\\s*["\']?window_p95_skew_ms["\']?[\\s\\S]*?value:\\s*["\']?([0-9.]+)',diag)
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
        run(ssh, " ".join(env) + f" DEYES_WS={shlex.quote(args.remote_home + '/deyes_competition_ws')} bash {shlex.quote(args.remote_home + '/scripts/race_onekey_try.sh')}")
    else:
        print("DEPLOY_OK: live vision dry-run passed; no arm motion was requested.")
    sftp.close(); ssh.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
