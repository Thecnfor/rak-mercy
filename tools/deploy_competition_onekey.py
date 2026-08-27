#!/usr/bin/env python3
"""Incrementally deploy the Deyes CUDA-depth competition chain to the Jetson."""

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


def local_files() -> list[tuple[Path, PurePosixPath]]:
    mappings = [
        (ROOT / "Deyes" / "src" / "deyes_interfaces", PurePosixPath("ws/src/Deyes/src/deyes_interfaces")),
        (ROOT / "Deyes" / "src" / "deyes_capture_cpp", PurePosixPath("ws/src/Deyes/src/deyes_capture_cpp")),
        (ROOT / "Deyes" / "src" / "deyes_stereo", PurePosixPath("ws/src/Deyes/src/deyes_stereo")),
        (ROOT / "Deyes" / "src" / "deyes_bringup", PurePosixPath("ws/src/Deyes/src/deyes_bringup")),
        (ROOT / "Deyes" / "config", PurePosixPath("ws/src/Deyes/config")),
    ]
    result: list[tuple[Path, PurePosixPath]] = []
    for source, remote in mappings:
        for path in source.rglob("*"):
            if path.is_file() and not any(part in IGNORE_PARTS for part in path.parts):
                result.append((path, remote / PurePosixPath(path.relative_to(source).as_posix())))
    for name in (
        "send_one_goal.py", "set_venue_head.py", "pick_pen_degraded.py",
        "place_pen_degraded.py", "race_onekey_try.sh",
    ):
        result.append((ROOT / "scripts" / name, PurePosixPath("scripts") / name))
    for name in ("probe_opencv_cuda.sh", "install_opencv_cuda_isolated.sh"):
        result.append((ROOT / "depends" / name, PurePosixPath("assets/depends") / name))
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
            sys.stdout.write(channel.recv(65536).decode(errors="replace"))
            sys.stdout.flush()
        if channel.recv_stderr_ready():
            sys.stderr.write(channel.recv_stderr(65536).decode(errors="replace"))
            sys.stderr.flush()
        if channel.exit_status_ready() and not channel.recv_ready() and not channel.recv_stderr_ready():
            break
    code = channel.recv_exit_status()
    if code:
        raise RuntimeError(f"remote command failed ({code})")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default=os.environ.get("ROBOT_IP", "192.168.43.60"))
    parser.add_argument("--user", default=os.environ.get("ROBOT_USER", "elephant"))
    parser.add_argument("--password", default=os.environ.get("ROBOT_PASSWORD", ""))
    parser.add_argument("--remote-home", default="/home/elephant")
    parser.add_argument(
        "--opencv-assets",
        type=Path,
        default=ROOT.parent / "temp" / "rak-mercy-baseline" / "offline-assets" / "depends",
        help="external, non-Git CUDA OpenCV archive directory",
    )
    parser.add_argument("--stop-existing", action="store_true")
    parser.add_argument("--run", action="store_true", help="start the race after deploy; default is dry-run only")
    args = parser.parse_args()
    if not args.password:
        args.password = getpass.getpass("Jetson SSH password: ")
    try:
        import paramiko
    except ImportError as exc:
        raise SystemExit("Install local dependency: py -m pip install paramiko") from exc

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(args.host, username=args.user, password=args.password, timeout=10)
    sftp = ssh.open_sftp()
    base = PurePosixPath(args.remote_home) / "deyes_competition_deploy"
    files = local_files()
    _, probe_stdout, _ = ssh.exec_command(
        f"test -f {shlex.quote(args.remote_home + '/opencv-4.8.0-cuda/lib/cmake/opencv4/OpenCVConfig.cmake')} && echo ready"
    )
    if probe_stdout.read().decode().strip() != "ready":
        archive = args.opencv_assets / "opencv-4.8.0-cuda-xavier-nx-ubuntu20.04-cuda11.4.tar.gz"
        if archive.is_file():
            files.append((archive, PurePosixPath("assets/depends") / archive.name))
        else:
            print(f"WARNING: CUDA OpenCV absent remotely and archive not found: {archive}")
    uploaded = skipped = 0
    for local, relative in files:
        remote = base / relative
        mkdir_p(sftp, remote.parent)
        digest = hashlib.sha256(local.read_bytes()).hexdigest()
        if remote_sha256(ssh, str(remote)) == digest:
            skipped += 1
            continue
        sftp.put(str(local), str(remote))
        if local.suffix in {".sh", ".py"}:
            sftp.chmod(str(remote), stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR | stat.S_IRGRP | stat.S_IXGRP)
        uploaded += 1

    # Runtime assets are copied to stable paths; source stays in the isolated workspace.
    home = shlex.quote(args.remote_home)
    base_q = shlex.quote(str(base))
    stop = "pkill -INT -f '[i]mx219_stereo_capture_node|[c]uda_stereo_depth_node|[y]olo_detector_node' || true" if args.stop_existing else ":"
    command = f"""set -euo pipefail
{stop}
mkdir -p {home}/deyes_competition_ws/src {home}/deyes_competition_assets/depends {home}/scripts {home}/temp/deyes/competition
cp -a {base_q}/ws/src/Deyes {home}/deyes_competition_ws/src/
cp {base_q}/assets/depends/*.sh {home}/deyes_competition_assets/
cp {base_q}/assets/depends/*.sh {home}/deyes_competition_assets/depends/
if compgen -G {base_q}/assets/depends/'*.tar.gz' >/dev/null; then
  cp {base_q}/assets/depends/*.tar.gz {home}/deyes_competition_assets/depends/
fi
cp {base_q}/ws/src/Deyes/config/camera/venue_20260827_quick_stereo.yaml {home}/deyes_competition_assets/
cp {base_q}/ws/src/Deyes/config/stereo/competition_fixed_scene.yaml {home}/deyes_competition_assets/
cp {base_q}/scripts/* {home}/scripts/
chmod +x {home}/scripts/*.sh {home}/scripts/*.py {home}/deyes_competition_assets/*.sh
OPENCV_PREFIX={home}/opencv-4.8.0-cuda
DEYES_OPENCV_PREFIX="$OPENCV_PREFIX" {home}/deyes_competition_assets/depends/install_opencv_cuda_isolated.sh
source /opt/ros/galactic/setup.bash
export OpenCV_DIR="$OPENCV_PREFIX/lib/cmake/opencv4"
export PKG_CONFIG_PATH="$OPENCV_PREFIX/lib/pkgconfig:${{PKG_CONFIG_PATH:-}}"
export LD_LIBRARY_PATH="/opt/ros/galactic/lib:/opt/ros/galactic/lib/aarch64-linux-gnu:$OPENCV_PREFIX/lib:${{LD_LIBRARY_PATH:-}}"
cd {home}/deyes_competition_ws
colcon build --symlink-install --packages-select deyes_interfaces deyes_capture_cpp deyes_stereo deyes_bringup --cmake-args -DOpenCV_DIR="$OpenCV_DIR"
DEGRADED_DRY_RUN=1 DEYES_WS={home}/deyes_competition_ws bash {home}/scripts/race_onekey_try.sh
"""
    print(f"Incremental sync: uploaded={uploaded}, unchanged={skipped}")
    run(ssh, command)
    if args.run:
        run(ssh, f"ALLOW_DEGRADED=1 DEYES_WS={home}/deyes_competition_ws bash {home}/scripts/race_onekey_try.sh")
    else:
        print("DEPLOY_OK. Live run: set ROBOT_PASSWORD, then add --run (explicit fallback enabled by operator).")
    sftp.close()
    ssh.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
