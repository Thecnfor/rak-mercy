#!/usr/bin/env bash
# Restore/build the repository-pinned CUDA OpenCV without touching system OpenCV.
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PREFIX="${DEYES_OPENCV_PREFIX:-/home/elephant/opencv-4.8.0-cuda}"
WORK="${DEYES_OPENCV_BUILD_ROOT:-/home/elephant/temp/deyes/opencv-build}"
BINARY="$ROOT/opencv-4.8.0-cuda-xavier-nx-ubuntu20.04-cuda11.4.tar.gz"
SRC="$ROOT/opencv-4.8.0.tar.gz"
CONTRIB="$ROOT/opencv_contrib-4.8.0.tar.gz"

if "$ROOT/probe_opencv_cuda.sh" "$PREFIX" >/dev/null 2>&1; then
  echo "CUDA OpenCV already ready at $PREFIX"
  exit 0
fi
mkdir -p "$WORK"
if [ -f "$BINARY" ]; then
  echo "Restoring pinned binary archive into private prefix parent"
  expected=2319EC8314D3BBDD17509B1C1AAE83F2337096CE36B30AB4D6D13F9B02211072
  actual="$(sha256sum "$BINARY" | awk '{print toupper($1)}')"
  [ "$actual" = "$expected" ] || { echo "binary archive sha256 mismatch" >&2; exit 3; }
  tar -xzf "$BINARY" -C "$(dirname "$PREFIX")"
else
  [ -f "$SRC" ] && [ -f "$CONTRIB" ] || {
    echo "CUDA OpenCV absent. Restore the three archives listed in depends/ASSETS.md." >&2
    exit 4
  }
  src_dir="$WORK/opencv-4.8.0"
  contrib_dir="$WORK/opencv_contrib-4.8.0"
  [ -d "$src_dir" ] || tar -xzf "$SRC" -C "$WORK"
  [ -d "$contrib_dir" ] || tar -xzf "$CONTRIB" -C "$WORK"
  cmake -S "$src_dir" -B "$WORK/build" -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_INSTALL_PREFIX="$PREFIX" \
    -D OPENCV_EXTRA_MODULES_PATH="$contrib_dir/modules" \
    -D WITH_CUDA=ON -D CUDA_ARCH_BIN=7.2 -D CUDA_ARCH_PTX= \
    -D WITH_CUDNN=ON -D WITH_CUBLAS=ON -D ENABLE_FAST_MATH=ON -D CUDA_FAST_MATH=ON \
    -D WITH_GSTREAMER=ON -D WITH_V4L=ON -D WITH_GTK=OFF -D WITH_QT=OFF \
    -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF \
    -D BUILD_DOCS=OFF -D BUILD_opencv_apps=OFF -D BUILD_opencv_python3=OFF \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D BUILD_LIST=core,imgproc,calib3d,videoio,highgui,features2d,flann,cudev,cudaarithm,cudafilters,cudaimgproc,cudawarping,cudastereo,ximgproc
  cmake --build "$WORK/build" -- -j"${DEYES_BUILD_JOBS:-4}"
  cmake --install "$WORK/build"
fi
"$ROOT/probe_opencv_cuda.sh" "$PREFIX"
