#!/usr/bin/env bash
# Verify that a private OpenCV prefix can build/run the Deyes CUDA geometry chain.
set -euo pipefail

PREFIX="${1:-${DEYES_OPENCV_PREFIX:-/home/elephant/opencv-4.8.0-cuda}}"
CONFIG="$PREFIX/lib/cmake/opencv4/OpenCVConfig.cmake"
if [ ! -f "$CONFIG" ]; then
  echo "OPENCV_CUDA_MISSING:$CONFIG" >&2
  exit 2
fi

required=(libopencv_core libopencv_imgproc libopencv_calib3d libopencv_cudaimgproc libopencv_cudawarping libopencv_cudafilters libopencv_cudastereo libopencv_ximgproc)
for name in "${required[@]}"; do
  compgen -G "$PREFIX/lib/${name}.so*" >/dev/null || {
    echo "OPENCV_CUDA_MODULE_MISSING:$name" >&2
    exit 3
  }
done

if command -v nvcc >/dev/null 2>&1; then
  echo "CUDA_COMPILER=$(nvcc --version | tail -n 1)"
fi
echo "OPENCV_DIR=$PREFIX/lib/cmake/opencv4"
echo "OPENCV_CUDA_OK"
