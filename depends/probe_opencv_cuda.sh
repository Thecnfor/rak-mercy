#!/usr/bin/env bash
# Verify that a private OpenCV prefix can build/run the Deyes CUDA geometry chain.
set -euo pipefail

PREFIX="${1:-${DEYES_OPENCV_PREFIX:-/home/elephant/opencv-4.8.0-cuda}}"
CONFIG="$PREFIX/lib/cmake/opencv4/OpenCVConfig.cmake"
ARCH="$(uname -m)"
EXPECTED_ARCH="${DEYES_EXPECT_ARCH:-aarch64}"
if [ "$ARCH" != "$EXPECTED_ARCH" ] && [ "${DEYES_ALLOW_NON_JETSON_PROBE:-0}" != 1 ]; then
  echo "JETSON_ARCH_MISMATCH:expected=$EXPECTED_ARCH actual=$ARCH" >&2
  exit 1
fi
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

modules_file="$PREFIX/lib/cmake/opencv4/OpenCVModules.cmake"
[ -f "$modules_file" ] || { echo "OPENCV_MODULE_MANIFEST_MISSING:$modules_file" >&2; exit 4; }
for module in opencv_core opencv_calib3d opencv_cudaimgproc opencv_cudawarping opencv_cudafilters opencv_cudastereo opencv_ximgproc; do
  grep -q "$module" "$modules_file" || { echo "OPENCV_CONFIG_MODULE_MISSING:$module" >&2; exit 5; }
done

version_tool="$PREFIX/bin/opencv_version"
if [ -x "$version_tool" ]; then
  build_info="$(LD_LIBRARY_PATH="$PREFIX/lib:${LD_LIBRARY_PATH:-}" "$version_tool" --verbose)"
  printf '%s\n' "$build_info" | grep -Eq 'NVIDIA CUDA:[[:space:]]+YES' || { echo "OPENCV_BUILT_WITHOUT_CUDA" >&2; exit 6; }
  printf '%s\n' "$build_info" | grep -Eq '(NVIDIA GPU arch|CUDA_ARCH_BIN).*(7\.2|72)' || { echo "OPENCV_CUDA_ARCH_72_MISSING" >&2; exit 7; }
else
  echo "OPENCV_VERSION_TOOL_MISSING:$version_tool" >&2
  exit 8
fi

if command -v nvcc >/dev/null 2>&1; then
  echo "CUDA_COMPILER=$(nvcc --version | tail -n 1)"
fi
echo "OPENCV_DIR=$PREFIX/lib/cmake/opencv4"
echo "HOST_ARCH=$ARCH CUDA_ARCH_BIN=7.2"
echo "OPENCV_CUDA_OK"
