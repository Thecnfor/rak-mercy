# Deyes 现场机依赖说明

本目录保存 `Deyes` 现场机离线依赖的安装说明与可追溯清单。二进制包不进入 Git，目标是避免仓库膨胀，同时仍可在现场从受控归档恢复 CUDA 深度链环境。

## 当前包含

- [ASSETS.md](ASSETS.md)：离线资产的相对路径、用途和 SHA-256。
- `models/README.md`：目标检测模型的生成与现场使用说明。

资产归档位置由运维流程维护；当前工作区归档在 `E:/a_robot/temp/rak-mercy-baseline/offline-assets/depends/`。恢复前必须按 `ASSETS.md` 校验，且不得把二进制包重新提交到 Git。

其中 `opencv-4.8.0.tar.gz` 与 `opencv_contrib-4.8.0.tar.gz` 两个源码包用于在 Jetson 现场机上编译一套带 CUDA 的独立 OpenCV，供 `deyes_capture_cpp/cuda_stereo_depth_node` 使用。

## 适用环境

- 目标平台：`Jetson Xavier NX`
- 架构：`aarch64`
- 系统：`Ubuntu 20.04`
- CUDA：`11.4`
- 目标 OpenCV 安装前缀：`/home/elephant/opencv-4.8.0-cuda`

如果现场机不是以上环境，尤其不是 `aarch64 + CUDA 11.4 + Xavier NX(sm_72)`，则不能直接假定本说明完全适用，必须先重新确认 `CUDA_ARCH_BIN`、系统依赖和 ROS 环境。

## 为什么要放在 depends

- 当前系统自带 OpenCV 缺少以下 CUDA 模块：
  - `cudaimgproc`
  - `cudawarping`
  - `cudafilters`
  - `cudastereo`
- `Deyes` 当前 CUDA 深度链依赖这些模块，因此仅靠系统 OpenCV 无法构建：
  - `src/deyes_capture_cpp/src/cuda_stereo_depth_node.cpp`
  - `src/deyes_capture_cpp/CMakeLists.txt`
- 现场机一旦需要重装或更换，优先使用本目录中的离线源码包恢复环境。

## 现场机系统依赖

在现场机上先安装以下系统依赖：

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  pkg-config \
  libavcodec-dev \
  libavformat-dev \
  libswscale-dev \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libjpeg-dev \
  libpng-dev \
  libtiff-dev \
  libopenexr-dev \
  libtbb-dev \
  libeigen3-dev \
  python3-dev \
  python3-numpy
```

## 用 depends 中的源码包编译 OpenCV CUDA

### 1. 复制源码包到现场机

从受控离线资产归档恢复后，至少将以下两个文件传到现场机：

- `depends/opencv-4.8.0.tar.gz`
- `depends/opencv_contrib-4.8.0.tar.gz`

### 2. 解压并配置

以下命令以工作用户 `elephant` 为例：

```bash
mkdir -p /home/elephant/third_party
cd /home/elephant/third_party

rm -rf opencv opencv_contrib opencv_build_cuda
mkdir -p opencv_build_cuda

tar -xzf /path/to/rak-mercy/depends/opencv-4.8.0.tar.gz
tar -xzf /path/to/rak-mercy/depends/opencv_contrib-4.8.0.tar.gz

mv opencv-4.8.0 opencv
mv opencv_contrib-4.8.0 opencv_contrib

cd /home/elephant/third_party/opencv_build_cuda
cmake -S /home/elephant/third_party/opencv -B . \
  -D CMAKE_BUILD_TYPE=Release \
  -D CMAKE_INSTALL_PREFIX=/home/elephant/opencv-4.8.0-cuda \
  -D OPENCV_EXTRA_MODULES_PATH=/home/elephant/third_party/opencv_contrib/modules \
  -D WITH_CUDA=ON \
  -D CUDA_ARCH_BIN=7.2 \
  -D CUDA_ARCH_PTX= \
  -D WITH_CUDNN=ON \
  -D WITH_CUBLAS=ON \
  -D ENABLE_FAST_MATH=ON \
  -D CUDA_FAST_MATH=ON \
  -D WITH_GSTREAMER=ON \
  -D WITH_V4L=ON \
  -D WITH_GTK=OFF \
  -D WITH_QT=OFF \
  -D BUILD_TESTS=OFF \
  -D BUILD_PERF_TESTS=OFF \
  -D BUILD_EXAMPLES=OFF \
  -D BUILD_DOCS=OFF \
  -D BUILD_opencv_apps=OFF \
  -D BUILD_opencv_python3=OFF \
  -D OPENCV_GENERATE_PKGCONFIG=ON \
  -D BUILD_LIST=core,imgproc,calib3d,videoio,highgui,features2d,flann,cudev,cudaarithm,cudafilters,cudaimgproc,cudawarping,cudastereo,ximgproc
```

### 3. 编译并安装

```bash
cd /home/elephant/third_party/opencv_build_cuda
cmake --build . -- -j4
cmake --install .
```

### 4. 验证 OpenCV CUDA 安装

```bash
pkg-config --modversion opencv4
ls /home/elephant/opencv-4.8.0-cuda/lib/pkgconfig/opencv4.pc
```

更关键的是确认 `CMake` 能从新前缀找到 CUDA 模块，而不是继续找到系统自带的 CPU OpenCV。

## 让 Deyes 使用新的 OpenCV

`Deyes` 重新构建前，建议显式指定新的 OpenCV 前缀：

```bash
cd /home/elephant/deyes_ws
source /opt/ros/galactic/setup.bash

export OpenCV_DIR=/home/elephant/opencv-4.8.0-cuda/lib/cmake/opencv4
export PKG_CONFIG_PATH=/home/elephant/opencv-4.8.0-cuda/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/home/elephant/opencv-4.8.0-cuda/lib:$LD_LIBRARY_PATH

colcon build --packages-select deyes_capture_cpp deyes_bringup deyes_stereo \
  --cmake-args -DOpenCV_DIR=/home/elephant/opencv-4.8.0-cuda/lib/cmake/opencv4
```

## Deyes 现场机最小验收

OpenCV CUDA 装好后，至少要重新验证以下三步：

### 1. 主链构建

```bash
cd /home/elephant/deyes_ws
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 pkg executables deyes_capture_cpp
```

应至少能看到：

- `imx219_stereo_capture_node`
- `cuda_stereo_depth_node`

### 2. C++ 主链回归

```bash
ros2 launch deyes_bringup imx219_stereo.launch.py \
  enable_monitor:=true \
  use_cpp_capture:=true
```

### 3. CUDA 深度链验证

独立 CUDA 深度节点：

```bash
ros2 launch deyes_bringup cuda_depth.launch.py
```

一体化主链：

```bash
ros2 launch deyes_bringup imx219_stereo.launch.py \
  enable_monitor:=true \
  use_cpp_capture:=true \
  enable_cuda_depth:=true
```

## 现场排障重点

- 如果 `CMake` 仍提示找不到 `cudaimgproc/cudawarping/cudafilters/cudastereo`，优先检查：
  - `OpenCV_DIR`
  - `PKG_CONFIG_PATH`
  - `LD_LIBRARY_PATH`
  - 是否误用了系统 `/usr/lib/...` 下的旧 OpenCV
- 如果 `cmake` 配置能通过，但 `make` 中途失败，优先检查：
  - 磁盘空间
  - swap
  - `-j` 并发数是否过高
- 如果 `deyes_capture_cpp` 构建通过但 launch 失败，再回头看：
  - 相机资源是否被旧视觉链占用
  - `nvargus-daemon` 状态
  - `calib_path` 是否存在

## 现场使用建议

- 比赛出发前，确认随项目交付了 `depends/ASSETS.md` 以及与其校验值匹配的受控离线资产归档。
- 现场若更换 Jetson，不要依赖外网重新拉源码，直接使用本目录的 tar 包。
- 后续如果升级 OpenCV 版本，必须同步更新：
  - 本目录中的源码包
  - 本说明中的版本号与安装前缀
