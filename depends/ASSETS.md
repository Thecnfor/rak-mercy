# 离线资产清单

二进制离线资产不进入 Git。它们由本工作区归档在
`E:/a_robot/temp/rak-mercy-baseline/offline-assets/depends/`；恢复时复制回本目录并用下表
SHA-256 校验。归档目录本身也不得作为源码提交。

| 相对路径 | 用途 | SHA-256 |
| --- | --- | --- |
| `opencv-4.8.0.tar.gz` | OpenCV 4.8.0 源码包 | `CBF47ECC336D2BFF36B0DCD7D6C179A9BB59E805136AF6B9670CA944AEF889BD` |
| `opencv_contrib-4.8.0.tar.gz` | OpenCV contrib 4.8.0 源码包 | `B4AEF0F25A22EDCD7305DF830FA926CA304EA9DB65DE6CCD02F6CFA5F3357DBB` |
| `opencv-4.8.0-cuda-xavier-nx-ubuntu20.04-cuda11.4.tar.gz` | Jetson Xavier NX CUDA OpenCV 备份 | `2319EC8314D3BBDD17509B1C1AAE83F2337096CE36B30AB4D6D13F9B02211072` |
| `models/yolov5s.onnx` | YOLOv5s ONNX 模型 | `DC4CB6A204E2CE5917B896F3B2A9B4F66647F569468F5F7855FD7D87E9F3B60E` |
| `models/yolov5s.engine` | Jetson TensorRT 引擎 | `8DA5C0CB23368CD8C451C7565559F7243CD607C4D1450FEE72BDCBDCA4DD0335` |

`yolov5s.engine` 与 Jetson/TensorRT/CUDA 版本绑定；更换目标机时必须重新生成，不能只依赖校验值。
