# IMX219 Argus 外部恢复（Jetson）

`nvarguscamerasrc` 停流后，采集节点会以 **75** 退出。这个退出码表示已观察到
Argus/appsink 卡死；节点不会在同一进程内重建 GStreamer，因为该路径已出现过
`InvalidState` 和段错误。

对于该退出码，`deyes-stereo-capture.service` 采用受限的恢复链：

```text
capture watchdog exit 75 -> systemd ExecStopPost -> restart nvargus-daemon
                         -> systemd restarts only the capture node after 8 s
```

它不会 `pkill`、不会停止机械臂/导航/YOLO，也不会为参数错误、人工停止或其他退出码
重启 Argus。十分钟内最多重启 Argus 两次；连续第三次故障由 systemd 的
`StartLimitBurst=3` 留在 failed 状态，必须先检查相机线缆、供电和日志。

## 安装与启动

在已构建并已部署 Deyes 工作区的 Jetson 上执行：

```bash
cd <Deyes>/tools
sudo ./install_argus_capture_supervisor.sh
sudoedit /etc/default/deyes-stereo-capture
sudo systemctl enable --now deyes-stereo-capture.service
```

`DEYES_CALIB_PATH` 必须指向物理棋盘格标定 YAML；调试规格文件不能用于抓取。
该 service 只占用双相机。深度/点云应在另一个终端按需启动，避免再启动一份 capture：

```bash
source /opt/ros/galactic/setup.bash
source <deyes-workspace>/install/setup.bash
ros2 launch deyes_bringup cuda_depth.launch.py calib_path:=<physical-calib.yaml>
ros2 launch deyes_bringup pointcloud.launch.py calib_path:=<physical-calib.yaml>
```

检查与停止：

```bash
systemctl status deyes-stereo-capture.service
journalctl -u deyes-stereo-capture.service -u nvargus-daemon.service -b
sudo systemctl stop deyes-stereo-capture.service
```

不要在 service 已运行时再执行带默认 `use_cpp_capture:=true` 的
`imx219_stereo.launch.py`，否则会争用相机。
