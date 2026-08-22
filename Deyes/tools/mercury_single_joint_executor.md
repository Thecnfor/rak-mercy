# Mercury X1 单关节最小增量执行器

`mercury_single_joint_executor.py` 是唯一可选的实机试动模块。它不是抓取执行器、不是轨迹控制器，且默认 `dry_run=true`，不会打开串口或构造 `pymycobot.Mercury`。

## 强制门禁

实机调用必须同时满足：

- 函数参数 `dry_run=False`；
- `enable_live_execution=True`；
- `operator_confirmed=True`；
- profile 仍为 `dry_run=True`，且包含被选一侧手臂的六轴实测限位；
- 仅单侧上电、急停和 deadman 可触达、工作区清空；
- 串口未被占用。占用时报告 `serial_port_busy_or_unavailable`，不会 kill 任何进程。

执行器不会只依赖 advisory `flock`：在创建 `Mercury` 对象前，它会遍历 `/proc/*/fd`，解析每个 fd 到目标串口的 symlink/设备 inode。发现任意其他进程（包括未使用 flock 的 pyserial/ROS bridge）即返回 `serial_port_owned_by_other_process`；`/proc` 无法完整读取时返回 `serial_owner_scan_unreliable:*` 并拒绝。首次扫描通过后，它会持有排他 flock 到本次执行结束，并在取得锁后再次扫描；不会关闭、kill 或改写占用者。

连接后执行器依次读取 `is_power_on`、`get_robot_status`、`get_error_information` 与 `get_angles`。它不自动上电、不清除错误、不回零。只接受一个 0..5 关节的 `0 < |delta_deg| <= 1°` 与 `speed <= 2`；Pymycobot 调用使用一基关节号。发送后必须在超时内通过 `get_angles` 回读目标，默认容差 `0.30°`；超时或读回失败即调用 `stop()` 并锁定人工干预。

## 调用接口

此处仅展示接口，禁止在未完成现场 checklist 时执行：

```python
from deyes_stereo.mercury_single_joint_executor import execute_single_joint_jog

report = execute_single_joint_jog(
    port="/dev/left_arm", profile=measured_left_profile,
    joint_index=0, delta_deg=0.5, speed_deg_s=2.0,
    dry_run=False, enable_live_execution=True, operator_confirmed=True,
)
```

所有返回值均是结构化报告，含 `state`、`reason/failure_code`、电源/状态/错误读数、目标与回读误差。只有 `state=succeeded` 表示一次受限试动完成；任何其他状态都禁止继续自动动作，先人工确认现场安全。
