"""Deliberately narrow Mercury X1 single-joint commissioning executor.

The only physical command this module can issue is one selected arm joint,
changed by at most one degree at vendor speed ``<= 2``.  It is intentionally
not a pick controller.  Live execution is opt-in through *two* flags and is
off by default.  No process is killed and a busy serial port is rejected.
"""

from __future__ import annotations

import os
import stat
import time
from dataclasses import asdict
from math import isfinite
from typing import Any, Callable

from .mercury_arm_safety_contract import MercuryArmSafetyProfile, build_single_joint_jog_preview


def _report(state: str, reason: str, **values: Any) -> dict[str, Any]:
    return {
        "schema": "mercury_single_joint_jog_report/v1", "state": state, "reason": reason,
        "failure_code": "" if reason == "ok" else reason, "commands_emitted": False,
        "motion_command_emitted": False, "dry_run": True, **values,
    }


class SerialOwnershipScanError(RuntimeError):
    """The process table could not be inspected completely enough to be safe."""


def _same_endpoint(target: os.stat_result, candidate: os.stat_result) -> bool:
    """Compare a device by rdev, or an ordinary inode for deterministic tests."""
    if stat.S_ISCHR(target.st_mode):
        return stat.S_ISCHR(candidate.st_mode) and target.st_rdev == candidate.st_rdev
    return target.st_dev == candidate.st_dev and target.st_ino == candidate.st_ino


def find_serial_port_owners(port: str, *, proc_root: str = "/proc", self_pid: int | None = None) -> list[int]:
    """Find every other process with an fd resolving to ``port``.

    An unreadable process/fd directory is unsafe rather than "probably free".
    Symlink resolution happens through ``os.stat(/proc/PID/fd/N)``; this covers
    aliases such as ``/dev/serial/by-id/...`` pointing at the same tty device.
    """
    if os.name == "nt":
        raise SerialOwnershipScanError("proc_serial_owner_scan_not_supported")
    try:
        target = os.stat(port)
        process_entries = list(os.scandir(proc_root))
    except OSError as exc:
        raise SerialOwnershipScanError("proc_serial_owner_scan_unavailable") from exc
    me = os.getpid() if self_pid is None else int(self_pid)
    owners: list[int] = []
    for process in process_entries:
        if not process.name.isdecimal():
            continue
        pid = int(process.name)
        if pid == me:
            continue
        fd_dir = os.path.join(process.path, "fd")
        try:
            descriptors = list(os.scandir(fd_dir))
        except FileNotFoundError:
            # The process exited between scanning /proc and its fd directory.
            continue
        except PermissionError as exc:
            raise SerialOwnershipScanError("proc_serial_owner_scan_permission_denied") from exc
        except OSError as exc:
            raise SerialOwnershipScanError("proc_serial_owner_scan_failed") from exc
        for descriptor in descriptors:
            try:
                candidate = os.stat(descriptor.path)
            except FileNotFoundError:
                continue  # fd closed while it was being inspected.
            except PermissionError as exc:
                raise SerialOwnershipScanError("proc_serial_fd_scan_permission_denied") from exc
            except OSError as exc:
                raise SerialOwnershipScanError("proc_serial_fd_scan_failed") from exc
            if _same_endpoint(target, candidate):
                owners.append(pid)
                break
    return sorted(set(owners))


def acquire_serial_port_lock(port: str) -> int:
    """Open and hold an exclusive advisory lock until ``release`` is called."""
    if os.name == "nt":
        raise RuntimeError("serial_exclusive_lock_not_supported")
    try:
        import fcntl
        descriptor = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return descriptor
    except (OSError, ImportError) as exc:
        try:
            os.close(descriptor)  # type: ignore[possibly-undefined]
        except (OSError, UnboundLocalError):
            pass
        raise RuntimeError("serial_exclusive_lock_unavailable") from exc


def release_serial_port_lock(descriptor: int) -> None:
    try:
        import fcntl
        fcntl.flock(descriptor, fcntl.LOCK_UN)
    finally:
        os.close(descriptor)


def serial_port_busy(port: str) -> bool:
    """Compatibility helper: true on owner, lock, or scan uncertainty."""
    try:
        if find_serial_port_owners(port):
            return True
        descriptor = acquire_serial_port_lock(port)
    except (RuntimeError, SerialOwnershipScanError):
        return True
    release_serial_port_lock(descriptor)
    return False


def _factory(port: str) -> Any:
    try:
        from pymycobot import Mercury
    except ImportError as exc:
        raise RuntimeError("pymycobot_mercury_not_available") from exc
    return Mercury(port)


def _finite_angles(value: Any) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise ValueError("joint_feedback_must_be_six_values")
    try:
        angles = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ValueError("joint_feedback_must_be_six_values") from exc
    if not all(isfinite(item) for item in angles):
        raise ValueError("joint_feedback_must_be_six_values")
    return angles


def _healthy(robot: Any) -> tuple[bool, str, dict[str, Any]]:
    required = ("is_power_on", "get_robot_status", "get_error_information", "get_angles", "send_angle", "stop")
    missing = [name for name in required if not callable(getattr(robot, name, None))]
    if missing:
        return False, "mercury_sdk_capability_missing:" + ",".join(missing), {}
    try:
        powered = robot.is_power_on()
        status = robot.get_robot_status()
        errors = robot.get_error_information()
    except Exception as exc:  # SDK/vendor transport errors must not be retried blindly.
        return False, "mercury_status_read_failed:" + type(exc).__name__, {}
    if powered not in (True, 1):
        return False, "arm_power_not_confirmed", {"power_state": powered, "robot_status": status, "errors": errors}
    if status is None:
        return False, "robot_status_missing", {"power_state": powered, "errors": errors}
    # Vendor SDKs commonly return 0, [] or None for no error.  Anything else
    # is surfaced rather than guessed away.
    if errors not in (None, 0, [], ()):
        return False, "robot_error_present", {"power_state": powered, "robot_status": status, "errors": errors}
    return True, "ok", {"power_state": powered, "robot_status": status, "errors": errors}


def _stop(robot: Any) -> str | None:
    try:
        robot.stop()
    except Exception as exc:
        return type(exc).__name__
    return None


def execute_single_joint_jog(
    *, port: str, profile: MercuryArmSafetyProfile, joint_index: int, delta_deg: float,
    speed_deg_s: float = 2.0, dry_run: bool = True, enable_live_execution: bool = False,
    operator_confirmed: bool = False, timeout_sec: float = 4.0, readback_tolerance_deg: float = 0.30,
    mercury_factory: Callable[[str], Any] | None = None,
    port_busy_check: Callable[[str], bool] | None = None,
    serial_owner_scan: Callable[[str], list[int]] = find_serial_port_owners,
    serial_lock_acquire: Callable[[str], int] = acquire_serial_port_lock,
    serial_lock_release: Callable[[int], None] = release_serial_port_lock,
    monotonic: Callable[[], float] = time.monotonic, sleep: Callable[[float], None] = time.sleep,
) -> dict[str, Any]:
    """Perform the guarded first-motion procedure, or a no-I/O dry-run report.

    ``enable_live_execution`` and ``operator_confirmed`` are intentionally
    separate confirmations.  This function never powers an arm on, clears an
    error, homes a joint, or makes a second movement.
    """
    if not str(port or "").strip():
        return _report("rejected", "serial_port_required", profile=asdict(profile))
    if not isfinite(float(timeout_sec)) or timeout_sec <= 0.0:
        return _report("rejected", "timeout_sec_invalid", profile=asdict(profile))
    if not isfinite(float(readback_tolerance_deg)) or not 0.0 < readback_tolerance_deg <= 1.0:
        return _report("rejected", "readback_tolerance_deg_invalid", profile=asdict(profile))
    if dry_run:
        return _report(
            "dry_run_ready", "dry_run_no_serial_connection", profile=asdict(profile),
            requested_joint_index=joint_index, requested_delta_deg=delta_deg,
            requested_speed_deg_s=speed_deg_s, enable_live_execution=enable_live_execution,
            operator_confirmed=operator_confirmed,
        )
    if enable_live_execution is not True:
        return _report("rejected", "enable_live_execution_false", profile=asdict(profile))
    if operator_confirmed is not True:
        return _report("rejected", "operator_confirmation_missing", profile=asdict(profile))
    if profile.dry_run is not True:
        # Profile stays dry-run by design.  The executor's two flags are the
        # sole reviewed transition and a modified profile is never accepted.
        return _report("rejected", "profile_dry_run_must_remain_true", profile=asdict(profile))
    # First inspect all other fds, then hold a lock, then scan again.  flock
    # alone is advisory; /proc catches pyserial/ROS bridges that do not flock.
    try:
        owners = serial_owner_scan(port)
    except Exception as exc:
        return _report("rejected", "serial_owner_scan_unreliable:" + type(exc).__name__, profile=asdict(profile), port=port)
    if owners:
        return _report("rejected", "serial_port_owned_by_other_process", profile=asdict(profile), port=port, owner_pids=owners)
    if port_busy_check is not None and port_busy_check(port):
        return _report("rejected", "serial_port_busy_or_unavailable", profile=asdict(profile), port=port)
    try:
        lock_descriptor = serial_lock_acquire(port)
    except Exception as exc:
        return _report("rejected", "serial_exclusive_lock_unavailable:" + type(exc).__name__, profile=asdict(profile), port=port)
    try:
        try:
            owners = serial_owner_scan(port)
        except Exception as exc:
            return _report("rejected", "serial_owner_scan_unreliable:" + type(exc).__name__, profile=asdict(profile), port=port)
        if owners:
            return _report("rejected", "serial_port_owned_by_other_process", profile=asdict(profile), port=port, owner_pids=owners)
        try:
            robot = (mercury_factory or _factory)(port)
        except Exception as exc:
            return _report("rejected", "serial_connection_failed:" + type(exc).__name__, profile=asdict(profile), port=port)
        healthy, health_reason, health = _healthy(robot)
        if not healthy:
            return _report("rejected", health_reason, profile=asdict(profile), port=port, **health)
        try:
            current = _finite_angles(robot.get_angles())
        except Exception as exc:
            return _report("rejected", "joint_feedback_read_failed:" + type(exc).__name__, profile=asdict(profile), port=port, **health)
        preview = build_single_joint_jog_preview(
            current_positions_deg=current, joint_index=joint_index, delta_deg=delta_deg,
            speed_deg_s=speed_deg_s, profile=profile,
        )
        if preview["state"] != "dry_run_ready":
            return _report("rejected", preview["reason"], profile=asdict(profile), port=port, current_positions_deg=current, safety_preview=preview, **health)
        target = float(preview["preview"]["positions_deg"][joint_index])
        try:
            # Pymycobot uses one-based joint IDs.  The contract deliberately uses
            # zero-based indexes to match the 6-value feedback vector.
            robot.send_angle(int(joint_index) + 1, target, int(speed_deg_s))
        except Exception as exc:
            stop_error = _stop(robot)
            return _report("locked_manual_intervention", "send_angle_failed:" + type(exc).__name__, profile=asdict(profile), port=port, current_positions_deg=current, stop_error=stop_error, **health)
        deadline = monotonic() + float(timeout_sec)
        last_feedback = current
        while monotonic() <= deadline:
            try:
                last_feedback = _finite_angles(robot.get_angles())
            except Exception as exc:
                stop_error = _stop(robot)
                return _report("locked_manual_intervention", "joint_readback_failed:" + type(exc).__name__, profile=asdict(profile), port=port, target_deg=target, stop_error=stop_error, **health)
            if abs(last_feedback[joint_index] - target) <= float(readback_tolerance_deg):
                return _report("succeeded", "ok", profile=asdict(profile), port=port, target_deg=target, readback_positions_deg=last_feedback, readback_error_deg=abs(last_feedback[joint_index] - target), commands_emitted=True, motion_command_emitted=True, dry_run=False, **health)
            sleep(0.10)
        stop_error = _stop(robot)
        return _report("locked_manual_intervention", "joint_readback_timeout_stopped" if stop_error is None else "joint_readback_timeout_stop_failed", profile=asdict(profile), port=port, target_deg=target, readback_positions_deg=last_feedback, stop_error=stop_error, **health)
    finally:
        serial_lock_release(lock_descriptor)
