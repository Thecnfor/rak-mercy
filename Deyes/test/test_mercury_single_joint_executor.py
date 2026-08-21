from deyes_stereo.mercury_arm_safety_contract import MercuryArmSafetyProfile

from deyes_stereo.mercury_single_joint_executor import (
    SerialOwnershipScanError, execute_single_joint_jog, find_serial_port_owners,
)


PROFILE = MercuryArmSafetyProfile(
    arm_side="left", joint_min_deg=(-90.,) * 6, joint_max_deg=(90.,) * 6,
    workspace_min_base_m=(.1, -.4, .1), workspace_max_base_m=(.8, .4, .7),
)


class FakeMercury:
    def __init__(self, *, angles=None, power=1, errors=0, arrive=True):
        self.angles = list(angles or [0.] * 6)
        self.power, self.errors, self.arrive = power, errors, arrive
        self.sent, self.stop_calls = [], 0

    def is_power_on(self): return self.power
    def get_robot_status(self): return [0]
    def get_error_information(self): return self.errors
    def get_angles(self): return list(self.angles)
    def send_angle(self, joint, target, speed):
        self.sent.append((joint, target, speed))
        if self.arrive: self.angles[joint - 1] = target
    def stop(self): self.stop_calls += 1


def test_default_dry_run_never_opens_serial_or_creates_mercury():
    result = execute_single_joint_jog(port="/dev/left_arm", profile=PROFILE, joint_index=0, delta_deg=.5)
    assert result["state"] == "dry_run_ready"
    assert result["commands_emitted"] is False


def test_live_requires_both_independent_flags_and_rejects_busy_port():
    common = dict(port="/dev/left_arm", profile=PROFILE, joint_index=0, delta_deg=.5, dry_run=False)
    assert execute_single_joint_jog(**common)["reason"] == "enable_live_execution_false"
    assert execute_single_joint_jog(**common, enable_live_execution=True)["reason"] == "operator_confirmation_missing"
    assert execute_single_joint_jog(**common, enable_live_execution=True, operator_confirmed=True, serial_owner_scan=lambda _: [777]) ["reason"] == "serial_port_owned_by_other_process"


def test_guarded_success_reads_power_status_errors_feedback_and_one_based_joint_id():
    robot = FakeMercury()
    released = []
    result = execute_single_joint_jog(port="/dev/left_arm", profile=PROFILE, joint_index=2, delta_deg=.5, dry_run=False, enable_live_execution=True, operator_confirmed=True, mercury_factory=lambda _: robot, serial_owner_scan=lambda _: [], serial_lock_acquire=lambda _: 9, serial_lock_release=released.append)
    assert result["state"] == "succeeded" and result["commands_emitted"] is True
    assert robot.sent == [(3, .5, 2)]
    assert released == [9]


def test_fault_limit_and_timeout_stop_are_fail_closed():
    fault = FakeMercury(errors=7)
    result = execute_single_joint_jog(port="/dev/left_arm", profile=PROFILE, joint_index=0, delta_deg=.5, dry_run=False, enable_live_execution=True, operator_confirmed=True, mercury_factory=lambda _: fault, serial_owner_scan=lambda _: [], serial_lock_acquire=lambda _: 1, serial_lock_release=lambda _: None)
    assert result["reason"] == "robot_error_present" and not fault.sent
    stalled = FakeMercury(arrive=False)
    ticks = iter([0., 0., 1.])
    result = execute_single_joint_jog(port="/dev/left_arm", profile=PROFILE, joint_index=0, delta_deg=.5, dry_run=False, enable_live_execution=True, operator_confirmed=True, timeout_sec=.5, mercury_factory=lambda _: stalled, serial_owner_scan=lambda _: [], serial_lock_acquire=lambda _: 1, serial_lock_release=lambda _: None, monotonic=lambda: next(ticks), sleep=lambda _: None)
    assert result["reason"] == "joint_readback_timeout_stopped"
    assert stalled.stop_calls == 1


def test_proc_fd_scan_resolves_symlink_to_real_temp_target_and_reports_owner(tmp_path, monkeypatch):
    target = tmp_path / "tty-test"
    target.write_text("not a real tty", encoding="utf-8")
    proc_root = tmp_path / "proc"
    fd_dir = proc_root / "4242" / "fd"
    fd_dir.mkdir(parents=True)
    (fd_dir / "7").symlink_to(target)
    monkeypatch.setattr("deyes_stereo.mercury_single_joint_executor.os.name", "posix")
    assert find_serial_port_owners(str(target), proc_root=str(proc_root), self_pid=9999) == [4242]


def test_proc_scan_failure_is_fail_closed_before_factory_is_called(tmp_path):
    target = tmp_path / "tty-test"
    target.write_text("x", encoding="utf-8")
    calls = []
    result = execute_single_joint_jog(
        port=str(target), profile=PROFILE, joint_index=0, delta_deg=.5, dry_run=False,
        enable_live_execution=True, operator_confirmed=True,
        serial_owner_scan=lambda _: (_ for _ in ()).throw(SerialOwnershipScanError("no proc")),
        mercury_factory=lambda _: calls.append("factory"),
    )
    assert result["reason"] == "serial_owner_scan_unreliable:SerialOwnershipScanError"
    assert calls == []
