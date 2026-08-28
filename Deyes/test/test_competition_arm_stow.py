import pytest

from deyes_stereo.competition_arm_stow import ArmStowPlan, execute_stow_plan


class FakeMercury:
    def __init__(self, initial, *, powered=True, status=None):
        self.angles = list(initial)
        self.powered = powered
        self.status = status or [0] * 6
        self.calls = []

    def get_angles(self): return self.angles
    def get_robot_status(self): return self.status
    def is_power_on(self): return int(self.powered)
    def power_on(self): self.calls.append(("power_on",)); self.powered = True
    def send_angles(self, values, speed): self.calls.append(("send_angles", tuple(values), speed)); self.angles = list(values)
    def stop(self): self.calls.append(("stop",))


def _plan():
    return ArmStowPlan(
        order=("left", "right"),
        power_on_deg={"left": (0,) * 6, "right": (0,) * 6},
        stow_deg={"left": (10,) * 6, "right": (-10,) * 6},
    )


def test_stows_in_evidence_order_one_port_at_a_time():
    robots = {"/dev/left_arm": FakeMercury((0,) * 6), "/dev/right_arm": FakeMercury((0,) * 6)}
    acquired = []
    released = []
    result = execute_stow_plan(
        _plan(), mercury_factory=lambda port: robots[port], serial_owner_scan=lambda _: [],
        lock_acquire=lambda side: acquired.append(side) or side, lock_release=released.append,
    )
    assert result["success"] and result["order"] == ["left", "right"]
    assert acquired == released == ["left", "right"]
    assert robots["/dev/left_arm"].calls == [("send_angles", (10,) * 6, 5)]
    assert robots["/dev/right_arm"].calls == [("send_angles", (-10,) * 6, 5)]


def test_unknown_start_or_serial_owner_fails_before_motion():
    wrong = FakeMercury((2,) * 6)
    with pytest.raises(RuntimeError, match="initial_pose_mismatch"):
        execute_stow_plan(
            _plan(), mercury_factory=lambda _: wrong, serial_owner_scan=lambda _: [],
            lock_acquire=lambda side: side, lock_release=lambda _: None,
        )
    assert wrong.calls == []
    with pytest.raises(RuntimeError, match="serial_port_owned"):
        execute_stow_plan(
            _plan(), mercury_factory=lambda _: wrong, serial_owner_scan=lambda _: [123],
            lock_acquire=lambda side: side, lock_release=lambda _: None,
        )


def test_fault_after_command_calls_stop_and_does_not_continue():
    fault = FakeMercury((0,) * 6, status=[1, 0, 0, 0, 0, 0])
    with pytest.raises(RuntimeError, match="robot_status_not_ok"):
        execute_stow_plan(
            _plan(), mercury_factory=lambda _: fault, serial_owner_scan=lambda _: [],
            lock_acquire=lambda side: side, lock_release=lambda _: None,
        )
    assert fault.calls == []
