from deyes_stereo.pick_nav_contract import ARRIVED_VERIFY, LEAVE_GRANTED, LOCKED, NAVIGATING, PICK_ARMED, WAIT_PICK_TERMINAL, PickNavCoordinator


NOW = 10_000_000_000
MISSION = {"mission_id": "m-1", "nav_epoch": 7, "goal": {"frame_id": "map"}}


def _nav(stamp, **changes):
    value = {"mission_id": "m-1", "nav_epoch": 7, "stamp_ns": stamp, "result": "succeeded", "position_error_m": .04, "yaw_error_rad": .07, "linear_speed_mps": .0, "angular_speed_radps": .0}
    value.update(changes)
    return value


def _snapshot(**changes):
    value = {"state": "snapshot_frozen", "mission_id": "m-1", "nav_epoch": 7, "transaction_id": "tx-1"}
    value.update(changes)
    return value


def _pick(**changes):
    value = {"mission_id": "m-1", "nav_epoch": 7, "transaction_id": "tx-1", "calibration_id": "cal-1", "trusted_for_execution": True, "dry_run": False}
    value.update(changes)
    return value


def _armed():
    machine = PickNavCoordinator()
    assert machine.start(MISSION)["state"] == NAVIGATING
    assert machine.navigation_evidence(_nav(NOW), now_ns=NOW)["state"] == ARRIVED_VERIFY
    assert machine.navigation_evidence(_nav(NOW + 500_000_000), now_ns=NOW + 500_000_000)["state"] == PICK_ARMED
    return machine


def test_mission_start_needs_only_mission_and_epoch_then_arms_after_stationary_window():
    gate = _armed()._gate("test")
    assert gate["schema"] == "pick_nav_gate/v1"
    assert gate["state"] == PICK_ARMED and gate["pick_authorized"] is True
    assert gate["mission_id"] == "m-1" and gate["nav_epoch"] == 7
    assert gate["arrival_evidence"] == {"stamp_ns": NOW + 500_000_000, "odom_stationary_sec": .5, "linear_speed_m_s": 0.0, "angular_speed_rad_s": 0.0}


def test_nav_success_alone_and_motion_do_not_arm_pick():
    machine = PickNavCoordinator(); machine.start(MISSION)
    assert machine.navigation_evidence(_nav(NOW, linear_speed_mps=.011), now_ns=NOW)["reason"] == "arrival_not_stationary"
    assert machine.navigation_evidence(_nav(NOW + 500_000_000), now_ns=NOW + 500_000_000)["state"] == ARRIVED_VERIFY


def test_bad_arrival_nav_failure_stale_or_epoch_mismatch_lock_and_latch_until_explicit_reset():
    for evidence in (_nav(NOW, position_error_m=.051), _nav(NOW, result="failed"), _nav(NOW - 400_000_000), _nav(NOW, nav_epoch=8)):
        machine = PickNavCoordinator(); machine.start(MISSION)
        assert machine.navigation_evidence(evidence, now_ns=NOW)["state"] == LOCKED
        assert machine.start(MISSION)["state"] == LOCKED
        assert machine.reset()["state"] == LOCKED
        assert machine.reset(explicit=True)["state"] == "IDLE"


def test_snapshot_binds_transaction_and_pick_start_binds_calibration_before_terminal():
    machine = _armed()
    assert machine.bind_snapshot(_snapshot())["state"] == WAIT_PICK_TERMINAL
    assert machine.begin_pick(_pick())["reason"] == "pick_started"
    assert machine.pick_terminal(_pick(state="succeeded"))["state"] == LEAVE_GRANTED


def test_old_snapshot_or_terminal_mismatch_and_dry_run_or_failure_lock():
    cases = ((_snapshot(transaction_id="old", mission_id="old"), None), (_snapshot(), _pick(state="failed")), (_snapshot(), _pick(state="succeeded", dry_run=True)), (_snapshot(), _pick(state="succeeded", transaction_id="old")))
    for snapshot, terminal in cases:
        machine = _armed()
        result = machine.bind_snapshot(snapshot)
        if terminal is None:
            assert result["state"] == LOCKED
            continue
        assert result["state"] == WAIT_PICK_TERMINAL
        assert machine.begin_pick(_pick())["state"] == WAIT_PICK_TERMINAL
        assert machine.pick_terminal(terminal)["state"] == LOCKED
