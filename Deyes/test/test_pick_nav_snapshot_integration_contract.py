from deyes_stereo.pick_nav_contract import PickNavCoordinator
from deyes_stereo.single_shot_snapshot_contract import validate_nav_gate


def test_armed_coordinator_gate_is_accepted_by_existing_snapshot_validator():
    machine = PickNavCoordinator()
    machine.start({"mission_id": "desk-a", "nav_epoch": 2})
    machine.navigation_evidence({"mission_id": "desk-a", "nav_epoch": 2, "stamp_ns": 1_000_000_000, "result": "succeeded", "position_error_m": .01, "yaw_error_rad": .01, "linear_speed_mps": 0.0, "angular_speed_radps": 0.0}, now_ns=1_000_000_000)
    gate = machine.navigation_evidence({"mission_id": "desk-a", "nav_epoch": 2, "stamp_ns": 1_500_000_000, "result": "succeeded", "position_error_m": .01, "yaw_error_rad": .01, "linear_speed_mps": 0.0, "angular_speed_radps": 0.0}, now_ns=1_500_000_000)
    parsed, reason = validate_nav_gate(gate, receipt_age_sec=.1)
    assert reason == "ok" and parsed is not None and parsed.mission_id == "desk-a"


def test_old_or_mismatched_gate_cannot_authorize_snapshot_or_transaction():
    machine = PickNavCoordinator()
    machine.start({"mission_id": "desk-a", "nav_epoch": 2})
    assert machine.navigation_evidence({"mission_id": "old", "nav_epoch": 2, "stamp_ns": 1, "result": "succeeded", "position_error_m": 0., "yaw_error_rad": 0., "linear_speed_mps": 0., "angular_speed_radps": 0.}, now_ns=1)["pick_authorized"] is False
    gate = machine._gate("locked")
    assert validate_nav_gate(gate, receipt_age_sec=.1)[1] == "nav_gate_state_not_pick_armed"
