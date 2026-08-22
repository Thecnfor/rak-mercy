from deyes_stereo.single_shot_snapshot_contract import SnapshotLimits, StabilitySample, StabilityTracker, new_transaction_id, validate_nav_gate


def sample(index, *, skew=2., center=(0.,0.,.68), normal=(0.,0.,1.), odom=True, rejects=0):
    return StabilitySample(
        stamp_ns=1_000_000_000+index*125_000_000,receipt_sec=index*.125,pair_skew_ms=skew,
        plane_center_m=center,plane_normal=normal,plane_valid=True,
        base_linear_m_s=0. if odom else None,base_angular_rad_s=0. if odom else None,
        joint_positions_deg=(0.,)*6 if odom else None,odom_age_sec=0. if odom else None,
        joint_age_sec=0. if odom else None,diagnostics_age_sec=0.,pair_reject_count=rejects,
    )


def test_debug_stability_freezes_exactly_once_after_five_samples_and_half_second():
    tracker=StabilityTracker(SnapshotLimits())
    for index in range(4):assert tracker.update(sample(index,odom=False))[0] is False
    assert tracker.update(sample(4,odom=False)) == (True,"stable_snapshot_ready")
    assert tracker.update(sample(5,odom=False)) == (False,"transaction_already_frozen")
    assert new_transaction_id(42)=="pick-42"


def test_live_stability_requires_fresh_odom_and_right_arm_feedback():
    tracker=StabilityTracker(live_mode=True)
    assert tracker.update(sample(0,odom=False))[1]=="odom_missing"
    assert tracker.update(sample(0))[1]=="waiting_for_stability_window"


def test_motion_skew_and_new_pair_rejection_reset_the_window():
    tracker=StabilityTracker()
    tracker.update(sample(0));tracker.update(sample(1))
    assert tracker.update(sample(2,center=(.02,0.,.68)))[1]=="plane_center_motion_exceeds_limit"
    assert tracker.update(sample(3,skew=11.))[1]=="pair_skew_exceeds_limit"
    tracker.update(sample(4,rejects=0))
    assert tracker.update(sample(5,rejects=1))[1]=="new_pair_rejection_observed"


def nav_gate(**overrides):
    result={
        "schema":"pick_nav_gate/v1", "state":"PICK_ARMED", "pick_authorized":True,
        "mission_id":"table-1-pen", "nav_epoch":7,
        "arrival_evidence":{"stamp_ns":1_000_000_000,"odom_stationary_sec":.5,"linear_speed_m_s":.0,"angular_speed_rad_s":.0},
    }
    result.update(overrides)
    return result


def test_nav_gate_requires_explicit_fresh_arrival_evidence_and_mission_identity():
    gate, reason=validate_nav_gate(nav_gate(),receipt_age_sec=.1)
    assert reason=="ok" and gate is not None
    assert gate.mission_id=="table-1-pen" and gate.nav_epoch==7
    assert validate_nav_gate(nav_gate(state="ARRIVED"),receipt_age_sec=.1)[1]=="nav_gate_state_not_pick_armed"
    assert validate_nav_gate(nav_gate(pick_authorized=False),receipt_age_sec=.1)[1]=="nav_gate_not_authorized"
    assert validate_nav_gate(nav_gate(mission_id=""),receipt_age_sec=.1)[1]=="nav_gate_mission_id_missing"
    assert validate_nav_gate(nav_gate(nav_epoch=0),receipt_age_sec=.1)[1]=="nav_gate_arrival_evidence_invalid"
    assert validate_nav_gate(nav_gate(),receipt_age_sec=.51)[1]=="nav_gate_arrival_evidence_stale"


def test_nav_gate_rejects_insufficient_or_moving_odom_evidence():
    gate=nav_gate();gate["arrival_evidence"]["odom_stationary_sec"]=.49
    assert validate_nav_gate(gate,receipt_age_sec=.1)[1]=="nav_gate_odom_not_stable_long_enough"
    gate=nav_gate();gate["arrival_evidence"]["linear_speed_m_s"]=.011
    assert validate_nav_gate(gate,receipt_age_sec=.1)[1]=="nav_gate_base_linear_velocity_exceeds_limit"
    gate=nav_gate();gate["arrival_evidence"]["angular_speed_rad_s"]=.021
    assert validate_nav_gate(gate,receipt_age_sec=.1)[1]=="nav_gate_base_angular_velocity_exceeds_limit"
