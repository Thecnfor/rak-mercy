from math import cos, radians, sin

from deyes_stereo.handeye_multiview_contract import (
    HandeyeCollectionLimits, build_handeye_multiview_session,
    build_handeye_solver_payload,
)


def _sample(index, *, skew_ns=5_000_000):
    angle = radians(index * 4.0)
    return {
        "sample_id": f"s-{index}", "correspondence_id": f"board-corner-{index}",
        "arm_side": "left" if index % 2 else "right", "joint_positions_deg": [float(index)] * 6,
        "camera_checkerboard_pose": {"frame_id": "left_camera_optical_frame", "stamp_ns": 10_000_000_000 + index * 100_000_000, "position_m": [.03 * index, .02 * (index % 3), .40 + .01 * index], "quaternion_xyzw": [0., 0., sin(angle / 2), cos(angle / 2)]},
        "end_effector_base_pose": {"frame_id": "base_link", "stamp_ns": 10_000_000_000 + index * 100_000_000 + skew_ns, "position_m": [.30 + .03 * index, -.10 + .02 * (index % 3), .50 + .01 * index], "quaternion_xyzw": [0., sin(angle / 2), 0., cos(angle / 2)]},
    }


def _payload(samples=None, **changes):
    result = {"session_id": "handeye-session-1", "calibration_id": "handeye-1", "robot_id": "x1", "camera_pair_id": "pair-1", "stereo_calibration_id": "stereo-1", "dry_run": True, "samples": samples if samples is not None else [_sample(i) for i in range(8)]}
    result.update(changes)
    return result


def test_multiview_session_requires_time_aligned_diverse_samples_but_stays_unvalidated():
    session = build_handeye_multiview_session(_payload())
    assert session["state"] == "ready_for_solver"
    assert session["validated"] is False and session["commands_emitted"] is False
    assert session["metrics"]["sample_count"] == 8
    assert session["metrics"]["arms_used"] == ["left", "right"]
    solver = build_handeye_solver_payload(session, operator_confirmation=True)
    assert len(solver["correspondences"]) == 8


def test_timestamp_execution_and_coordinate_frame_failures_are_explicit():
    skewed = [_sample(i, skew_ns=21_000_000 if i == 0 else 5_000_000) for i in range(8)]
    assert build_handeye_multiview_session(_payload(skewed))["reason"] == "camera_end_effector_timestamp_skew_exceeds_limit"
    assert build_handeye_multiview_session(_payload(request_execution=True))["reason"] == "motion_execution_not_supported_by_handeye_collection"
    bad_frame = [_sample(i) for i in range(8)]
    bad_frame[0]["end_effector_base_pose"]["frame_id"] = "tool0"
    assert build_handeye_multiview_session(_payload(bad_frame))["reason"] == "end_effector_frame_must_be_base_link"


def test_sample_count_and_diversity_fail_closed():
    assert build_handeye_multiview_session(_payload([_sample(i) for i in range(7)]))["reason"] == "insufficient_multiview_samples"
    repeated = [_sample(0) for _ in range(8)]
    for index, sample in enumerate(repeated):
        sample["correspondence_id"] = f"unique-{index}"
    result = build_handeye_multiview_session(_payload(repeated), limits=HandeyeCollectionLimits(min_rotation_span_deg=1.0))
    assert result["reason"] == "camera_translation_diversity_insufficient"


def test_nonready_session_cannot_become_solver_payload():
    session = build_handeye_multiview_session(_payload([_sample(i) for i in range(7)]))
    try:
        build_handeye_solver_payload(session)
    except ValueError as exc:
        assert str(exc) == "handeye_session_not_ready_for_solver"
    else:
        raise AssertionError("rejected collection must not feed the solver")
