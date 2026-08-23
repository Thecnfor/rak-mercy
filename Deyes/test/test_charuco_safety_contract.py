"""Regression tests for offline ChArUco safety gates; no robot or serial use."""
import numpy as np
from deyes_stereo.charuco_board_generator import board_metadata
from deyes_stereo.charuco_handeye import validate_session
from deyes_stereo.stereo_calibration_contract import CALIBRATION_SIZE, validation_gate

def _sample(index: int) -> dict:
    transform = np.eye(4); transform[:3, 3] = [.11 * index, 0, 0]
    return {"locked": True, "serial_busy": False, "feedback_fresh": True, "captured_while_dragging": False,
            "head_deviation_deg": .1, "common_charuco_ids": 12, "skew_ms": 5,
            "base_T_gripper": transform.tolist()}

def test_board_dimensions_and_identity_are_fixed():
    assert board_metadata("stereo") == {"squares_x":8,"squares_y":6,"square_length_mm":30,"marker_length_mm":22,"dictionary":"DICT_5X5_1000","schema":"deyes_charuco_board/v1","kind":"stereo","width_mm":240,"height_mm":180}
    assert board_metadata("handeye")["width_mm"] == 100

def test_charuco_stereo_is_allowed_but_unknown_sources_fail_closed():
    assert validation_gate(sample_count=50,resolution=CALIBRATION_SIZE,reproj_rms_px=.2,epipolar_p95_px=.2,source="physical_charuco",left_right_confirmed=True,baseline_sign_confirmed=True,scale_confirmed=True,coverage_complete=True,board_inner_corners=(8,6),square_size_m=.03).validated
    assert not validation_gate(sample_count=50,resolution=CALIBRATION_SIZE,reproj_rms_px=.2,epipolar_p95_px=.2,source="sim",left_right_confirmed=True,baseline_sign_confirmed=True,scale_confirmed=True,coverage_complete=True,board_inner_corners=(8,6),square_size_m=.03).validated

def test_handeye_rejects_unsafe_and_degenerate_evidence():
    payload={"drag_mode":"manual_save_pause","drag_teach_execute_called":False,"samples":[_sample(i) for i in range(12)],"rotation_axes_span_deg":{"axis_1":31,"axis_2":31}}
    assert validate_session(payload).ready
    payload["samples"][0]["captured_while_dragging"] = True
    assert "unsafe_robot_sample" in validate_session(payload).reasons
    payload["samples"][0]["captured_while_dragging"] = False
    payload["rotation_axes_span_deg"]={"axis_1":29,"axis_2":31}
    assert "rotation_span_below_30deg" in validate_session(payload).reasons
