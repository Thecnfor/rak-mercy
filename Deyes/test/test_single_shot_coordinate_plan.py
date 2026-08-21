import numpy as np
import pytest

from deyes_stereo.coordinate_chain_contract import transform_request
from deyes_stereo.pen_pick_dry_run_contract import PickPlanLimits, build_plan_from_coordinate_result


def test_aggregate_grasp_geometry_transforms_point_axis_and_normal_together():
    request={"kind":"grasp_geometry","source_frame":"left_camera_optical_frame","target_frame":"base_link","stamp_ns":42,"position_m":[.1,.2,.3],"axis_unit":[1,0,0],"approach_normal_unit":[0,0,1],"candidate_id":"pen-1","transaction_id":"pick-42","quality":{"detection_confidence":.9,"depth_mad_m":.002,"mask_depth_valid_ratio":.8}}
    rotation=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
    result=transform_request(request,rotation,np.array([.3,0.,.1]),tf_quaternion_xyzw=[0,0,.70710678,.70710678])
    assert result["grasp_point_base_m"]==pytest.approx([.1,.1,.4])
    assert result["axis_base_unit"]==pytest.approx([0,1,0])
    assert result["approach_normal_base_unit"]==pytest.approx([0,0,1])


def test_coordinate_result_identity_is_required_and_bound_into_plan():
    coordinate={"kind":"grasp_geometry","frame_id":"base_link","stamp_ns":1_000_000_000,"candidate_id":"pen-1","transaction_id":"pick-1000000000","calibration_id":"handeye-1","trusted_for_execution":True,"grasp_point_base_m":[.4,0.,.2],"axis_base_unit":[1,0,0],"approach_normal_base_unit":[0,0,1],"quality":{"detection_confidence":.9,"depth_mad_m":.002,"mask_depth_valid_ratio":.8}}
    limits=PickPlanLimits(max_candidate_age_sec=1.,workspace_min_base_m=(0.,-.5,0.),workspace_max_base_m=(1.,.5,1.))
    plan=build_plan_from_coordinate_result(coordinate,now_stamp_ns=1_100_000_000,limits=limits,site_profile_validated=True)
    assert plan["state"]=="dry_run_ready" and plan["transaction_id"]=="pick-1000000000"
    assert plan["calibration_id"]=="handeye-1" and plan["target_id"]=="pen-1"
    coordinate["transaction_id"]="pick-other"
    assert build_plan_from_coordinate_result(coordinate,now_stamp_ns=1_100_000_000,limits=limits,site_profile_validated=True)["reason"]=="coordinate_transaction_identity_invalid"


def test_coordinate_navigation_identity_is_bound_into_plan_and_rejects_partial_values():
    coordinate={"kind":"grasp_geometry","frame_id":"base_link","stamp_ns":1_000_000_000,"candidate_id":"pen-1","transaction_id":"pick-1000000000","calibration_id":"handeye-1","trusted_for_execution":True,"mission_id":"m-1","nav_epoch":6,"grasp_point_base_m":[.4,0.,.2],"axis_base_unit":[1,0,0],"approach_normal_base_unit":[0,0,1],"quality":{"detection_confidence":.9,"depth_mad_m":.002,"mask_depth_valid_ratio":.8}}
    limits=PickPlanLimits(max_candidate_age_sec=1.,workspace_min_base_m=(0.,-.5,0.),workspace_max_base_m=(1.,.5,1.))
    plan=build_plan_from_coordinate_result(coordinate,now_stamp_ns=1_100_000_000,limits=limits,site_profile_validated=True)
    assert plan["mission_id"]=="m-1" and plan["nav_epoch"]==6
    coordinate.pop("nav_epoch")
    assert build_plan_from_coordinate_result(coordinate,now_stamp_ns=1_100_000_000,limits=limits,site_profile_validated=True)["reason"]=="coordinate_navigation_identity_incomplete"
