import numpy as np
import pytest
from deyes_stereo.competition_pick_target_contract import TargetPolicy, build_competition_pick_target

STAMP=8_000_000_123
DETECTION={"stamp_ns":STAMP,"complete":True,"detections":[{"target_id":"p","bbox_xyxy":[20,20,60,60]}]}
FEATURES={"stamp_ns":STAMP,"features":[{"target_id":"p","axis_complete":True,"axis_endpoints_px":[[30,40],[50,40]]}]}
INFO={"stamp_ns":STAMP,"depth_stamp_ns":STAMP}
HULL=[[.2,-.2],[.6,-.2],[.6,.2],[.2,.2]]

class Projector:
    def ray_for_pixel(self,u,v,info):
        return {"origin_m":[0,0,.5],"direction_unit":[.4,.01,-.365],"predicted_camera_z_m":.7}

def depth(value=.7): return np.full((100,100),value)
def plane(distance, **kw): return {"stamp_ns":STAMP,"valid_for_table_removal":True,"degraded":False,
    "plane_distance_camera_m":distance,"residual_rms_m":.002,**kw}

def test_axis_midpoint_projects_to_fixed_135_without_clamping_and_560_to_650_audit():
    policy=TargetPolicy(reference_plane_distance_m=.60)
    result=build_competition_pick_target(detection=DETECTION,pen_features=FEATURES,depth_m=depth(),camera_info=INFO,
        ground_plane=plane(.69),projector=Projector(),touch_hull_xy_m=HULL,policy=policy)
    assert result["valid"] and result["trusted_for_venue_execution"]
    assert result["selection_source"]=="axis_midpoint" and result["right_arm_sdk_target_m"][2]==.135
    assert result["xy_clamped"] is False and result["height_verification"]=="fixed_height_verified"

def test_healthy_plane_over_25mm_from_reference_plus_90_rejects_but_bad_plane_continues():
    policy=TargetPolicy(reference_plane_distance_m=.60)
    bad=build_competition_pick_target(detection=DETECTION,pen_features=FEATURES,depth_m=depth(),camera_info=INFO,
        ground_plane=plane(.72),projector=Projector(),touch_hull_xy_m=HULL,policy=policy)
    assert bad["reason"]=="table_height_deviation_exceeds_25mm"
    poor=plane(.90); poor["residual_rms_m"]=.02
    continued=build_competition_pick_target(detection=DETECTION,pen_features=FEATURES,depth_m=depth(),camera_info=INFO,
        ground_plane=poor,projector=Projector(),touch_hull_xy_m=HULL,policy=policy)
    assert continued["valid"] and continued["height_verification"]=="fixed_height_unverified"

def test_bbox_fallback_is_separate_requires_margin_and_depth_agreement():
    no_axis={"stamp_ns":STAMP,"features":[]}
    result=build_competition_pick_target(detection=DETECTION,pen_features=no_axis,depth_m=depth(),camera_info=INFO,
        ground_plane=None,projector=Projector(),touch_hull_xy_m=HULL,policy=TargetPolicy(bbox_fallback_enabled=True))
    assert result["valid"] and result["selection_source"]=="bbox_center"
    mismatch=build_competition_pick_target(detection=DETECTION,pen_features=FEATURES,depth_m=depth(.8),camera_info=INFO,
        ground_plane=None,projector=Projector(),touch_hull_xy_m=HULL)
    assert mismatch["reason"]=="projected_depth_disagrees_with_cuda_median"

def test_missing_projected_camera_z_and_outside_hull_fail_without_clamp():
    class Missing:
        def ray_for_pixel(self,u,v,info): return ([0,0,.5],[.4,.01,-.365])
    missing=build_competition_pick_target(detection=DETECTION,pen_features=FEATURES,depth_m=depth(),camera_info=INFO,
        ground_plane=None,projector=Missing(),touch_hull_xy_m=HULL)
    assert missing["reason"]=="projector_predicted_camera_z_missing"
    outside=build_competition_pick_target(detection=DETECTION,pen_features=FEATURES,depth_m=depth(),camera_info=INFO,
        ground_plane=None,projector=Projector(),touch_hull_xy_m=[[0,0],[.1,0],[.1,.1],[0,.1]])
    assert outside["reason"]=="target_outside_touch_convex_hull"

@pytest.mark.parametrize("changed",["detection","features","camera"])
def test_all_sensor_inputs_require_exact_stamp(changed):
    detection=dict(DETECTION); features=dict(FEATURES); info=dict(INFO)
    if changed=="detection": detection["stamp_ns"]+=1
    elif changed=="features": features["stamp_ns"]+=1
    else: info["stamp_ns"]+=1
    result=build_competition_pick_target(detection=detection,pen_features=features,depth_m=depth(),camera_info=info,
        ground_plane=None,projector=Projector(),touch_hull_xy_m=HULL)
    assert result["reason"]=="exact_stamp_mismatch"
