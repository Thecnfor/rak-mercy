"""Synthetic ROS-free checks for classical pen-pixel extraction and stamp joins."""
import sys
from pathlib import Path
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/"src"/"deyes_stereo"))
from deyes_stereo.pen_feature_extractor import ExtractorParams, PenFeatureJoiner, RectifiedImage, extract_one_pen
from deyes_stereo.pen_grasp_contract import parse_pen_features

def scene(angle:float=0.,value:int=30,center=(80,60)):
    image=np.full((120,160),150,np.uint8); length=70; direction=np.array([np.cos(np.deg2rad(angle)),np.sin(np.deg2rad(angle))]); a=tuple(np.int32(np.array(center)-direction*length/2));b=tuple(np.int32(np.array(center)+direction*length/2));cv2.line(image,a,b,value,5);return image,[min(a[0],b[0])-8,min(a[1],b[1])-8,max(a[0],b[0])+8,max(a[1],b[1])+8]
def payload(stamp,bbox,count=1):
    return {"stamp_sec":0,"stamp_nanosec":stamp,"frame_id":"left","image_width":160,"image_height":120,"detections":[{"class_name":"pen","confidence":.9,"target_id":"target_00","bbox_xyxy":bbox} for _ in range(count)]}
def test_horizontal_vertical_diagonal_and_low_contrast_extract_axis():
    for angle,value in ((0,30),(90,30),(55,135)):
        gray,bbox=scene(angle,value);feature,reason=extract_one_pen(RectifiedImage(1,"left",160,120,gray),payload(1,bbox))
        assert feature is not None, reason
        assert len(feature["mask_pixels_px"]) >= 12 and len(feature["axis_endpoints_px"]) == 2
def test_empty_multi_edge_and_stamp_fail_closed():
    gray,bbox=scene(); image=RectifiedImage(1,"left",160,120,gray)
    assert extract_one_pen(image,{**payload(1,bbox),"detections":[]})[1] == "waiting_for_one_pen"
    assert extract_one_pen(image,payload(1,bbox,2))[1] == "ambiguous_multi_target"
    assert extract_one_pen(image,{**payload(1,bbox),"ambiguous":True})[1] == "ambiguous_multi_target"
    assert extract_one_pen(image,payload(2,bbox))[1] == "detection_image_stamp_mismatch"
    assert extract_one_pen(image,{**payload(1,bbox),"frame_id":"raw_left"})[1] == "detection_image_frame_mismatch"
    assert extract_one_pen(image,{**payload(1,bbox),"image_width":159})[1] == "detection_image_size_mismatch"
    edge,b=scene(0,30,(15,60)); feature,reason=extract_one_pen(RectifiedImage(1,"left",160,120,edge),payload(1,b)); assert feature is not None and feature["axis_complete"] is False

def test_empty_box_rejects_and_insufficient_elongation_is_explicitly_incomplete():
    blank=np.full((120,160),150,np.uint8); box=[40,40,120,80]
    assert extract_one_pen(RectifiedImage(1,"left",160,120,blank),payload(1,box))[1] == "no_elongated_component"
    gray,bbox=scene(); feature,reason=extract_one_pen(RectifiedImage(1,"left",160,120,gray),payload(1,bbox),ExtractorParams(min_aspect_ratio=20.0))
    assert feature is not None and reason == "axis_incomplete" and feature["axis_complete"] is False
def test_exact_stamp_joiner_allows_out_of_order_and_expires():
    gray,bbox=scene(); image=RectifiedImage(7,"left",160,120,gray); joiner=PenFeatureJoiner(2,10)
    assert joiner.add_image(image,0) is None
    assert joiner.add_detection(payload(7,bbox),1) is not None
    assert joiner.add_detection(payload(9,bbox),2) is None
    joiner.expire(100); assert joiner.add_image(RectifiedImage(9,"left",160,120,gray),101) is None

def test_feature_keeps_pen_grasp_contract_fields():
    gray,bbox=scene(); feature,reason=extract_one_pen(RectifiedImage(1,"left",160,120,gray),payload(1,bbox))
    assert reason == "ok"
    parsed=parse_pen_features({"features":[feature]})
    assert parsed[0]["id"] == "target_00"
