import importlib.util
import json
from pathlib import Path
import sys

import yaml

from deyes_stereo.competition_clearance_evidence import canonical_sha256, motion_contract


ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools/finalize_isaac_clearance_evidence.py"


def _load():
    spec = importlib.util.spec_from_file_location("finalize_clearance", TOOL)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def _raw(profile, clearance=11.0):
    return {
        "passed": True,
        "assets": {"collision_prim_count": 55, "all_required_collisions_enabled": True,
                   "scene_usd_sha256": "1"*64, "robot_usd_sha256": "2"*64,
                   "scene_config_sha256": "3"*64},
        "simulation": {"physics_hz": 60.0, "synthetic_attachment": False,
                       "rigid_body_disabled": False, "teleport_used": False},
        "initial_pose": {"both_arms_auto_stowed": True, "selected_order": "left_then_right",
                         "power_on_left_rad": [0.0]*6, "power_on_right_rad": [0.0]*6,
                         "stow_left_rad": [0.1]*6, "stow_right_rad": [-0.1]*6},
        "run": {"passed": True, "nav_table_1_reached": True, "nav_table_2_reached": True,
                "ik_fk_passed": True, "joint_feedback_passed": True, "stage_timeouts_passed": True,
                "dynamic_contact_grasp": True, "pen_placed_on_table_2": True, "forbidden_contacts": [],
                "minimum_arm_raw_mm": clearance+8, "minimum_arm_conservative_mm": clearance,
                "minimum_fingertip_table_raw_mm": 2, "minimum_navigation_raw_mm": 58,
                "minimum_navigation_conservative_mm": 50, "pen_lift_mm": 30},
    }


def test_finalizer_writes_unlock_only_for_two_valid_runs(tmp_path, monkeypatch):
    module = _load()
    profile_path = ROOT / "config/stereo/competition_venue_65cm.yaml"
    profile = yaml.safe_load(profile_path.read_text())
    raw_paths=[]
    for index, clearance in enumerate((11.0, 10.5)):
        path=tmp_path/f"raw{index}.json"; path.write_text(json.dumps(_raw(profile,clearance))); raw_paths.append(path)
    evidence=tmp_path/"competition_isaac_clearance_evidence.json"; output=tmp_path/"profile.yaml"
    monkeypatch.setattr(sys,"argv",["tool","--profile",str(profile_path),"--raw-run",str(raw_paths[0]),
        "--raw-run",str(raw_paths[1]),"--output-evidence",str(evidence),"--output-profile",str(output)])
    assert module.main()==0
    result=yaml.safe_load(output.read_text())
    assert result["transport"]["collision_clearance_validated"] is True
    assert result["transport"]["tcp_vertical_clearance_conservative_mm"]==10.5


def test_finalizer_does_not_write_profile_on_forbidden_contact(tmp_path, monkeypatch):
    module = _load(); profile_path=ROOT/"config/stereo/competition_venue_65cm.yaml"
    profile=yaml.safe_load(profile_path.read_text()); raw=_raw(profile); raw["run"]["forbidden_contacts"]=["arm:table"]
    paths=[]
    for i in range(2):
        p=tmp_path/f"bad{i}.json"; p.write_text(json.dumps(raw)); paths.append(p)
    output=tmp_path/"profile.yaml"
    monkeypatch.setattr(sys,"argv",["tool","--profile",str(profile_path),"--raw-run",str(paths[0]),
        "--raw-run",str(paths[1]),"--output-evidence",str(tmp_path/"evidence.json"),"--output-profile",str(output)])
    assert module.main()==module.EXIT_CONTACT
    assert not output.exists()
