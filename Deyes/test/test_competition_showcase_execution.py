from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import yaml


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "Deyes/src/deyes_stereo"))

from deyes_stereo.competition_showcase_contract import build_showcase_target  # noqa: E402


def _load_script(name: str):
    path = ROOT / "scripts" / name
    spec = importlib.util.spec_from_file_location(name.removesuffix(".py"), path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeMercury:
    def __init__(self, feedback: list[float] | None = None) -> None:
        self.pose = [0.0] * 6
        self.calls: list[tuple[object, ...]] = []
        self.feedback = iter(feedback or [10.0] * 6)

    def is_power_on(self) -> bool: return True
    def set_gripper_mode(self, mode: int) -> None: self.calls.append(("mode", mode))
    def set_gripper_value(self, value: int, speed: int) -> None: self.calls.append(("gripper", value, speed))
    def get_gripper_value(self) -> float: return next(self.feedback)
    def send_base_coords(self, pose: list[float], speed: int) -> None:
        self.pose = list(pose); self.calls.append(("move", tuple(pose), speed))
    def get_base_coords(self) -> list[float]: return list(self.pose)
    def get_robot_status(self) -> list[int]: return [0] * 6


def test_showcase_pick_reports_completed_motion_without_claiming_object_success(
    tmp_path: Path, monkeypatch,
) -> None:
    script = _load_script("pick_pen_degraded.py")
    profile = yaml.safe_load(
        (ROOT / "Deyes/config/stereo/competition_venue_65cm.yaml").read_text()
    )
    profile["transport"].update({
        "transport_validated": True,
        "kinematics_validated": True,
        "collision_clearance_validated": True,
        "tcp_vertical_clearance_conservative_mm": 10.0,
    })
    profile_path = tmp_path / "profile.yaml"
    profile_path.write_text(yaml.safe_dump(profile), encoding="utf-8")
    target_path = tmp_path / "showcase-target.json"
    target_path.write_text(json.dumps(build_showcase_target("target_timeout")), encoding="utf-8")
    result_path = tmp_path / "pick-result.json"
    fake = FakeMercury()
    monkeypatch.setitem(sys.modules, "pymycobot", SimpleNamespace(Mercury=lambda *_: fake))
    monkeypatch.setattr(script.time, "sleep", lambda _: None)
    monkeypatch.setattr(sys, "argv", [
        "pick_pen_degraded.py",
        "--venue-profile", str(profile_path),
        "--showcase-target-json", str(target_path),
        "--result-json", str(result_path),
    ])

    assert script.main() == 2
    result = json.loads(result_path.read_text(encoding="utf-8"))
    assert result["motion_completed"] is True
    assert result["transport_pose_reached"] is True
    assert result["hardware_ok"] is True
    assert result["object_grasp_verified"] is False
    assert result["navigation_permitted"] is False
    assert result["verification_failure_class"] == "perception"
    assert any(call[0] == "move" and call[1][2] == 260.0 for call in fake.calls)


def test_showcase_place_reports_motion_success_without_claiming_object_delivery(
    tmp_path: Path, monkeypatch,
) -> None:
    script = _load_script("place_pen_degraded.py")
    profile = yaml.safe_load(
        (ROOT / "Deyes/config/stereo/competition_venue_65cm.yaml").read_text()
    )
    profile["transport"].update({
        "transport_validated": True,
        "kinematics_validated": True,
        "collision_clearance_validated": True,
        "tcp_vertical_clearance_conservative_mm": 10.0,
    })
    profile_path = tmp_path / "profile.yaml"
    profile_path.write_text(yaml.safe_dump(profile), encoding="utf-8")
    result_path = tmp_path / "place-result.json"
    fake = FakeMercury()
    monkeypatch.setitem(sys.modules, "pymycobot", SimpleNamespace(Mercury=lambda *_: fake))
    monkeypatch.setattr(script.time, "sleep", lambda _: None)
    monkeypatch.setattr(sys, "argv", [
        "place_pen_degraded.py",
        "--venue-profile", str(profile_path),
        "--object-state", "unverified",
        "--showcase-mode",
        "--result-json", str(result_path),
    ])

    assert script.main() == 0
    result = json.loads(result_path.read_text(encoding="utf-8"))
    assert result["success"] is True
    assert result["motion_completed"] is True
    assert result["object_state"] == "unverified"
    assert result["object_delivery_verified"] is False
    assert result["showcase_mode"] is True


def test_sensor_verified_pick_keeps_navigation_permission_truthful(
    tmp_path: Path, monkeypatch,
) -> None:
    script = _load_script("pick_pen_degraded.py")
    profile = yaml.safe_load(
        (ROOT / "Deyes/config/stereo/competition_venue_65cm.yaml").read_text()
    )
    profile["transport"].update({
        "transport_validated": True,
        "kinematics_validated": True,
        "collision_clearance_validated": True,
        "tcp_vertical_clearance_conservative_mm": 10.0,
    })
    profile_path = tmp_path / "profile.yaml"
    profile_path.write_text(yaml.safe_dump(profile), encoding="utf-8")
    target_path = tmp_path / "target.json"
    target_path.write_text("{}", encoding="utf-8")
    feedback_path = tmp_path / "feedback.json"
    result_path = tmp_path / "result.json"
    fake = FakeMercury([10.0] * 3 + [16.0] * 3)
    monkeypatch.setitem(sys.modules, "pymycobot", SimpleNamespace(Mercury=lambda *_: fake))
    monkeypatch.setattr(script.time, "sleep", lambda _: None)
    monkeypatch.setattr(script, "_capture_feedback", lambda *_args, **_kwargs: {
        "schema": "competition_grasp_feedback/v1",
        "live": True,
        "source": "live_ros2_and_mercury_feedback",
        "roi_pen_last3": [False, False, False],
        "detector_frames_last3_ambiguous": [False, False, False],
        "gripper_feedback_delta": 6.0,
    })
    monkeypatch.setattr(sys, "argv", [
        "pick_pen_degraded.py",
        "--venue-profile", str(profile_path),
        "--target-json", str(target_path),
        "--feedback-adapter", str(ROOT / "scripts/competition_grasp_feedback_adapter.py"),
        "--feedback-json", str(feedback_path),
        "--result-json", str(result_path),
    ])

    assert script.main() == 0
    result = json.loads(result_path.read_text(encoding="utf-8"))
    assert result["object_grasp_verified"] is True
    assert result["success"] is True
    assert result["navigation_permitted"] is True
    assert result["verification_failure_class"] is None
