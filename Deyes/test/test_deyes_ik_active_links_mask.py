"""Regression tests for the Mercury official-URDF ikpy mask."""
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "src" / "deyes_ik_server"))

from deyes_ik_server import ikpy_solver  # noqa: E402


class FakeChainFactory:
    link_names = ()

    @classmethod
    def from_urdf_file(cls, _path, last_link_vector=None, **_kwargs):
        return SimpleNamespace(links=[SimpleNamespace(name=name) for name in cls.link_names])


def _install_fake_chain(monkeypatch, names):
    FakeChainFactory.link_names = tuple(names)
    monkeypatch.setattr(ikpy_solver, "_HAS_IKPY", True)
    monkeypatch.setattr(ikpy_solver, "ikpy", SimpleNamespace(chain=SimpleNamespace(Chain=FakeChainFactory)), raising=False)


def test_official_nine_link_chain_mask_has_nine_entries_and_six_active(monkeypatch):
    names = ["Base link", "joint1_R", "joint2_R", "joint3_R", "joint4_R",
             "joint5_R", "joint6_R", "joint7_R", "tool_fixed"]
    _install_fake_chain(monkeypatch, names)
    mask = ikpy_solver._build_active_links_mask("official-mercury-x1.urdf", "right")
    assert len(mask) == len(names) == 9
    assert sum(mask) == 6
    assert [name for name, active in zip(names, mask) if active] == names[1:7]


def test_unknown_joint_names_fail_closed(monkeypatch):
    # An upstream rename must not guess which links map to actuator channels.
    names = ["base_link", "axis_a", "axis_b", "axis_c", "axis_d",
             "axis_e", "axis_f", "axis_g", "tool_fixed"]
    _install_fake_chain(monkeypatch, names)
    with pytest.raises(ValueError, match="URDF_ARM_JOINTS_INVALID:right:expected=6:found=0"):
        ikpy_solver._build_active_links_mask("renamed-nine-link.urdf", "right")


def test_nonfinite_orientation_fails_before_ik_execution():
    solver = object.__new__(ikpy_solver.IkpySolver7DOF)
    result = solver.solve([0.4, 0.01, 0.2, float("nan"), -12.0, 0.0])
    assert result.success is False
    assert result.failure_code == "POSE_NONFINITE"
