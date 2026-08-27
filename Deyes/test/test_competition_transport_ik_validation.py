import hashlib
import json
from pathlib import Path

import pytest
import yaml

from deyes_stereo.competition_transport_ik_validation import validate_official_urdf_solution


REPO_ROOT = Path(__file__).resolve().parents[2]
DEYES_ROOT = REPO_ROOT / "Deyes"
URDF = DEYES_ROOT / "test/fixtures/mercury_x1_official_527e1c787c2b.urdf"
SOURCE_URDF_SHA256 = "745075e319bc935beb03d4894ccd5d44fdc92cc3e01c5d56aa9920a3164cfdcd"
FIXTURE_SHA256 = "aa9d8c94a22716d9f7bafda1902bf7c5768137701be02205cf7ff661f8211716"
EVIDENCE = REPO_ROOT / "docs/evidence/competition_transport_ik_20260827/manifest.json"
SOLUTION = [
    0.9039235121922143,
    1.0947778286517673,
    -0.5541657632173106,
    -1.4630624027485444,
    0.5699777942163569,
    1.297798402557579,
]


def test_official_urdf_fixture_is_pinned_and_transport_solution_recomputes() -> None:
    fixture_bytes = URDF.read_bytes()
    assert hashlib.sha256(fixture_bytes).hexdigest() == FIXTURE_SHA256
    assert fixture_bytes.endswith(b"\n")
    assert hashlib.sha256(fixture_bytes[:-1]).hexdigest() == SOURCE_URDF_SHA256
    evidence = json.loads(EVIDENCE.read_text(encoding="utf-8"))
    source = evidence["official_urdf_source"]
    assert source["commit"] == "527e1c787c2bd86189de7c8df0f9879380ffd9c5"
    assert source["sha256"] == SOURCE_URDF_SHA256
    assert source["fixture_path"] == str(URDF.relative_to(REPO_ROOT)).replace("\\", "/")
    assert source["fixture_sha256"] == FIXTURE_SHA256
    assert source["fixture_scope"] == "full_official_urdf"

    result = validate_official_urdf_solution(URDF, SOLUTION)
    assert result["validated"] is True
    assert result["joint_limits_passed"] is True
    assert result["position_residual_mm"] <= 5.0
    assert result["orientation_residual_deg"] <= 2.0
    assert result["fk_xyz_m"] == pytest.approx(evidence["transport_ik"]["fk_xyz_m"], abs=1e-12)
    assert result["position_residual_mm"] == pytest.approx(
        evidence["transport_ik"]["position_residual_mm"], abs=1e-12
    )
    assert result["orientation_residual_deg"] == pytest.approx(
        evidence["transport_ik"]["orientation_residual_deg"], abs=1e-12
    )
    assert result["joint_limit_evidence"] == evidence["transport_ik"]["joint_limit_evidence"]


def test_tcp_evidence_is_explicitly_vertical_only_and_not_collision_clearance() -> None:
    evidence = json.loads(EVIDENCE.read_text(encoding="utf-8"))
    clearance = evidence["tcp_vertical_clearance"]
    assert clearance["reference_touch_z_mm"] == 135.0
    assert clearance["pick_nominal_mm"] == [100.0, 45.0, 5.0, 0.0, 45.0, 100.0]
    assert clearance["transport_nominal_mm"] == 125.0
    assert clearance["place_nominal_mm"] == [65.0, 30.0, 65.0, 125.0]
    assert clearance["minimum_non_contact_nominal_mm"] == 5.0
    assert clearance["feedback_xyz_tolerance_mm"] == 5.0
    assert clearance["conservative_minimum_mm"] == 0.0
    assert evidence["collision_clearance_validated"] is False

    site = yaml.safe_load(
        (DEYES_ROOT / "config/stereo/competition_venue_65cm.yaml").read_text(encoding="utf-8")
    )
    transport = site["transport"]
    assert transport["evidence_manifest"] == str(EVIDENCE.relative_to(REPO_ROOT)).replace("\\", "/")
    assert transport["tcp_vertical_clearance_nominal_mm"] == 5.0
    assert transport["tcp_vertical_clearance_conservative_mm"] == 0.0
    assert transport["collision_clearance_validated"] is False
    assert transport["kinematics_validated"] is True
    assert transport["transport_validated"] is False
    assert transport["live_behavior_when_unvalidated"] == "fail_closed_before_hardware_init"
