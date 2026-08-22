"""Static safety checks for the Phase 4 deployment and field-evidence helpers."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_deployment_requires_environment_and_never_deletes_or_kills() -> None:
    content = (ROOT / "tools" / "deploy_deyes_stereo.sh").read_text(encoding="utf-8")
    assert 'ROBOT_IP:?' in content
    assert 'ROBOT_USER:?' in content
    assert 'REMOTE_WORKSPACE:?' in content
    assert 'REMOTE_PROJECT_ROOT="${REMOTE_PROJECT_ROOT:-${REMOTE_WORKSPACE}/src/Deyes}"' in content
    assert 'REMOTE_PACKAGE_ROOT="${REMOTE_PACKAGE_ROOT:-${REMOTE_PROJECT_ROOT}/src}"' in content
    assert "--base-paths '${REMOTE_PACKAGE_ROOT}/deyes_capture_cpp'" in content
    assert "-DOpenCV_DIR=${REMOTE_OPENCV_PREFIX}/lib/cmake/opencv4" in content
    assert "DUPLICATE_ROS_PACKAGE:" in content
    assert "${REMOTE_WORKSPACE}/src/\\${package}/package.xml" in content
    assert "BatchMode=yes" in content
    assert "rsync -a --delete" not in content
    assert "kill -" not in content.lower()
    assert "pkill" not in content.lower()
    assert "I_UNDERSTAND_REMOTE_WRITE" in content
    assert "colcon --log-base '${REMOTE_TEMP}/logs' build" in content


def test_field_commands_keep_evidence_outside_the_checkout() -> None:
    content = (ROOT / "tools" / "m4_acceptance_commands.sh").read_text(encoding="utf-8")
    assert "TEMP_ROOT:?" in content
    assert "runtime_acceptance_monitor" in content
    assert "truth_samples.csv" in content
    assert "/x1/stereo/points_status" in content
