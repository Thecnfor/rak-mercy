from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SYSTEMD = ROOT / "tools" / "systemd"


def test_argus_recovery_is_exact_exit_75_only_and_rate_limited() -> None:
    content = (SYSTEMD / "deyes-argus-recover.sh").read_text(encoding="utf-8")

    assert '"${service_result}" != "failure"' in content
    assert '"${exit_code}" != "exited"' in content
    assert '"${exit_status}" != "75"' in content
    assert "max_recoveries=2" in content
    assert "window_sec=600" in content
    assert "systemctl restart nvargus-daemon.service" in content
    assert "pkill" not in content
    assert "killall" not in content


def test_capture_service_runs_camera_as_robot_user_and_uses_privileged_poststop_only() -> None:
    content = (SYSTEMD / "deyes-stereo-capture.service").read_text(encoding="utf-8")

    assert "User=elephant" in content
    assert "Restart=on-failure" in content
    assert "RestartSec=8" in content
    assert "StartLimitBurst=3" in content
    assert "ExecStopPost=+/usr/local/lib/deyes/deyes-argus-recover.sh" in content
    assert "ExecStart=/usr/local/lib/deyes/deyes-stereo-capture-exec.sh" in content


def test_capture_exec_preserves_watchdog_exit_and_requires_physical_calibration() -> None:
    content = (SYSTEMD / "deyes-stereo-capture-exec.sh").read_text(encoding="utf-8")

    assert 'DEYES_CALIB_PATH:?DEYES_CALIB_PATH must be set to a physical calibration YAML' in content
    assert "exec ros2 run deyes_capture_cpp imx219_stereo_capture_node" in content
    assert '-p calib_path:="${DEYES_CALIB_PATH}"' in content


def test_safe_deployment_syncs_supervisor_files_but_does_not_install_or_start_it() -> None:
    content = (ROOT / "tools" / "deploy_deyes_stereo.sh").read_text(encoding="utf-8")

    assert 'tools/systemd/' in content
    assert 'install_argus_capture_supervisor.sh' in content
    assert "systemctl enable" not in content
    assert "systemctl restart nvargus-daemon" not in content
