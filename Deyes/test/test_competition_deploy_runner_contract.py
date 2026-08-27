"""ROS-free/static checks for the one-key competition deployment and runner."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
RUNNER = ROOT / "scripts" / "race_onekey_try.sh"
DEPLOY = ROOT / "tools" / "deploy_competition_onekey.py"
PS1 = ROOT / "tools" / "deploy_competition_onekey.ps1"
EXPECTED_ONNX = "8916cbf25949d6e8b03c01e6ca1c7871aeac0ad105c931f1dd9881cb5d5a4c4e"


def _load_deploy():
    spec = importlib.util.spec_from_file_location("competition_deploy", DEPLOY)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _bash() -> str | None:
    found = shutil.which("bash")
    if found:
        return found
    candidate = Path(r"C:\Program Files\Git\bin\bash.exe")
    return str(candidate) if candidate.is_file() else None


def _validate(mode: str, tmp_path: Path, **values: str) -> subprocess.CompletedProcess[str]:
    bash = _bash()
    if not bash:
        raise unittest.SkipTest("bash unavailable; static runner checks still execute")
    env = os.environ.copy()
    env.update({"COMPETITION_VALIDATE_ONLY": mode, "LOG_DIR": str(tmp_path), "PYTHON_BIN": "py", **values})
    return subprocess.run([bash, str(RUNNER)], env=env, text=True, capture_output=True, check=False)


class CompetitionDeployRunnerTest(unittest.TestCase):
    def test_deploy_inventory_contains_exact_packages_scripts_config_and_pinned_onnx(self) -> None:
        deploy = _load_deploy(); remotes = {str(remote) for _, remote in deploy.local_files()}
        for package in ("deyes_interfaces", "deyes_capture_cpp", "deyes_stereo", "deyes_bringup"):
            self.assertTrue(any(path.startswith(f"ws/src/Deyes/src/{package}/") for path in remotes))
        self.assertTrue(any(path.startswith("ws/src/Deyes/config/") for path in remotes))
        self.assertIn("scripts/race_onekey_try.sh", remotes)
        self.assertIn("scripts/competition_target_snapshot_adapter.py", remotes)
        onnx = ROOT / "Deyes" / "models" / "pen" / "pen_student_01875_416_v1.onnx"
        self.assertEqual(hashlib.sha256(onnx.read_bytes()).hexdigest(), EXPECTED_ONNX)
        self.assertIn("models/pen/pen_student_01875_416_v1.onnx", remotes)

    def test_deploy_command_is_non_destructive_and_has_live_acceptance_gates(self) -> None:
        content = DEPLOY.read_text(encoding="utf-8")
        self.assertNotIn("rm -rf", content); self.assertNotIn("ALLOW_DEGRADED", content)
        for token in ("trtexec", "--fp16", "engine_sha256", "tensorrt_version", "cuda_version",
                      "ldd", "depth topic below 12Hz", "pair skew missing or above 10ms", "enable_pen_features:=true"):
            self.assertIn(token, content)
        self.assertIn("pkill -INT -f '[i]mx219_stereo_capture_node", content)
        bash = _bash()
        if bash:
            deploy = _load_deploy()
            command = deploy.remote_deploy_command(argparse.Namespace(
                remote_home="/home/elephant", stop_existing=True, vision_dry_run_seconds=1),
                deploy.PurePosixPath("/home/elephant/deyes_competition_deploy"))
            checked = subprocess.run([bash, "-n"], input=command, text=True, capture_output=True, check=False)
            self.assertEqual(checked.returncode, 0, checked.stderr)

    def test_runner_order_permissions_and_powershell_flags_are_fail_closed(self) -> None:
        content = RUNNER.read_text(encoding="utf-8")
        order = [content.index(token) for token in ("run_step nav_goal3", "run_step set_head", 'trace "competition_target" "started"',
            "run_step pick", "validate_grasp ||", "stop_vision\n\nsource_ros1", "run_step nav_goal4", "run_step place")]
        self.assertEqual(order, sorted(order))
        for token in ('FIXED_TABLE_HEIGHT_MM="${FIXED_TABLE_HEIGHT_MM:-650}"', 'ALLOW_BBOX_CENTER="${ALLOW_BBOX_CENTER:-1}"',
                      'ALLOW_FIXED_XY_FALLBACK="${ALLOW_FIXED_XY_FALLBACK:-0}"', 'FORCE_FIXED_TARGET="${FORCE_FIXED_TARGET:-0}"'):
            self.assertIn(token, content)
        self.assertNotIn("ALLOW_DEGRADED", content); self.assertNotIn("retry", content.lower())
        self.assertIn("competition_venue_65cm.yaml", content)
        self.assertIn("--x-mm", content); self.assertIn("--venue-profile", content); self.assertIn("--result-json", content)
        self.assertIn('data.get("success") is True', content); self.assertIn('data.get("navigation_permitted", verified) is True', content)
        adapter = (ROOT / "tools" / "competition_target_snapshot_adapter.py").read_text(encoding="utf-8")
        self.assertIn("waiting_for_exact_stamp_projector_adapter", adapter)
        self.assertIn("competition_pick_target/v1", adapter)
        ps1 = PS1.read_text(encoding="utf-8")
        self.assertIn("AllowFixedXyFallback", ps1); self.assertIn("ForceFixedTarget", ps1); self.assertNotIn("ALLOW_DEGRADED", ps1)

    def test_engine_sidecar_fault_matrix(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            tmp_path = Path(directory); engine = tmp_path / "model.engine"; engine.write_bytes(b"device-local-engine")
            manifest = tmp_path / "model.engine.manifest.json"
            manifest.write_text(json.dumps({"onnx_sha256": EXPECTED_ONNX, "engine_sha256": hashlib.sha256(engine.read_bytes()).hexdigest(),
                "tensorrt_version": "8", "cuda_version": "11.4", "input_shape": [1, 3, 416, 416],
                "output_layout": "yolov5:[1,N,5+C]", "precision": "fp16"}), encoding="utf-8")
            result = _validate("engine", tmp_path, DEYES_ENGINE_PATH=str(engine), DEYES_ENGINE_MANIFEST=str(manifest))
            self.assertEqual(result.returncode, 0, result.stderr)
            engine.write_bytes(b"tampered")
            result = _validate("engine", tmp_path, DEYES_ENGINE_PATH=str(engine), DEYES_ENGINE_MANIFEST=str(manifest))
            self.assertNotEqual(result.returncode, 0); self.assertIn("engine SHA", result.stderr)

    def test_ground_plane_fault_matrix_and_grasp_gate(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            tmp_path = Path(directory); target = tmp_path / "target.json"; admission = tmp_path / "admission.json"
            common = {"COMPETITION_TRANSACTION_ID": "test", "FIXED_TABLE_HEIGHT_MM": "650"}
            target.write_text(json.dumps({"schema": "competition_pick_target/v1", "valid": True,
                "trusted_for_venue_execution": True, "fixed_table_height_m": .650,
                "height_verification": "fixed_height_unverified"}))
            result = _validate("target", tmp_path, **common)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(json.loads(admission.read_text())["mode"], "fixed_height_unverified")
            target.write_text(json.dumps({"schema": "competition_pick_target/v1", "valid": True,
                "trusted_for_venue_execution": True, "height_verification": {"status": "healthy", "verified": True,
                "measured_table_height_mm": 676}}))
            self.assertNotEqual(_validate("target", tmp_path, **common).returncode, 0)
            target.write_text(json.dumps({"schema": "competition_pick_target/v1", "valid": True,
                "trusted_for_venue_execution": False, "fixed_table_height_m": .650,
                "height_verification": "fixed_height_unverified", "right_arm_sdk_target_m": [.4, .01, .65]}))
            forced = dict(common, FORCE_FIXED_TARGET="1")
            self.assertEqual(_validate("target", tmp_path, **forced).returncode, 0)
            self.assertNotEqual(_validate("target", tmp_path, **common).returncode, 0)
            grasp = tmp_path / "grasp_verification.json"; grasp.write_text('{"success": true, "navigation_permitted": false}')
            self.assertNotEqual(_validate("grasp", tmp_path).returncode, 0)
            grasp.write_text('{"success": true, "navigation_permitted": true}')
            self.assertEqual(_validate("grasp", tmp_path).returncode, 0)


if __name__ == "__main__":
    unittest.main()
