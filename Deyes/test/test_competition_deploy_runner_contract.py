"""ROS-free/static checks for the one-key competition deployment and runner."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[2]
RUNNER = ROOT / "scripts" / "race_onekey_try.sh"
DEPLOY = ROOT / "tools" / "deploy_competition_onekey.py"
PS1 = ROOT / "tools" / "deploy_competition_onekey.ps1"
GRASP_ADAPTER = ROOT / "scripts" / "competition_grasp_feedback_adapter.py"
PICK = ROOT / "scripts" / "pick_pen_degraded.py"
PLACE = ROOT / "scripts" / "place_pen_degraded.py"
SEND_GOAL = ROOT / "scripts" / "send_one_goal.py"
EXPECTED_ONNX = "8916cbf25949d6e8b03c01e6ca1c7871aeac0ad105c931f1dd9881cb5d5a4c4e"


def _engine_bindings() -> list[dict[str, object]]:
    return [
        {"index": 0, "name": "images", "io": "input", "shape": [1, 3, 416, 416], "dtype": "float32"},
        {"index": 1, "name": "output0", "io": "output", "shape": [1, 10647, 6], "dtype": "float16"},
    ]


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
    env.update({"COMPETITION_VALIDATE_ONLY": mode, "LOG_DIR": str(tmp_path), "PYTHON_BIN": sys.executable, **values})
    return subprocess.run([bash, str(RUNNER)], env=env, text=True, capture_output=True, check=False)


class CompetitionDeployRunnerTest(unittest.TestCase):
    def test_deployer_requires_an_explicit_robot_address(self) -> None:
        deploy_source = DEPLOY.read_text(encoding="utf-8")
        powershell_source = PS1.read_text(encoding="utf-8")
        self.assertNotIn("192.168.43.60", deploy_source)
        self.assertNotIn("192.168.43.60", powershell_source)
        self.assertIn("--host or ROBOT_IP is required", deploy_source)
        self.assertIn("Set -RobotIp or ROBOT_IP before deploying", powershell_source)

    def test_ros_free_runner_fixture_completes_goal4_and_place_after_target_and_grasp_failure(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            tmp = Path(directory)
            fake_bin = tmp / "bin"; fake_bin.mkdir()
            fake_scripts = tmp / "scripts"; fake_scripts.mkdir()
            install = tmp / "install"; install.mkdir()
            assets = tmp / "assets"; assets.mkdir()
            logs = tmp / "logs"
            events = tmp / "events.log"

            def executable(path: Path, source: str) -> None:
                path.write_text(source, encoding="utf-8")
                path.chmod(0o755)

            package_path = ROOT / "Deyes/src/deyes_stereo"
            setup = tmp / "setup.bash"
            setup.write_text("true\n", encoding="utf-8")
            (install / "setup.bash").write_text(
                f'export PYTHONPATH="{package_path}:${{PYTHONPATH:-}}"\n', encoding="utf-8"
            )
            executable(assets / "probe_opencv_cuda.sh", "#!/usr/bin/env bash\nexit 0\n")
            executable(fake_bin / "rostopic", "#!/usr/bin/env bash\necho /move_base/goal\n")
            executable(fake_bin / "roslaunch", "#!/usr/bin/env bash\nexec sleep 30\n")
            executable(fake_bin / "ros2", """#!/usr/bin/env bash
if [[ "$1" == launch || "$1" == run ]]; then
  if [[ "$1" == run && "${TARGET_NODE_EXIT_AFTER_REJECTION:-0}" == 1 ]]; then
    sleep .1
    exit 3
  fi
  trap 'exit 0' INT TERM
  while true; do sleep .1; done
fi
exit 2
""")
            executable(fake_scripts / "send_one_goal.py", """#!/usr/bin/env python3
import os,sys
with open(os.environ['EVENT_LOG'],'a') as stream: stream.write(sys.argv[1]+'\\n')
if os.environ.get('FAIL_GOAL') == sys.argv[1]: raise SystemExit(2)
""")
            executable(fake_scripts / "set_venue_head.py", """#!/usr/bin/env python3
import os
with open(os.environ['EVENT_LOG'],'a') as stream: stream.write('head\\n')
""")
            executable(fake_scripts / "competition_target_snapshot_adapter.py", """#!/usr/bin/env python3
import sys
print('target rejected: projector_not_usable_and_validated',file=sys.stderr)
raise SystemExit(3)
""")
            executable(fake_scripts / "competition_grasp_feedback_adapter.py", "#!/usr/bin/env python3\n")
            executable(fake_scripts / "prepare_competition_arms.py", """#!/usr/bin/env python3
import argparse,json,os
p=argparse.ArgumentParser(); p.add_argument('--result-json'); p.add_argument('--venue-profile'); a=p.parse_args()
with open(os.environ['EVENT_LOG'],'a') as stream: stream.write('arm_stow\\n')
json.dump({'schema':'competition_arm_stow_result/v1','success':True,'order':['left','right'],'commands_emitted':True},open(a.result_json,'w'))
""")
            executable(fake_scripts / "pick_pen_degraded.py", """#!/usr/bin/env python3
import argparse,json,os
p=argparse.ArgumentParser(); p.add_argument('--result-json'); p.add_argument('--showcase-target-json'); p.add_argument('--target-json'); p.add_argument('--feedback-adapter'); p.add_argument('--feedback-json'); p.add_argument('--x-mm'); p.add_argument('--y-mm'); p.add_argument('--venue-profile'); a=p.parse_args()
with open(os.environ['EVENT_LOG'],'a') as stream: stream.write('pick_transport\\n')
verified=bool(a.target_json)
json.dump({'schema':'competition_grasp_verification/v1','success':verified,'navigation_permitted':verified,'motion_completed':True,'transport_pose_reached':True,'hardware_ok':True,'object_grasp_verified':verified,'verification_failure_class':None if verified else 'object_absent','reason':'ok' if verified else 'grasp_not_verified','commands_emitted':True},open(a.result_json,'w'))
raise SystemExit(0 if verified else 2)
""")
            executable(fake_scripts / "place_pen_degraded.py", """#!/usr/bin/env python3
import argparse,json,os
p=argparse.ArgumentParser(); p.add_argument('--result-json'); p.add_argument('--venue-profile'); p.add_argument('--object-state'); p.add_argument('--showcase-mode',action='store_true'); a=p.parse_args()
assert a.object_state in ('verified','unverified')
with open(os.environ['EVENT_LOG'],'a') as stream: stream.write('place\\n')
json.dump({'schema':'competition_place_execution/v1','success':True,'motion_completed':True,'object_state':a.object_state,'object_delivery_verified':a.object_state=='verified','showcase_mode':a.showcase_mode,'commands_emitted':True},open(a.result_json,'w'))
""")

            engine = tmp / "model.engine"; engine.write_bytes(b"fixture-engine")
            manifest = tmp / "model.engine.manifest.json"
            manifest.write_text(json.dumps({
                "onnx_sha256": EXPECTED_ONNX,
                "engine_sha256": hashlib.sha256(engine.read_bytes()).hexdigest(),
                "tensorrt_version": "fixture", "cuda_version": "fixture",
                "input_shape": [1, 3, 416, 416], "output_layout": "yolov5:[1,N,5+C]",
                "precision": "fp16", "bindings": _engine_bindings(),
            }), encoding="utf-8")
            env = os.environ.copy()
            env.update({
                "PATH": str(fake_bin) + os.pathsep + env["PATH"],
                "PYTHONPATH": str(package_path), "PYTHON_BIN": sys.executable,
                "RAC_SCRIPTS": str(fake_scripts), "DEYES_INSTALL": str(install),
                "DEYES_ASSETS": str(assets), "DEYES_OPENCV_PREFIX": str(tmp / "opencv"),
                "DEYES_CALIB_PATH": str(ROOT / "Deyes/config/camera/venue_20260827_quick_stereo.yaml"),
                "DEYES_PROJECTOR_PATH": str(ROOT / "Deyes/config/camera/venue_20260827_touch_projector.yaml"),
                "DEYES_DETECTOR_CONFIG": str(ROOT / "Deyes/config/stereo/competition_fixed_scene.yaml"),
                "COMPETITION_SITE_METADATA": str(ROOT / "Deyes/config/stereo/competition_venue_65cm.yaml"),
                "DEYES_ENGINE_PATH": str(engine), "DEYES_ENGINE_MANIFEST": str(manifest),
                "ROS1_SETUP": str(setup), "ROS2_SETUP": str(setup), "MERCURY_ROS1_SETUP": str(setup),
                "LOG_DIR": str(logs), "EVENT_LOG": str(events),
                "COMPETITION_TRANSACTION_ID": "fixture-showcase", "VISION_STARTUP_SEC": "0",
                "TARGET_SUBSCRIBER_READY_SEC": "0", "TARGET_STARTUP_SEC": "0",
                "COMPETITION_SHOWCASE_CONTINUE": "1", "PROCESS_INT_POLLS": "1",
                "PROCESS_TERM_POLLS": "1",
            })
            completed = subprocess.run(
                ["bash", str(RUNNER)], env=env, text=True, capture_output=True,
                check=False, timeout=20,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn("SHOWCASE COMPLETE", completed.stdout)
            self.assertIn("COMPETITION SUCCESS: false", completed.stdout)
            result = json.loads((logs / "transaction_result.json").read_text(encoding="utf-8"))
            self.assertIs(result["competition_success"], False)
            self.assertIs(result["showcase_complete"], True)
            self.assertEqual(result["target_source"], "fixed_marker_showcase")
            self.assertEqual(events.read_text(encoding="utf-8").splitlines(), [
                "arm_stow", "goal3_right", "head", "pick_transport", "goal4_back", "place",
            ])

            executable(fake_scripts / "competition_target_snapshot_adapter.py", """#!/usr/bin/env python3
import sys
print('target rejected: detection_count_must_be_exactly_one',file=sys.stderr)
raise SystemExit(3)
""")
            terminal_rejection_logs = tmp / "terminal-rejection-showcase-logs"
            events.write_text("", encoding="utf-8")
            env.update({
                "LOG_DIR": str(terminal_rejection_logs),
                "COMPETITION_TRANSACTION_ID": "fixture-terminal-rejection-showcase",
                "COMPETITION_SHOWCASE_CONTINUE": "1",
                "TARGET_NODE_EXIT_AFTER_REJECTION": "1",
                "TARGET_STARTUP_SEC": ".3",
            })
            terminal_rejection = subprocess.run(
                ["bash", str(RUNNER)], env=env, text=True, capture_output=True,
                check=False, timeout=20,
            )
            self.assertEqual(terminal_rejection.returncode, 0, terminal_rejection.stderr)
            terminal_result = json.loads(
                (terminal_rejection_logs / "transaction_result.json").read_text(encoding="utf-8")
            )
            self.assertIs(terminal_result["competition_success"], False)
            self.assertIs(terminal_result["showcase_complete"], True)
            self.assertEqual(events.read_text(encoding="utf-8").splitlines(), [
                "arm_stow", "goal3_right", "head", "pick_transport", "goal4_back", "place",
            ])
            env.pop("TARGET_NODE_EXIT_AFTER_REJECTION")
            env["TARGET_STARTUP_SEC"] = "0"

            strict_logs = tmp / "strict-logs"
            events.write_text("", encoding="utf-8")
            env.update({
                "LOG_DIR": str(strict_logs),
                "COMPETITION_TRANSACTION_ID": "fixture-strict",
                "COMPETITION_SHOWCASE_CONTINUE": "0",
            })
            strict = subprocess.run(
                ["bash", str(RUNNER)], env=env, text=True, capture_output=True,
                check=False, timeout=20,
            )
            self.assertNotEqual(strict.returncode, 0)
            self.assertNotIn("SHOWCASE COMPLETE", strict.stdout)
            strict_result = json.loads(
                (strict_logs / "transaction_result.json").read_text(encoding="utf-8")
            )
            self.assertIs(strict_result["showcase_complete"], False)
            self.assertIn("competition_target_failed", strict_result["hard_stop_reason"])
            self.assertEqual(events.read_text(encoding="utf-8").splitlines(), [
                "arm_stow", "goal3_right", "head",
            ])

            executable(fake_scripts / "competition_target_snapshot_adapter.py", """#!/usr/bin/env python3
import sys
print('target JSON malformed',file=sys.stderr)
raise SystemExit(4)
""")
            config_logs = tmp / "target-config-hard-stop-logs"
            events.write_text("", encoding="utf-8")
            env.update({
                "LOG_DIR": str(config_logs),
                "COMPETITION_TRANSACTION_ID": "fixture-target-config-hard-stop",
                "COMPETITION_SHOWCASE_CONTINUE": "1",
            })
            config_failure = subprocess.run(
                ["bash", str(RUNNER)], env=env, text=True, capture_output=True,
                check=False, timeout=20,
            )
            self.assertNotEqual(config_failure.returncode, 0)
            config_result = json.loads(
                (config_logs / "transaction_result.json").read_text(encoding="utf-8")
            )
            self.assertIs(config_result["showcase_complete"], False)
            self.assertEqual(events.read_text(encoding="utf-8").splitlines(), [
                "arm_stow", "goal3_right", "head",
            ])

            executable(fake_scripts / "competition_target_snapshot_adapter.py", """#!/usr/bin/env python3
import argparse,json
p=argparse.ArgumentParser(); p.add_argument('--topic'); p.add_argument('--output'); p.add_argument('--timeout'); a=p.parse_args()
json.dump({'schema':'competition_pick_target/v1','valid':True,'trusted_for_venue_execution':True,'fixed_table_height_m':0.650,'height_verification':'fixed_height_unverified','selection_source':'axis_midpoint','projector_usable_and_validated':True,'degraded':False,'execution_allowed':True,'commands_emitted':False,'right_arm_sdk_target_m':[0.4,0.01,0.135],'orientation_deg':[179.99,-12,0]},open(a.output,'w'))
""")
            success_logs = tmp / "verified-success-logs"
            events.write_text("", encoding="utf-8")
            env.update({
                "LOG_DIR": str(success_logs),
                "COMPETITION_TRANSACTION_ID": "fixture-verified-success",
                "COMPETITION_SHOWCASE_CONTINUE": "1",
            })
            verified = subprocess.run(
                ["bash", str(RUNNER)], env=env, text=True, capture_output=True,
                check=False, timeout=20,
            )
            self.assertEqual(verified.returncode, 0, verified.stderr)
            verified_result = json.loads(
                (success_logs / "transaction_result.json").read_text(encoding="utf-8")
            )
            self.assertIs(verified_result["competition_success"], True)
            self.assertIs(verified_result["showcase_complete"], True)
            self.assertIs(verified_result["object_grasp_verified"], True)
            self.assertEqual(verified_result["target_source"], "live_competition_target")
            self.assertEqual(events.read_text(encoding="utf-8").splitlines(), [
                "arm_stow", "goal3_right", "head", "pick_transport", "goal4_back", "place",
            ])

            for failed_goal, expected_events in (
                ("goal3_right", ["arm_stow", "goal3_right"]),
                ("goal4_back", ["arm_stow", "goal3_right", "head", "pick_transport", "goal4_back"]),
            ):
                executable(fake_scripts / "competition_target_snapshot_adapter.py", """#!/usr/bin/env python3
import sys
print('target rejected: projector_not_usable_and_validated',file=sys.stderr)
raise SystemExit(3)
""")
                navigation_logs = tmp / f"{failed_goal}-hard-stop-logs"
                events.write_text("", encoding="utf-8")
                env.update({
                    "LOG_DIR": str(navigation_logs),
                    "COMPETITION_TRANSACTION_ID": f"fixture-{failed_goal}-hard-stop",
                    "COMPETITION_SHOWCASE_CONTINUE": "1",
                    "FAIL_GOAL": failed_goal,
                })
                navigation_failure = subprocess.run(
                    ["bash", str(RUNNER)], env=env, text=True, capture_output=True,
                    check=False, timeout=20,
                )
                self.assertNotEqual(navigation_failure.returncode, 0)
                navigation_result = json.loads(
                    (navigation_logs / "transaction_result.json").read_text(encoding="utf-8")
                )
                self.assertIs(navigation_result["showcase_complete"], False)
                self.assertIn("navigation", navigation_result["hard_stop_reason"])
                self.assertEqual(
                    events.read_text(encoding="utf-8").splitlines(), expected_events
                )
            env.pop("FAIL_GOAL")

            executable(fake_scripts / "competition_target_snapshot_adapter.py", """#!/usr/bin/env python3
import sys
print('target rejected: table_height_deviation_exceeds_25mm',file=sys.stderr)
raise SystemExit(3)
""")
            plane_logs = tmp / "plane-hard-stop-logs"
            events.write_text("", encoding="utf-8")
            env.update({
                "LOG_DIR": str(plane_logs),
                "COMPETITION_TRANSACTION_ID": "fixture-plane-hard-stop",
                "COMPETITION_SHOWCASE_CONTINUE": "1",
            })
            plane_failure = subprocess.run(
                ["bash", str(RUNNER)], env=env, text=True, capture_output=True,
                check=False, timeout=20,
            )
            self.assertNotEqual(plane_failure.returncode, 0)
            plane_result = json.loads(
                (plane_logs / "transaction_result.json").read_text(encoding="utf-8")
            )
            self.assertIs(plane_result["showcase_complete"], False)
            self.assertIn("showcase target contract failed", plane_result["hard_stop_reason"])
            self.assertEqual(events.read_text(encoding="utf-8").splitlines(), [
                "arm_stow", "goal3_right", "head",
            ])

    def test_run_defaults_to_showcase_continuation_with_explicit_strict_override(self) -> None:
        runner = RUNNER.read_text(encoding="utf-8")
        self.assertIn(
            'COMPETITION_SHOWCASE_CONTINUE="${COMPETITION_SHOWCASE_CONTINUE:-1}"',
            runner,
        )
        self.assertIn("bool01 COMPETITION_SHOWCASE_CONTINUE", runner)

        ps1 = PS1.read_text(encoding="utf-8")
        self.assertIn("[switch]$StrictResultGates", ps1)
        self.assertIn('"--strict-result-gates"', ps1)

        deploy = DEPLOY.read_text(encoding="utf-8")
        self.assertIn('parser.add_argument("--strict-result-gates", action="store_true")', deploy)
        self.assertIn('COMPETITION_SHOWCASE_CONTINUE=', deploy)

    def test_runner_has_truthful_showcase_fallback_and_dual_terminal_status(self) -> None:
        content = RUNNER.read_text(encoding="utf-8")
        for token in (
            "competition_showcase_target/v1",
            "fixed_marker_showcase",
            "decide_pick_attempt",
            "retry_snapshot",
            "validate_retry_snapshot",
            "PICK_ATTEMPT_COUNT=2",
            "competition_transaction_result/v1",
            '"competition_success"',
            '"showcase_complete"',
            "--showcase-target-json",
            "--object-state",
            "SHOWCASE COMPLETE",
            "COMPETITION SUCCESS: false",
        ):
            self.assertIn(token, content)
        self.assertNotIn('result["navigation_permitted"] = True', content)
        self.assertNotIn('data["navigation_permitted"] = True', content)

    def test_deploy_inventory_contains_exact_packages_scripts_config_and_pinned_onnx(self) -> None:
        deploy = _load_deploy(); remotes = {str(remote) for _, remote in deploy.local_files()}
        for package in ("deyes_interfaces", "deyes_capture_cpp", "deyes_stereo", "deyes_bringup"):
            self.assertTrue(any(path.startswith(f"ws/src/Deyes/src/{package}/") for path in remotes))
        self.assertTrue(any(path.startswith("ws/src/Deyes/config/") for path in remotes))
        self.assertIn("scripts/race_onekey_try.sh", remotes)
        self.assertIn("scripts/competition_target_snapshot_adapter.py", remotes)
        self.assertIn("scripts/competition_grasp_feedback_adapter.py", remotes)
        onnx = ROOT / "Deyes" / "models" / "pen" / "pen_student_01875_416_v1.onnx"
        self.assertEqual(hashlib.sha256(onnx.read_bytes()).hexdigest(), EXPECTED_ONNX)
        self.assertIn("models/pen/pen_student_01875_416_v1.onnx", remotes)

    def test_deploy_command_is_non_destructive_and_has_live_acceptance_gates(self) -> None:
        content = DEPLOY.read_text(encoding="utf-8")
        self.assertNotIn("rm -rf", content); self.assertNotIn("ALLOW_DEGRADED", content)
        for token in ("trtexec", "--fp16", "engine_sha256", "tensorrt_version", "cuda_version",
                      "ldd", "depth topic below 12Hz", "pair skew missing or above 10ms", "enable_pen_features:=true",
                      "venue_20260827_touch_projector.yaml"):
            self.assertIn(token, content)
        self.assertIn("pkill -INT -f '[i]mx219_stereo_capture_node", content)
        self.assertIn("os.path.commonpath((prefix,resolved)) != prefix", content)
        self.assertIn("CUDA node leaked outside isolated OpenCV", content)
        self.assertNotIn("grep -E '(/usr/lib|/lib/)'", content)
        self.assertIn("m['tensorrt_version']==os.environ['TRT_VERSION']", content)
        self.assertIn("m['cuda_version']==os.environ['CUDA_VERSION']", content)
        self.assertIn("unable to determine CUDA version", content)
        self.assertIn("unsupported deployment architecture", content)
        for topic in ("/x1/stereo/left/camera_info_rect", "/x1/detection/boxes",
                      "/x1/detection/boxes_status", "/x1/ground/plane",
                      "/x1/ground/plane_status", "/x1/detection/pen_features",
                      "/x1/detection/pen_features_status"):
            self.assertIn(topic, content)
        for token in ("bindings", "num_io_tensors", "get_tensor_name", "num_bindings",
                      "get_binding_name", "runtime_bindings.json", "vision_contract.json"):
            self.assertIn(token, content)
        bash = _bash()
        if bash:
            deploy = _load_deploy()
            command = deploy.remote_deploy_command(argparse.Namespace(
                remote_home="/home/elephant", stop_existing=True, vision_dry_run_seconds=1),
                deploy.PurePosixPath("/home/elephant/deyes_competition_deploy"))
            checked = subprocess.run([bash, "-n"], input=command, text=True, capture_output=True, check=False)
            self.assertEqual(checked.returncode, 0, checked.stderr)
            heredocs = re.findall(r"<<'PY'[^\n]*\n(.*?)\nPY(?:\n|$)", command, re.DOTALL)
            self.assertGreaterEqual(len(heredocs), 6)
            for index, source in enumerate(heredocs):
                compile(source, f"deploy-heredoc-{index}.py", "exec")
            self.assertIn("vision contract topics missing", command)

    def test_opencv_private_prefix_gate_accepts_private_and_rejects_system_paths(self) -> None:
        deploy = _load_deploy()
        prefix = "/home/elephant/opencv-4.8.0-cuda"
        accepted = deploy.validate_opencv_ldd_paths([
            prefix + "/lib/libopencv_core.so.4.8",
            prefix + "/lib/libopencv_cudastereo.so.4.8",
        ], prefix)
        self.assertEqual(len(accepted), 2)
        with self.assertRaisesRegex(ValueError, "outside isolated OpenCV prefix"):
            deploy.validate_opencv_ldd_paths([
                prefix + "/lib/libopencv_core.so.4.8",
                "/usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.5",
            ], prefix)
        with self.assertRaisesRegex(ValueError, "no OpenCV libraries"):
            deploy.validate_opencv_ldd_paths([], prefix)

        command = deploy.remote_deploy_command(
            argparse.Namespace(
                stop_existing=False,
                vision_dry_run_seconds=1,
                remote_home="/home/elephant",
            ),
            deploy.PurePosixPath("/home/elephant/deyes_competition_deploy"),
        )
        self.assertIn("awk '/libopencv_/", command)
        self.assertNotIn("awk '/libopencv_(core|cuda)/", command)

    def test_runner_order_permissions_and_powershell_flags_are_fail_closed(self) -> None:
        content = RUNNER.read_text(encoding="utf-8")
        order = [content.index(token) for token in ("run_step nav_goal3", "run_step set_head", 'capture_target_once "$TARGET_JSON"',
            "run_step pick", "decide_pick_attempt", "stop_vision\n\nsource_ros1", "run_step nav_goal4", "run_step place")]
        self.assertEqual(order, sorted(order))
        for token in ('FIXED_TABLE_HEIGHT_MM="${FIXED_TABLE_HEIGHT_MM:-650}"', 'ALLOW_BBOX_CENTER="${ALLOW_BBOX_CENTER:-0}"',
                      'ALLOW_FIXED_XY_FALLBACK="${ALLOW_FIXED_XY_FALLBACK:-0}"', 'FORCE_FIXED_TARGET="${FORCE_FIXED_TARGET:-0}"'):
            self.assertIn(token, content)
        self.assertNotIn("ALLOW_DEGRADED", content)
        self.assertIn("PICK_ATTEMPT_COUNT=2", content)
        self.assertIn("ATTEMPT_NUMBER=2", content)
        self.assertNotIn("ATTEMPT_NUMBER=3", content)
        self.assertIn("competition_venue_65cm.yaml", content)
        self.assertIn("venue_20260827_touch_projector.yaml", content)
        self.assertIn('-p projector_path:="$PROJECTOR_PATH"', content)
        self.assertIn("--x-mm", content); self.assertIn("--venue-profile", content); self.assertIn("--result-json", content)
        self.assertIn("--target-json", content); self.assertIn("--feedback-adapter", content)
        self.assertIn('--result-json "$PLACE_JSON"', content)
        self.assertIn('trace "degraded" "active"', content)
        self.assertIn("export FORCE_FIXED_TARGET", content)
        self.assertIn("terminate_pid", content); self.assertIn("kill -KILL", content)
        self.assertIn("trap cleanup EXIT", content); self.assertIn("trap 'on_signal INT 130' INT", content)
        self.assertIn('data.get("success") is not True', content)
        self.assertIn('data.get("navigation_permitted") is not True', content)
        self.assertIn("resolve_deyes_python_site", content)
        self.assertIn("import pathlib, deyes_stereo", content)
        self.assertIn('export PYTHONPATH="$deyes_site${PYTHONPATH:+:$PYTHONPATH}"', content)
        adapter = (ROOT / "tools" / "competition_target_snapshot_adapter.py").read_text(encoding="utf-8")
        self.assertIn("waiting_for_exact_stamp_projector_adapter", adapter)
        self.assertIn("competition_pick_target/v1", adapter)
        ps1 = PS1.read_text(encoding="utf-8")
        self.assertIn("AllowFixedXyFallback", ps1); self.assertIn("ForceFixedTarget", ps1); self.assertNotIn("ALLOW_DEGRADED", ps1)
        self.assertIn("--result-json", PLACE.read_text(encoding="utf-8"))
        self.assertNotIn("retry", SEND_GOAL.read_text(encoding="utf-8").lower())
        self.assertIn("qos_profile_sensor_data", GRASP_ADAPTER.read_text(encoding="utf-8"))
        for script in (PICK, PLACE):
            motion_source = script.read_text(encoding="utf-8")
            self.assertLess(
                motion_source.index("profile.require_hardware_admission()"),
                motion_source.index("from pymycobot import Mercury"),
            )

    def test_default_venue_profile_blocks_live_pick_place_but_keeps_dry_run_plans(self) -> None:
        profile = ROOT / "Deyes/config/stereo/competition_venue_65cm.yaml"
        env = os.environ.copy()
        package_path = str(ROOT / "Deyes/src/deyes_stereo")
        env["PYTHONPATH"] = package_path + os.pathsep + env.get("PYTHONPATH", "")
        with tempfile.TemporaryDirectory() as directory:
            tmp_path = Path(directory)
            for script in (PICK, PLACE):
                dry = subprocess.run(
                    [sys.executable, str(script), "--venue-profile", str(profile), "--dry-run"],
                    env=env, text=True, capture_output=True, check=False,
                )
                self.assertEqual(dry.returncode, 0, dry.stderr)
                plan = json.loads(dry.stdout)
                self.assertIs(plan["commands_emitted"], False)
                self.assertIs(plan["kinematics_validated"], True)
                self.assertIs(plan["collision_clearance_validated"], False)
                self.assertIs(plan["transport_validated"], False)

            target = tmp_path / "target.json"
            target.write_text(json.dumps({
                "schema": "competition_pick_target/v1",
                "right_arm_sdk_target_m": [0.4, 0.01, 0.135],
            }), encoding="utf-8")
            pick_result = tmp_path / "pick.json"
            blocked_pick = subprocess.run(
                [sys.executable, str(PICK), "--venue-profile", str(profile),
                 "--result-json", str(pick_result), "--target-json", str(target),
                 "--feedback-json", str(tmp_path / "feedback.json"),
                 "--feedback-adapter", str(GRASP_ADAPTER)],
                env=env, text=True, capture_output=True, check=False,
            )
            self.assertEqual(blocked_pick.returncode, 2)
            self.assertIn("transport_motion_not_fully_validated", pick_result.read_text(encoding="utf-8"))

            place_result = tmp_path / "place.json"
            blocked_place = subprocess.run(
                [sys.executable, str(PLACE), "--venue-profile", str(profile),
                 "--result-json", str(place_result)],
                env=env, text=True, capture_output=True, check=False,
            )
            self.assertEqual(blocked_place.returncode, 2)
            self.assertIn("transport_motion_not_fully_validated", place_result.read_text(encoding="utf-8"))

            site = yaml.safe_load(profile.read_text(encoding="utf-8"))
            for missing_field in ("collision_clearance_validated", "tcp_vertical_clearance_conservative_mm"):
                transport = site["transport"]
                transport.update({"transport_validated": True, "kinematics_validated": True,
                                  "collision_clearance_validated": True,
                                  "tcp_vertical_clearance_conservative_mm": 10.0})
                transport.pop(missing_field)
                missing_profile = tmp_path / f"missing-{missing_field}.yaml"
                missing_profile.write_text(yaml.safe_dump(site), encoding="utf-8")
                missing_result = tmp_path / f"missing-{missing_field}.json"
                blocked = subprocess.run(
                    [sys.executable, str(PLACE), "--venue-profile", str(missing_profile),
                     "--result-json", str(missing_result)],
                    env=env, text=True, capture_output=True, check=False,
                )
                self.assertEqual(blocked.returncode, 2)
                self.assertIn("transport_motion_not_fully_validated", missing_result.read_text(encoding="utf-8"))

    def test_engine_sidecar_fault_matrix(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            tmp_path = Path(directory); engine = tmp_path / "model.engine"; engine.write_bytes(b"device-local-engine")
            manifest = tmp_path / "model.engine.manifest.json"
            manifest.write_text(json.dumps({"onnx_sha256": EXPECTED_ONNX, "engine_sha256": hashlib.sha256(engine.read_bytes()).hexdigest(),
                "tensorrt_version": "8", "cuda_version": "11.4", "input_shape": [1, 3, 416, 416],
                "output_layout": "yolov5:[1,N,5+C]", "precision": "fp16",
                "bindings": _engine_bindings()}), encoding="utf-8")
            result = _validate("engine", tmp_path, DEYES_ENGINE_PATH=str(engine), DEYES_ENGINE_MANIFEST=str(manifest))
            self.assertEqual(result.returncode, 0, result.stderr)
            payload = json.loads((tmp_path / "engine_validation.json").read_text(encoding="utf-8"))
            self.assertEqual(payload["bindings"], _engine_bindings())
            bad_bindings = json.loads(manifest.read_text(encoding="utf-8"))
            bad_bindings["bindings"][1]["shape"] = [1, 10647, 85]
            manifest.write_text(json.dumps(bad_bindings), encoding="utf-8")
            result = _validate("engine", tmp_path, DEYES_ENGINE_PATH=str(engine), DEYES_ENGINE_MANIFEST=str(manifest))
            self.assertNotEqual(result.returncode, 0); self.assertIn("output binding", result.stderr)
            bad_bindings.pop("bindings")
            manifest.write_text(json.dumps(bad_bindings), encoding="utf-8")
            result = _validate("engine", tmp_path, DEYES_ENGINE_PATH=str(engine), DEYES_ENGINE_MANIFEST=str(manifest))
            self.assertNotEqual(result.returncode, 0); self.assertIn("engine manifest missing: bindings", result.stderr)
            bad_bindings["bindings"] = _engine_bindings()
            manifest.write_text(json.dumps(bad_bindings), encoding="utf-8")
            engine.write_bytes(b"tampered")
            result = _validate("engine", tmp_path, DEYES_ENGINE_PATH=str(engine), DEYES_ENGINE_MANIFEST=str(manifest))
            self.assertNotEqual(result.returncode, 0); self.assertIn("engine SHA", result.stderr)

    def test_ground_plane_fault_matrix_and_grasp_gate(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            tmp_path = Path(directory); target = tmp_path / "target.json"; admission = tmp_path / "admission.json"
            common = {"COMPETITION_TRANSACTION_ID": "test", "FIXED_TABLE_HEIGHT_MM": "650"}
            target.write_text(json.dumps({"schema": "competition_pick_target/v1", "valid": True,
                "trusted_for_venue_execution": True, "fixed_table_height_m": .650,
                "height_verification": "fixed_height_unverified", "selection_source": "axis_midpoint",
                "projector_usable_and_validated": True, "degraded": False, "execution_allowed": True,
                "commands_emitted": False, "right_arm_sdk_target_m": [.4, .01, .135],
                "orientation_deg": [179.99, -12, 0]}))
            result = _validate("target", tmp_path, **common)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(json.loads(admission.read_text())["mode"], "fixed_height_unverified")
            target.write_text(json.dumps({"schema": "competition_pick_target/v1", "valid": True,
                "trusted_for_venue_execution": True, "height_verification": {"status": "healthy", "verified": True,
                "measured_table_height_mm": 676}, "selection_source": "axis_midpoint",
                "projector_usable_and_validated": True, "degraded": False, "execution_allowed": True,
                "commands_emitted": False, "right_arm_sdk_target_m": [.4, .01, .135],
                "orientation_deg": [179.99, -12, 0]}))
            self.assertNotEqual(_validate("target", tmp_path, **common).returncode, 0)
            target.write_text(json.dumps({"schema": "competition_pick_target/v1", "valid": True,
                "trusted_for_venue_execution": False, "fixed_table_height_m": .650,
                "projector_usable_and_validated": False, "height_verification": "fixed_height_unverified",
                "selection_source": "fixed_xy_fallback", "commands_emitted": False,
                "degraded": True, "degraded_mode": "forced_fixed_xy_marker", "execution_allowed": True,
                "force_fixed_target": True, "manual_action_required": "place pen at [400,10]mm marker",
                "right_arm_sdk_target_m": [.4, .01, .135], "orientation_deg": [179.99, -12, 0]}))
            forced = dict(common, FORCE_FIXED_TARGET="1", ALLOW_FIXED_XY_FALLBACK="1")
            self.assertEqual(_validate("target", tmp_path, **forced).returncode, 0)
            self.assertEqual(json.loads(admission.read_text())["mode"], "fixed_xy_degraded")
            self.assertNotEqual(_validate("target", tmp_path, **common).returncode, 0)
            target.write_text(json.dumps({"schema": "competition_pick_target/v1", "valid": True,
                "trusted_for_venue_execution": False, "fixed_table_height_m": .650,
                "height_verification": "fixed_height_unverified", "selection_source": "axis_midpoint",
                "projector_usable_and_validated": False, "degraded": True, "execution_allowed": True,
                "force_fixed_target": True, "degraded_mode": "forced_fixed_xy_marker",
                "manual_action_required": "place pen at [400,10]mm marker",
                "commands_emitted": False, "right_arm_sdk_target_m": [.4, .01, .135],
                "orientation_deg": [179.99, -12, 0]}))
            self.assertNotEqual(_validate("target", tmp_path, **forced).returncode, 0)
            target.write_text('{"accepted": true}')
            self.assertNotEqual(_validate("target", tmp_path, **forced).returncode, 0)
            target.write_text(json.dumps({"schema": "competition_pick_target/v1", "valid": True,
                "trusted_for_venue_execution": True, "fixed_table_height_m": .650,
                "height_verification": "fixed_height_unverified", "selection_source": "axis_midpoint",
                "projector_usable_and_validated": True, "degraded": False, "execution_allowed": True,
                "commands_emitted": False, "right_arm_sdk_target_m": [float("nan"), .01, .135],
                "orientation_deg": [179.99, -12, 0]}))
            self.assertNotEqual(_validate("target", tmp_path, **common).returncode, 0)
            grasp = tmp_path / "grasp_verification.json"; grasp.write_text('{"success": true, "navigation_permitted": false}')
            self.assertNotEqual(_validate("grasp", tmp_path).returncode, 0)
            grasp.write_text(json.dumps({"schema": "competition_grasp_verification/v1", "success": True,
                "navigation_permitted": True, "single_attempt_latched": True,
                "condition_b_roi_clear_3_and_feedback_delta_5": True,
                "feedback_evidence": {"schema": "competition_grasp_feedback/v1", "live": True,
                    "source": "live_ros2_and_mercury_feedback", "roi_pen_last3": [False, False, False],
                    "detector_frames_last3_ambiguous": [False, False, False],
                    "gripper_feedback_delta": 6.0}}))
            self.assertEqual(_validate("grasp", tmp_path).returncode, 0)
            grasp_payload = json.loads(grasp.read_text(encoding="utf-8"))
            del grasp_payload["feedback_evidence"]["detector_frames_last3_ambiguous"]
            grasp.write_text(json.dumps(grasp_payload), encoding="utf-8")
            self.assertNotEqual(_validate("grasp", tmp_path).returncode, 0)
            place = tmp_path / "place.json"; place.write_text('{"success": false}')
            self.assertNotEqual(_validate("place", tmp_path).returncode, 0)
            place.write_text(json.dumps({"schema": "competition_place_execution/v1",
                "success": True, "motion_completed": True, "object_state": "verified",
                "object_delivery_verified": True, "showcase_mode": False,
                "commands_emitted": True}))
            self.assertEqual(_validate("place", tmp_path).returncode, 0)
            place.write_text(json.dumps({"schema": "competition_place_execution/v1",
                "success": True, "motion_completed": True, "object_state": "unverified",
                "object_delivery_verified": False, "showcase_mode": True,
                "commands_emitted": True}))
            self.assertEqual(_validate("place", tmp_path).returncode, 0)
            place.write_text(json.dumps({"schema": "competition_place_execution/v1",
                "success": True, "motion_completed": True, "object_state": "unverified",
                "object_delivery_verified": True, "showcase_mode": True,
                "commands_emitted": True}))
            self.assertNotEqual(_validate("place", tmp_path).returncode, 0)
            place.write_text(json.dumps({"schema": "competition_place_execution/v1",
                "success": True, "motion_completed": True, "object_state": "unverified",
                "object_delivery_verified": False, "showcase_mode": True,
                "commands_emitted": False}))
            self.assertNotEqual(_validate("place", tmp_path).returncode, 0)

    def test_grasp_feedback_adapter_replays_real_payload_shape_and_requires_three_clear_frames(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            tmp_path = Path(directory)
            target = tmp_path / "target.json"
            target.write_text(json.dumps({"schema": "competition_pick_target/v1", "valid": True,
                                          "pixel_uv": [200, 120]}), encoding="utf-8")
            fixture = tmp_path / "boxes.jsonl"
            frames = [
                {"stamp_sec": 1, "stamp_nanosec": index, "model_id": "pen-yolov5-student-01875-416-v1",
                 "complete": True, "ambiguous": False, "rejection_reason": "",
                 "auto_grasp_permitted": bool(detections),
                 "detection_count": len(detections), "observed_detection_count": len(detections),
                 "detections": detections}
                for index, detections in enumerate((
                    [{"bbox_xyxy": [190, 110, 220, 130]}], [], [], []
                ))
            ]
            fixture.write_text("".join(json.dumps(frame) + "\n" for frame in frames), encoding="utf-8")
            output = tmp_path / "feedback.json"
            result = subprocess.run([sys.executable, str(GRASP_ADAPTER), "--target-json", str(target),
                                     "--output", str(output), "--detections-fixture", str(fixture),
                                     "--empty-closed-feedback", "10", "--gripper-feedback", "16"],
                                    text=True, capture_output=True, check=False)
            self.assertEqual(result.returncode, 0, result.stderr)
            data = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(data["schema"], "competition_grasp_feedback/v1")
            self.assertEqual(data["roi_pen_last3"], [False, False, False])
            self.assertEqual(data["gripper_feedback_delta"], 6.0)
            self.assertEqual(data["detector_frames_last3_ambiguous"], [False, False, False])

            target.write_text(json.dumps({"schema": "competition_pick_target/v1", "valid": True,
                                          "pixel_uv": None}), encoding="utf-8")
            missing_roi = subprocess.run([sys.executable, str(GRASP_ADAPTER), "--target-json", str(target),
                                          "--output", str(output), "--detections-fixture", str(fixture),
                                          "--empty-closed-feedback", "10", "--gripper-feedback", "16"],
                                         text=True, capture_output=True, check=False)
            self.assertNotEqual(missing_roi.returncode, 0)
            self.assertIn("target pixel_uv missing", missing_roi.stderr)

            target.write_text(json.dumps({"schema": "competition_pick_target/v1", "valid": True,
                                          "pixel_uv": [200, 120]}), encoding="utf-8")
            fixture.write_text("".join(json.dumps({"stamp_sec": 2, "stamp_nanosec": index,
                "model_id": "pen-yolov5-student-01875-416-v1", "complete": True,
                "ambiguous": False, "rejection_reason": "", "auto_grasp_permitted": True,
                "detection_count": 1, "observed_detection_count": 1,
                "detections": ["corrupt"]}) + "\n"
                for index in range(3)), encoding="utf-8")
            corrupt = subprocess.run([sys.executable, str(GRASP_ADAPTER), "--target-json", str(target),
                                      "--output", str(output), "--detections-fixture", str(fixture),
                                      "--empty-closed-feedback", "10", "--gripper-feedback", "16"],
                                     text=True, capture_output=True, check=False)
            self.assertNotEqual(corrupt.returncode, 0)
            self.assertIn("detection item must be an object", corrupt.stderr)

    def test_grasp_feedback_adapter_rejects_ambiguous_rejected_or_count_inconsistent_frames(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            tmp_path = Path(directory)
            target = tmp_path / "target.json"
            target.write_text(json.dumps({"schema": "competition_pick_target/v1", "valid": True,
                                          "pixel_uv": [200, 120]}), encoding="utf-8")
            base = {"stamp_sec": 1, "stamp_nanosec": 1, "model_id": "pen-yolov5-student-01875-416-v1",
                    "complete": True, "ambiguous": False, "rejection_reason": "",
                    "auto_grasp_permitted": False, "detection_count": 0,
                    "observed_detection_count": 0, "detections": []}
            cases = (
                ({**base, "ambiguous": True, "rejection_reason": "ambiguous_multi_target"}, "ambiguous"),
                ({**base, "rejection_reason": "detector_rejected"}, "rejection_reason"),
                ({**base, "detection_count": 1}, "detection_count"),
                ({**base, "observed_detection_count": 1}, "observed_detection_count"),
                ({key: value for key, value in base.items() if key != "complete"}, "complete"),
            )
            for index, (frame, reason) in enumerate(cases):
                frame["stamp_nanosec"] = index + 1
                fixture = tmp_path / f"bad-{index}.jsonl"
                fixture.write_text(json.dumps(frame) + "\n", encoding="utf-8")
                result = subprocess.run(
                    [sys.executable, str(GRASP_ADAPTER), "--target-json", str(target),
                     "--output", str(tmp_path / f"out-{index}.json"),
                     "--detections-fixture", str(fixture), "--empty-closed-feedback", "10",
                     "--gripper-feedback", "16"],
                    text=True, capture_output=True, check=False,
                )
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(reason, result.stderr)


if __name__ == "__main__":
    unittest.main()
