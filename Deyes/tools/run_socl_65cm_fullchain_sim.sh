#!/usr/bin/env bash
# Reproducible Socl_ous-only launch.  This never opens robot serial devices.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SIM_ROOT="${X1_SIM_ROOT:-/var/workspace/temp/x1-65cm-sim}"
SOURCE_ROOT="${X1_V5_SCENE_ROOT:-/var/workspace/docker/isaac/scenes/team_rak_finals_20260820_dual_arm_transfer_v5_low_tables}"
SOURCE_CONFIG="${SOURCE_ROOT}/scene_config.json"
SOURCE_USD="${SOURCE_ROOT}/outputs/team_rak_finals_20260820_dual_arm_transfer_v5_low_tables.usd"
OVERRIDE_USD="${SIM_ROOT}/artifacts/team_rak_v5_65cm_single_pen_override.usda"
OVERRIDE_MANIFEST="${SIM_ROOT}/artifacts/team_rak_v5_65cm_single_pen_override.manifest.json"
LOG_ROOT="${SIM_ROOT}/logs"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
FIXTURE_EVIDENCE="${LOG_ROOT}/fixture_fault_matrix_${RUN_ID}.json"

mkdir -p "${SIM_ROOT}/artifacts" "${LOG_ROOT}/ros-onekey"

python3 "${REPO_ROOT}/Deyes/tools/generate_competition_65cm_scene_override.py" \
  --source-config "${SOURCE_CONFIG}" \
  --source-usd "${SOURCE_USD}" \
  --output-usda "${OVERRIDE_USD}" \
  --output-manifest "${OVERRIDE_MANIFEST}"

PYTHONPATH="${REPO_ROOT}/Deyes/src/deyes_stereo" \
  python3 "${REPO_ROOT}/Deyes/tools/run_competition_65cm_fixture.py" \
  --output "${FIXTURE_EVIDENCE}"

export OMNI_KIT_ACCEPT_EULA=YES
export ROS_DOMAIN_ID=46
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR="${LOG_ROOT}/ros-onekey"
export X1_SCENE_USD="${OVERRIDE_USD}"
export X1_NAMESPACE=x1_sim
export X1_EXECUTE=1
export X1_ENABLE_MOTION=1
export X1_ACKNOWLEDGE_MOTION_RISK=1
# The v5 scene has no grasp constraint.  This opt-in path is always reported
# as tier C synthetic attachment even though pen poses are read from PhysX.
export X1_RUN_SYNTHETIC_PICK_PLACE=1
export X1_PHYSICS_EVIDENCE_JSON="${LOG_ROOT}/physics_runtime_${RUN_ID}.json"
export LD_LIBRARY_PATH="/home/socl/miniconda3/envs/isaacsim51/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

ISAAC_LOG="${LOG_ROOT}/isaac_${RUN_ID}.log"
MAX_SECONDS="${X1_ISAAC_TIMEOUT_SEC:-420}"
echo "Isaac log: ${ISAAC_LOG}"
echo "Fixture evidence: ${FIXTURE_EVIDENCE}"
echo "Physics evidence: ${X1_PHYSICS_EVIDENCE_JSON}"

/home/socl/miniconda3/envs/isaacsim51/bin/isaacsim \
  isaacsim.exp.full.kit --no-window \
  --exec "${REPO_ROOT}/Deyes/tools/isaac_competition_65cm_adapter.py" \
  >"${ISAAC_LOG}" 2>&1 &
ISAAC_PID=$!
START_SECONDS=${SECONDS}
while kill -0 "${ISAAC_PID}" 2>/dev/null; do
  if [[ -s "${X1_PHYSICS_EVIDENCE_JSON}" ]]; then
    break
  fi
  if (( SECONDS - START_SECONDS >= MAX_SECONDS )); then
    kill -INT "${ISAAC_PID}" 2>/dev/null || true
    wait "${ISAAC_PID}" || true
    tail -n 80 "${ISAAC_LOG}"
    echo "Timed out before physics evidence was written" >&2
    exit 1
  fi
  sleep 1
done

if [[ ! -s "${X1_PHYSICS_EVIDENCE_JSON}" ]]; then
  wait "${ISAAC_PID}" || true
  tail -n 80 "${ISAAC_LOG}"
  echo "Isaac exited before physics evidence was written" >&2
  exit 1
fi

# Full-kit shutdown can outlive the completed evidence coroutine.  Interrupt
# this simulation-only process after the JSON is durable; do not retry it.
kill -INT "${ISAAC_PID}" 2>/dev/null || true
wait "${ISAAC_PID}" || true

python3 - "${X1_PHYSICS_EVIDENCE_JSON}" <<'PY'
import json
import pathlib
import sys

evidence = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
if evidence.get("accepted") is not True or evidence.get("simulation_time_advanced") is not True:
    raise SystemExit("Isaac physics evidence is not accepted")
pen = evidence.get("pen_trace")
if not isinstance(pen, dict) or pen.get("accepted") is not True:
    raise SystemExit("synthetic pen trace is not accepted")
print(json.dumps(evidence, indent=2, sort_keys=True))
PY
tail -n 25 "${ISAAC_LOG}"
