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

mkdir -p "${SIM_ROOT}/artifacts" "${LOG_ROOT}/ros-onekey"

python3 "${REPO_ROOT}/Deyes/tools/generate_competition_65cm_scene_override.py" \
  --source-config "${SOURCE_CONFIG}" \
  --source-usd "${SOURCE_USD}" \
  --output-usda "${OVERRIDE_USD}" \
  --output-manifest "${OVERRIDE_MANIFEST}"

PYTHONPATH="${REPO_ROOT}/Deyes/src/deyes_stereo" \
  python3 "${REPO_ROOT}/Deyes/tools/run_competition_65cm_fixture.py" \
  --output "${LOG_ROOT}/fixture_fault_matrix.json"

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
export X1_EXIT_AFTER_EVIDENCE=1
export X1_PHYSICS_EVIDENCE_JSON="${LOG_ROOT}/physics_runtime.json"
export LD_LIBRARY_PATH="/home/socl/miniconda3/envs/isaacsim51/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

timeout "${X1_ISAAC_TIMEOUT_SEC:-420}" \
  /home/socl/miniconda3/envs/isaacsim51/bin/isaacsim \
  isaacsim.exp.full.kit --no-window \
  --exec "${REPO_ROOT}/Deyes/tools/isaac_competition_65cm_adapter.py" \
  2>&1 | tee "${LOG_ROOT}/isaac_onekey.log"
