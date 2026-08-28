#!/usr/bin/env bash
# Socl_ous-only, real-physics clearance acceptance. Never touches robot devices.
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SIM_ROOT="${X1_CLEARANCE_ROOT:-/var/workspace/temp/x1-real-clearance}"
SOURCE_ROOT="${X1_V5_SCENE_ROOT:-/var/workspace/docker/isaac/scenes/team_rak_finals_20260820_dual_arm_transfer_v5_low_tables}"
SOURCE_USD="$SOURCE_ROOT/outputs/team_rak_finals_20260820_dual_arm_transfer_v5_low_tables.usd"
SOURCE_CONFIG="$SOURCE_ROOT/scene_config.json"
OVERRIDE_USD="$SIM_ROOT/artifacts/team_rak_v5_65cm_clearance.usda"
OVERRIDE_MANIFEST="$SIM_ROOT/artifacts/team_rak_v5_65cm_clearance.manifest.json"
JOINT_PLAN="${X1_CLEARANCE_JOINT_PLAN:-$SIM_ROOT/input/joint_plan.json}"
NAV_RESULT="${X1_CLEARANCE_NAV_RESULT:-$SIM_ROOT/input/nav2_result.json}"
RUN_ROOT="$SIM_ROOT/runs/$(date -u +%Y%m%dT%H%M%SZ)"
PROFILE="$REPO_ROOT/Deyes/config/stereo/competition_venue_65cm.yaml"
CANDIDATE_EVIDENCE="$RUN_ROOT/competition_isaac_clearance_evidence.json"
FINAL_EVIDENCE="$REPO_ROOT/Deyes/config/stereo/competition_isaac_clearance_evidence.json"
FINAL_PROFILE="$RUN_ROOT/competition_venue_65cm.accepted.yaml"

mkdir -p "$SIM_ROOT/artifacts" "$SIM_ROOT/input" "$RUN_ROOT"
python3 "$REPO_ROOT/Deyes/tools/generate_competition_65cm_scene_override.py" \
  --source-config "$SOURCE_CONFIG" --source-usd "$SOURCE_USD" \
  --output-usda "$OVERRIDE_USD" --output-manifest "$OVERRIDE_MANIFEST"

if [[ ! -s "$JOINT_PLAN" ]]; then
  PYTHONPATH="$REPO_ROOT/Deyes/src/deyes_ik_server" \
    /home/socl/miniconda3/envs/isaacsim51/bin/python \
    "$REPO_ROOT/Deyes/tools/generate_isaac_clearance_joint_plan.py" \
    --urdf "$REPO_ROOT/Deyes/test/fixtures/mercury_x1_official_527e1c787c2b.urdf" \
    --profile "$PROFILE" --scene-usd "$OVERRIDE_USD" --output "$JOINT_PLAN"
fi
if [[ ! -s "$NAV_RESULT" ]]; then
  echo "Clearance acceptance FAILED closed: real Nav2 result is missing: $NAV_RESULT" >&2
  exit 24
fi

export OMNI_KIT_ACCEPT_EULA=YES ROS_DOMAIN_ID=46 ROS_DISTRO=jazzy X1_EXECUTE=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp X1_NAMESPACE=x1_sim
export X1_SCENE_USD="$OVERRIDE_USD"
export X1_CLEARANCE_JOINT_PLAN="$JOINT_PLAN" X1_CLEARANCE_NAV_RESULT="$NAV_RESULT"
export LD_LIBRARY_PATH="/home/socl/miniconda3/envs/isaacsim51/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

raw_runs=()
for repeat in 1 2; do
  raw="$RUN_ROOT/raw_run_${repeat}.json"; log="$RUN_ROOT/isaac_run_${repeat}.log"
  raw_runs+=("$raw")
  export X1_CLEARANCE_RAW_JSON="$raw"
  set +e
  timeout "${X1_ISAAC_TIMEOUT_SEC:-600}" /home/socl/miniconda3/envs/isaacsim51/bin/isaacsim \
    isaacsim.exp.full.kit --no-window --/telemetry/enableAnonymousData=false \
    --exec "$REPO_ROOT/Deyes/tools/isaac_real_clearance_runtime.py" \
    >"$log" 2>&1
  rc=$?
  set -e
  if [[ ! -s "$raw" ]]; then
    tail -n 100 "$log" >&2 || true
    echo "Isaac run $repeat produced no raw evidence (rc=$rc)" >&2
    exit 24
  fi
done

set +e
PYTHONPATH="$REPO_ROOT/Deyes/src/deyes_stereo" python3 \
  "$REPO_ROOT/Deyes/tools/finalize_isaac_clearance_evidence.py" \
  --profile "$PROFILE" --raw-run "${raw_runs[0]}" --raw-run "${raw_runs[1]}" \
  --output-evidence "$CANDIDATE_EVIDENCE" --output-profile "$FINAL_PROFILE"
final_rc=$?
set -e
if [[ "$final_rc" != 0 ]]; then
  echo "Clearance acceptance FAILED closed; profile was not changed. Raw runs: $RUN_ROOT" >&2
  exit "$final_rc"
fi
cp "$FINAL_PROFILE" "$PROFILE"
cp "$CANDIDATE_EVIDENCE" "$FINAL_EVIDENCE"
echo "Clearance acceptance PASSED: $FINAL_EVIDENCE"
