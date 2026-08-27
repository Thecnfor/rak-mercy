#!/usr/bin/env bash
# race_onekey_ik.sh — VISION_IK=1 variant of race_onekey.sh.
# Same nav chain as race_onekey.sh; pick/place use IK-based scripts that
# can fall back to observation pose if ikpy/URDF is unavailable. Safe to
# swap in for race_onekey.sh on a single-arm match; never run both.
#
# Toggle behavior:
#   VISION_IK=1  (default here) -> use ik_pick / ik_place
#   VISION_IK=0               -> behave like race_onekey.sh (hardcoded)
#   FORCE_HARDCODED=1         -> bypass IK even if VISION_IK=1

set +e

# ---------------- 共用环境 (same as race_onekey.sh) ----------------
export LD_LIBRARY_PATH=/opt/ros/galactic/lib:/opt/ros/galactic/lib/aarch64-linux-gnu:/home/elephant/opencv-4.8.0-cuda/lib:${LD_LIBRARY_PATH}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
unset CYCLONEDDS_URI

source_ros1() { unset ROS_DISTRO ROS_VERSION; source /opt/ros/noetic/setup.bash; source /home/elephant/mercury_x1_ros/devel/setup.bash; }

RAC_SCRIPTS="$HOME/scripts"
LOG_DIR="/tmp/race_ik_$(date +%H%M%S)"
mkdir -p "$LOG_DIR"

VISION_IK="${VISION_IK:-1}"
FORCE_HARDCODED="${FORCE_HARDCODED:-0}"

if [ "$FORCE_HARDCODED" = "1" ]; then
    PICK_CMD="python3 $RAC_SCRIPTS/pick_pen_hardcoded.py"
    PLACE_CMD="python3 $RAC_SCRIPTS/place_pen_hardcoded.py"
    MODE="hardcoded-forced"
elif [ "$VISION_IK" = "1" ]; then
    PICK_CMD="python3 $RAC_SCRIPTS/ik_pick_pen.py"
    PLACE_CMD="python3 $RAC_SCRIPTS/ik_place_pen.py"
    MODE="ik"
else
    PICK_CMD="python3 $RAC_SCRIPTS/pick_pen_hardcoded.py"
    PLACE_CMD="python3 $RAC_SCRIPTS/place_pen_hardcoded.py"
    MODE="hardcoded"
fi

echo "[race_onekey_ik] mode=$MODE log=$LOG_DIR"
echo "[race_onekey_ik] pick=$PICK_CMD"
echo "[race_onekey_ik] place=$PLACE_CMD"

# ---------------- 杀光旧进程 ----------------
echo ""
echo "[1/5] Kill all old ROS (incl. zombies)"
pkill -9 -f amcl 2>/dev/null
pkill -9 -f move_base 2>/dev/null
pkill -9 -f pick_navigation 2>/dev/null
pkill -9 -f amcl_auto_localize 2>/dev/null
pkill -9 -f pick_pen 2>/dev/null
pkill -9 -f place_pen 2>/dev/null
pkill -9 -f ik_pick 2>/dev/null
pkill -9 -f ik_place 2>/dev/null
pkill -9 -f send_mission 2>/dev/null
pkill -9 -f send_one_goal 2>/dev/null
pkill -9 -f robot_pose_ekf 2>/dev/null
pkill -9 -f set_initial_pose 2>/dev/null
sleep 3

# ---------------- 启动导航 ----------------
echo ""
echo "[2/5] Start ROS1 navigation (move_base + amcl)"
source_ros1
roslaunch turn_on_mercury_robot navigation.launch > "$LOG_DIR/t1_nav.log" 2>&1 &

for i in {1..30}; do
    source_ros1 2>/dev/null
    if rostopic list 2>/dev/null | grep -q /move_base; then
        echo "  ✓ move_base ready (after ${i}*2s)"
        break
    fi
    sleep 2
done

# ---------------- 等 AMCL 自然收敛 ----------------
echo ""
echo "[3/5] Wait 25s for AMCL to settle on initial pose"
source_ros1 2>/dev/null
python3 "$RAC_SCRIPTS/set_initial_pose.py" \
    --site-yaml "$RAC_SCRIPTS/pick_navigation.site.yaml" \
    --target-id goal1_start \
    > "$LOG_DIR/initial_pose.log" 2>&1
sleep 20

# ---------------- Goal 1 → Goal 3 → Pick → Goal 4 → Place ----------------
echo ""
echo "[4/5] Run 3 goals + pick + place sequentially (mode=$MODE)"

echo "  → goal1_start"
python3 "$RAC_SCRIPTS/send_one_goal.py" goal1_start > "$LOG_DIR/goal1.log" 2>&1
echo "  goal1 done (RC=$?)"

echo "  → goal3_right (nav ~45s)"
python3 "$RAC_SCRIPTS/send_one_goal.py" goal3_right > "$LOG_DIR/goal3.log" 2>&1
echo "  goal3 done (RC=$?)"

# IK-based pick — fails closed back to observation pose if ikpy missing.
echo "  → $PICK_CMD"
$PICK_CMD > "$LOG_DIR/pick.log" 2>&1
PICK_RC=$?
echo "  pick RC=$PICK_RC"
if [ "$PICK_RC" -ne 0 ] && [ "$FORCE_HARDCODED" != "1" ] && [ "$VISION_IK" = "1" ]; then
    echo "  pick failed under IK — auto-degrading to hardcoded fallback"
    python3 "$RAC_SCRIPTS/pick_pen_hardcoded.py" > "$LOG_DIR/pick_fallback.log" 2>&1
    echo "  pick fallback RC=$?"
fi

echo "  → goal4_back (nav ~45s)"
python3 "$RAC_SCRIPTS/send_one_goal.py" goal4_back > "$LOG_DIR/goal4.log" 2>&1
echo "  goal4 done (RC=$?)"

echo "  → $PLACE_CMD"
$PLACE_CMD > "$LOG_DIR/place.log" 2>&1
PLACE_RC=$?
echo "  place RC=$PLACE_RC"
if [ "$PLACE_RC" -ne 0 ] && [ "$FORCE_HARDCODED" != "1" ] && [ "$VISION_IK" = "1" ]; then
    echo "  place failed under IK — auto-degrading to hardcoded fallback"
    python3 "$RAC_SCRIPTS/place_pen_hardcoded.py" > "$LOG_DIR/place_fallback.log" 2>&1
    echo "  place fallback RC=$?"
fi

echo ""
echo "[5/5] DONE: $(date)"
echo "  logs: $LOG_DIR"