#!/usr/bin/env bash
# race_onekey.sh — ROBOTAC 决赛 一键全自动启动 (无 prompt 版)
# 用法: bash ~/scripts/race_onekey.sh
#
# 设计原则 (从现场失败中修订):
# 1. 不等待 AMCL 收敛(不可靠)—— 直接 sleep 25s 让 AMCL 自然跑
# 2. 不走 adapter(它会报 stale_after_success)—— 直接 send_one_goal.py 到 move_base
# 3. 不调 amcl_auto_localize.py(它的 loginfo 不到 stdout,误判成 hang)
# 4. 没有任何 prompt 或 read -t 30, 全自动跑完
# 5. 每个 goal 后 sleep 一个合理时长让 move_base 执行, 不轮询 evidence

set +e

# ---------------- 共用环境 ----------------
export LD_LIBRARY_PATH=/opt/ros/galactic/lib:/opt/ros/galactic/lib/aarch64-linux-gnu:/home/elephant/opencv-4.8.0-cuda/lib:${LD_LIBRARY_PATH}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
unset CYCLONEDDS_URI

source_ros1() { unset ROS_DISTRO ROS_VERSION; source /opt/ros/noetic/setup.bash; source /home/elephant/mercury_x1_ros/devel/setup.bash; }

RAC_SCRIPTS="$HOME/scripts"
LOG_DIR="/tmp/race_$(date +%H%M%S)"
mkdir -p "$LOG_DIR"

echo "[race_onekey] start: $(date) log=$LOG_DIR"

# ---------------- 杀光旧进程 ----------------
echo ""
echo "[1/5] Kill all old ROS (incl. zombies)"
pkill -9 -f amcl 2>/dev/null
pkill -9 -f move_base 2>/dev/null
pkill -9 -f pick_navigation 2>/dev/null
pkill -9 -f amcl_auto_localize 2>/dev/null
pkill -9 -f pick_pen_hardcoded 2>/dev/null
pkill -9 -f place_pen_hardcoded 2>/dev/null
pkill -9 -f send_mission 2>/dev/null
pkill -9 -f send_one_goal 2>/dev/null
# robot_pose_ekf conflicts with AMCL's TF on some setups
pkill -9 -f robot_pose_ekf 2>/dev/null
# Old adapter / set_initial_pose zombies
pkill -9 -f set_initial_pose 2>/dev/null
sleep 3
echo "  remaining amcl procs (should be 0): $(pgrep -f amcl | wc -l)"

# ---------------- 启动导航 ----------------
echo ""
echo "[2/5] Start ROS1 navigation (move_base + amcl)"
source_ros1
roslaunch turn_on_mercury_robot navigation.launch > "$LOG_DIR/t1_nav.log" 2>&1 &

# 等 move_base topic 出现 (max 60s)
for i in {1..30}; do
    source_ros1 2>/dev/null
    if rostopic list 2>/dev/null | grep -q /move_base; then
        echo "  ✓ move_base ready (after ${i}*2s)"
        break
    fi
    sleep 2
done

# ---------------- 等 AMCL 自然收敛 + 发 hint ----------------
echo ""
echo "[3/5] Wait 25s for AMCL to settle on initial pose"
source_ros1 2>/dev/null
python3 "$RAC_SCRIPTS/set_initial_pose.py" \
    --site-yaml "$RAC_SCRIPTS/pick_navigation.site.yaml" \
    --target-id goal1_start \
    > "$LOG_DIR/initial_pose.log" 2>&1
echo "  initial pose hint published at goal1_start"

# 再给 AMCL 20s 接收 lidar 确认
sleep 20

# ---------------- Goal 1 → Goal 3 → Pick → Goal 4 → Place ----------------
echo ""
echo "[4/5] Run 3 goals + pick + place sequentially"

# Goal 1: start (already there, should be ~10s)
echo "  → goal1_start (already at start, expected quick)"
python3 "$RAC_SCRIPTS/send_one_goal.py" goal1_start > "$LOG_DIR/goal1.log" 2>&1
echo "  goal1 done (RC=$?)"

# Goal 3: navigate to right side, pick pen
echo "  → goal3_right (nav ~45s)"
python3 "$RAC_SCRIPTS/send_one_goal.py" goal3_right > "$LOG_DIR/goal3.log" 2>&1
echo "  goal3 done (RC=$?)"

# Pick
echo "  → pick_pen_hardcoded.py (single arm, ~20s)"
python3 "$RAC_SCRIPTS/pick_pen_hardcoded.py" > "$LOG_DIR/pick.log" 2>&1
echo "  pick RC=$?"

# Goal 4: navigate to back, place pen
echo "  → goal4_back (nav ~45s)"
python3 "$RAC_SCRIPTS/send_one_goal.py" goal4_back > "$LOG_DIR/goal4.log" 2>&1
echo "  goal4 done (RC=$?)"

# Place
echo "  → place_pen_hardcoded.py (single arm, ~15s)"
python3 "$RAC_SCRIPTS/place_pen_hardcoded.py" > "$LOG_DIR/place.log" 2>&1
echo "  place RC=$?"

# ---------------- 总结 ----------------
echo ""
echo "[5/5] DONE: $(date)"
echo "  logs: $LOG_DIR"
echo ""
echo "  === summary ==="
for f in "$LOG_DIR"/goal*.log "$LOG_DIR"/pick.log "$LOG_DIR"/place.log; do
    [ -f "$f" ] && echo "  $f: $(tail -n1 "$f")"
done