#!/usr/bin/env bash
# M2 物理双目标定（IMX219 stereo @ 640x360）
# 参考 Deyes/tools/m2_calibration_workflow.md
#
# 用法：
#   1. 准备 9x6 内角点棋盘 + 卡尺量新板单格边长
#   2. 启动 IMX219 立体相机发布（640x360@30）
#   3. ./m2_calibrate.sh capture <square_size_m>   # 采 50 帧
#   4. 核对候选输出: reproj_rms_px / epipolar_p95_px / T[0] 符号 / 尺度
#   5. ./m2_calibrate.sh compute <square_size_m>    # 求解,生成 YAML
#
# 候选 YAML 路径：
#   /home/elephant/temp/deyes/calibration/<utc-session>/stereo_calib_candidate.yaml
# **不会**自动 copy 到 config/camera/ — 等人工审阅报告再决定

# 注意：故意不开 `set -u`。
# ROS 2 的 setup.bash 引用 `AMENT_TRACE_SETUP_FILES`（tracing 时才设），
# 在普通 shell 上下文里是 unbound variable，set -u 会让 source 直接 abort。

ROS_SETUP=/opt/ros/galactic/setup.bash
WS_SETUP=/home/elephant/deyes_physical_ws_8375517/install/setup.bash
SESSION_ROOT=/home/elephant/temp/deyes/calibration
BOARD_COLS=9
BOARD_ROWS=6
SAMPLES=50

die() { echo "ERROR: $*" >&2; exit 1; }

if [ ! -f "$WS_SETUP" ]; then
  die "deyes workspace not found at $WS_SETUP"
fi

# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$WS_SETUP"

CMD="${1:-help}"
SQUARE_M="${2:-}"

case "$CMD" in
  capture)
    [ -n "$SQUARE_M" ] || die "用法: $0 capture <square_size_m>   (例: 0.0254 for 25.4mm)"
    SESSION="${SESSION_ROOT}/$(date -u +%Y%m%dT%H%M%SZ)"
    # Python 工具自己 mkdir session_dir (exist_ok=False)。
    # 我们必须**不**预创建，否则第二次 mkdir 会 FileExistsError。
    # 同时清掉之前失败留下的 stale dir，避免重跑撞名。
    if [ -d "$SESSION" ]; then
      echo "[m2] 清理 stale session dir: $SESSION"
      rm -rf "$SESSION"
    fi
    echo "[m2] session: $SESSION"
    echo "[m2] board: ${BOARD_COLS}x${BOARD_ROWS}  square: ${SQUARE_M}m  samples: ${SAMPLES}"
    echo "[m2] 移动棋盘覆盖 九宫格位置 + 近中远距离 + 轻微 yaw/pitch"
    ros2 run deyes_stereo physical_stereo_calibration capture \
      --session-dir "$SESSION" \
      --board-cols "$BOARD_COLS" --board-rows "$BOARD_ROWS" \
      --square-size-m "$SQUARE_M" \
      --samples "$SAMPLES"
    echo "[m2] capture done.  session: $SESSION"
    echo "[m2] next: 核对 manifest, 然后 $0 compute $SQUARE_M"
    ;;

  compute)
    [ -n "$SQUARE_M" ] || die "用法: $0 compute <square_size_m>"
    # 取最近的 session
    SESSION="$(ls -1dt "${SESSION_ROOT}"/*/ 2>/dev/null | head -1)"
    [ -n "$SESSION" ] || die "没找到 capture session, 先跑 $0 capture $SQUARE_M"
    SESSION="${SESSION%/}"
    echo "[m2] compute on session: $SESSION"
    echo "[m2] ** 3 项人工确认 (--confirm-* flags):"
    echo "       1. 左右目没交换 (--confirm-left-right)"
    echo "       2. 基线符号正确 (--confirm-baseline-sign)"
    echo "       3. 尺度匹配卡尺读数 (--confirm-scale)"
    read -rp "[m2] 3 项确认都做了? [y/N] " ok
    [ "$ok" = "y" ] || die "需要先 3 项确认再 compute"
    ros2 run deyes_stereo physical_stereo_calibration compute \
      --session-dir "$SESSION" \
      --robot-id "$(hostname)" \
      --camera-pair-id imx219-stereo \
      --board-cols "$BOARD_COLS" --board-rows "$BOARD_ROWS" \
      --square-size-m "$SQUARE_M" \
      --confirm-left-right --confirm-baseline-sign --confirm-scale
    echo ""
    echo "[m2] 候选 YAML: $SESSION/stereo_calib_candidate.yaml"
    echo "[m2] **不**自动 copy 到 config/camera/ — 审阅报告和报告里的:"
    echo "       reproj_rms_px (< 0.50 px ?)"
    echo "       epipolar_p95_px (< 0.50 px ?)"
    echo "       T[0] 符号 (符合已知左在右前的基线?)"
    echo "[m2] 7 个停止条件任一不满足 -> 标 validated=false, 不进点云/抓取"
    ;;

  help|*)
    cat <<EOF
M2 物理双目标定 — 用法:
  $0 capture <square_size_m>     # 采 50 帧棋盘
  $0 compute  <square_size_m>     # 求解, 写候选 YAML

<square_size_m>: 卡尺实测的单格边长 (m), 例 25.4mm -> 0.0254

棋盘规格 (M2 硬要求):
  - 9x6 内角点 (不是 8x7)
  - print at 100% scale
  - 边长用卡尺量, 不得继承旧板

候选 YAML 路径:
  $SESSION_ROOT/<utc-session>/stereo_calib_candidate.yaml

跑完 compute 后:
  1. 检查 reproj_rms_px < 0.50, epipolar_p95_px < 0.50
  2. 检查 T[0] 符号 (基线方向)
  3. 检查 K1/D1/K2/D2/P1/P2/Q 矩阵
  4. 报告 + 矩阵 -> 人工审阅
  5. **不**自动 copy 到 config/camera/stereo_calib.yaml — 等批准
EOF
    ;;
esac
