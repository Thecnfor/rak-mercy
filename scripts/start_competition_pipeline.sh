#!/usr/bin/env bash
# ROBOTAC 决赛一键启动脚本
# 启动 ROS 1 navigation + adapter + ros1_bridge + ROS 2 Deyes
# 用 4 个终端分别跑：T1/T2/T3/T4
#
# 用法：
#   ./start_competition_pipeline.sh 1    # 跑 T1 ROS 1 nav
#   ./start_competition_pipeline.sh 2    # 跑 T2 adapter
#   ./start_competition_pipeline.sh 3    # 跑 T3 ros1_bridge
#   ./start_competition_pipeline.sh 4    # 跑 T4 Deyes
#   ./start_competition_pipeline.sh 5 --target-id <id>  # 跑 T5 send_mission
#   ./start_competition_pipeline.sh 6    # 跑 T6 race_monitor
#
# 或者：
#   ./start_competition_pipeline.sh 0    # 杀光全部进程（清 Argus 鬼影）
#   ./start_competition_pipeline.sh test # 单跑 Deyes dry-run（不联动 nav）
#
# 关键环境变量（所有终端都需要）：
#   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#   export LD_LIBRARY_PATH=/opt/ros/galactic/lib:/opt/ros/galactic/lib/aarch64-linux-gnu:/home/elephant/opencv-4.8.0-cuda/lib
#
# 脚本/适配器位置：所有本地 Python 工具都放在 $HOME/scripts/，
# 上传方式（Mac → X1）：
#   scp scripts/{pick_navigation_adapter_ros1,race_monitor,send_mission}.py elephant@192.168.0.121:~/scripts/

# 不开 set -e：source 失败 / 单条命令失败不要让整个脚本退出
# 比赛当天任何小问题都不应该让"启动"流程崩
STAGE="${1:-help}"
LOG_PREFIX="/tmp/competition_$(date +%H%M%S)"

# 共用环境
export LD_LIBRARY_PATH=/opt/ros/galactic/lib:/opt/ros/galactic/lib/aarch64-linux-gnu:/home/elephant/opencv-4.8.0-cuda/lib
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI=""

# 路径（X1 上的实际位置）
# 所有本地 Python 工具（adapter / race_monitor / send_mission）都放到 $HOME/scripts/
export RAC_SCRIPTS="$HOME/scripts"
export RAC_ADAPTER="$RAC_SCRIPTS/pick_navigation_adapter_ros1.py"
export RAC_MONITOR="$RAC_SCRIPTS/race_monitor.py"
export RAC_SEND="$RAC_SCRIPTS/send_mission.py"
# ROS 2 workspace（队友的 deyes_physical_ws_8375517）
export RAC_DEYES=/home/elephant/deyes_physical_ws_8375517
# YOLO 笔检测模型
export PENCIL_ENGINE=/home/elephant/x1_real_robot/assets/models/pencil.engine
# 物理标定（spec 推导调试版，validated=false；手眼标定后用 M2 流程替换）
export STEREO_CALIB=$RAC_DEYES/install/deyes_bringup/share/deyes_bringup/config/camera/stereo_calib.yaml
# 比赛现场位姿白名单（由现场负责人填）
export SITE_YAML=/home/elephant/temp/deyes/pick_navigation.site.yaml

kill_all() {
  pkill -9 -f "_ros2_daemon" 2>/dev/null || true
  pkill -9 -f "single_shot_pick" 2>/dev/null || true
  pkill -9 -f "pick_nav_coordinator" 2>/dev/null || true
  pkill -9 -f "imx219_stereo" 2>/dev/null || true
  pkill -9 -f "deyes_" 2>/dev/null || true
  pkill -9 -f "move_base" 2>/dev/null || true
  pkill -9 -f "amcl" 2>/dev/null || true
  pkill -9 -f "mercury_x1" 2>/dev/null || true
  pkill -9 -f "dynamic_bridge" 2>/dev/null || true
  pkill -9 -f "pick_navigation_adapter_ros1" 2>/dev/null || true
  pkill -9 -f "race_monitor" 2>/dev/null || true
  pkill -9 -f "send_mission" 2>/dev/null || true
  sleep 2
}

case "$STAGE" in
  0|clean|kill)
    echo "[stage 0] kill all ROS 1/2 进程"
    kill_all
    # 额外清 Argus 鬼影
    sudo -n kill -9 $(pgrep -f nvargus-daemon) 2>/dev/null || true
    sleep 2
    sudo -n rm -rf /var/lib/nvargus-tegra/* 2>/dev/null || true
    sudo -n /usr/sbin/nvargus-daemon > /tmp/nvargus.log 2>&1 &
    sleep 3
    echo "  清理完成"
    ;;

  1|nav|ros1)
    echo "[stage 1] ROS 1 navigation（move_base + amcl）"
    unset ROS_DISTRO ROS_VERSION
    source /opt/ros/noetic/setup.bash
    source /home/elephant/mercury_x1_ros/devel/setup.bash
    # 默认用 team_rak_20260820.yaml（已 merge 到 launch 第 11 行）
    roslaunch turn_on_mercury_robot navigation.launch
    ;;

  2|adapter)
    echo "[stage 2] ROS 1 navigation adapter (pick_navigation_adapter_ros1.py)"
    source /opt/ros/noetic/setup.bash 2>/dev/null
    if [ ! -f "$RAC_ADAPTER" ]; then
      echo "  ⚠️  adapter 脚本不存在: $RAC_ADAPTER"
      echo "  scp scripts/pick_navigation_adapter_ros1.py 到 X1 的 ~/scripts/"
      exit 1
    fi
    if [ ! -f "$SITE_YAML" ]; then
      echo "  ⚠️  site YAML 不存在: $SITE_YAML"
      echo "  需要现场负责人填入桌前位姿白名单"
      exit 1
    fi
    python3 $RAC_ADAPTER \
      _enable_navigation:=true \
      _operator_confirmed:=true \
      _site_profile_path:=$SITE_YAML
    ;;

  3|bridge|ros1_bridge)
    echo "[stage 3] ros1_bridge dynamic_bridge"
    source /opt/ros/galactic/setup.bash
    source $RAC_DEYES/install/setup.bash
    # ros1_bridge 是 ROS 2 binary 但动态链接 ROS 1 的 libroscpp.so,必须 source noetic
    # 在 galactic 之后 source,这样 noetic 的 LD_LIBRARY_PATH 才会叠加到 galactic 上面
    source /opt/ros/noetic/setup.bash 2>/dev/null
    # 把 noetic 的 lib 路径加到 LD_LIBRARY_PATH 前面(以防 setup.bash 没正确覆盖)
    export LD_LIBRARY_PATH=/opt/ros/noetic/lib:${LD_LIBRARY_PATH}
    ros2 run ros1_bridge dynamic_bridge
    ;;

  4|deyes|vision)
    echo "[stage 4] ROS 2 Deyes single_shot_pick"
    source /opt/ros/galactic/setup.bash
    source $RAC_DEYES/install/setup.bash
    ros2 launch deyes_bringup navigation_single_shot_pick.launch.py \
      dry_run:=false \
      enable_live_execution:=true \
      operator_confirmed:=true \
      model_path:=$PENCIL_ENGINE \
      stereo_calibration_path:=$STEREO_CALIB \
      log_root:=/tmp/competition_log
    ;;

  5|send|mission)
    echo "[stage 5] T5 send_mission (Python 发 mission, 避开 JSON escape)"
    shift  # 去掉 stage 参数，剩下的传给 send_mission.py
    source /opt/ros/noetic/setup.bash 2>/dev/null
    if [ ! -f "$RAC_SEND" ]; then
      echo "  ⚠️  send 脚本不存在: $RAC_SEND"
      echo "  scp scripts/send_mission.py 到 X1 的 \$HOME/scripts/"
      exit 1
    fi
    python3 "$RAC_SEND" \
      --site-yaml "$SITE_YAML" \
      "$@"
    ;;

  6|monitor|watch)
    echo "[stage 6] T6 race_monitor (T1-T5 健康面板)"
    source /opt/ros/noetic/setup.bash 2>/dev/null
    source /opt/ros/galactic/setup.bash 2>/dev/null
    if [ ! -f "$RAC_MONITOR" ]; then
      echo "  ⚠️  monitor 脚本不存在: $RAC_MONITOR"
      echo "  scp scripts/race_monitor.py 到 X1 的 \$HOME/scripts/"
      exit 1
    fi
    python3 "$RAC_MONITOR"
    ;;

  test|dryrun|all-dryrun)
    echo "[stage test] 完整 dry-run（不联动 nav）"
    kill_all
    source /opt/ros/galactic/setup.bash
    source $RAC_DEYES/install/setup.bash
    ros2 launch deyes_bringup navigation_single_shot_pick.launch.py \
      dry_run:=true \
      enable_live_execution:=false \
      operator_confirmed:=false \
      model_path:=$PENCIL_ENGINE \
      stereo_calibration_path:=$STEREO_CALIB \
      log_root:=/tmp/dryrun_log
    ;;

  all|full)
    echo "[stage all] 顺序启动：T1=ros1 nav, T2=adapter, T3=ros1_bridge, T4=Deyes, T5=send, T6=monitor"
    echo "  推荐用 6 个 SSH 终端分别跑："
    echo "  Terminal 1: $0 1"
    echo "  Terminal 2: $0 2"
    echo "  Terminal 3: $0 3"
    echo "  Terminal 4: $0 4"
    echo "  Terminal 5: $0 5 --target-id <id>    # T5 手动 send mission"
    echo "  Terminal 6: $0 6                     # T6 race_monitor"
    ;;

  help|*)
    cat <<EOF
用法: $0 <stage>

stages:
  0    杀光所有进程（清 Argus 鬼影）
  1    T1 ROS 1 navigation (move_base + amcl)
  2    T2 ROS 1 adapter (pick_navigation_adapter_ros1.py)
  3    T3 ros1_bridge dynamic_bridge
  4    T4 ROS 2 Deyes (single_shot_pick, live mode)
  5    T5 send_mission (--target-id <id> 必填)
  6    T6 race_monitor (T1-T5 健康面板)
  test dry-run（不联动 nav，单独跑 Deyes）
  all  显示 6 终端用法

关键环境（所有终端必设）:
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export LD_LIBRARY_PATH=/opt/ros/galactic/lib:/opt/ros/galactic/lib/aarch64-linux-gnu:/home/elephant/opencv-4.8.0-cuda/lib

赛前同步（Mac → X1）:
  scp scripts/{pick_navigation_adapter_ros1,race_monitor,send_mission}.py \\
      elephant@192.168.0.121:~/scripts/
EOF
    ;;
esac
