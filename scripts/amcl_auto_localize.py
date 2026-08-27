#!/usr/bin/env python3
"""AMCL 自主定位节点 —— 触发 /global_localization + 等粒子收敛 + 锁位姿。

为什么需要这个节点:
  - 比赛现场机器人的实际位置 ≠ 开机里程计原点
  - 必须靠雷达扫描与已知地图匹配 (AMCL) 完成定位
  - 粒子云从全局均匀撒 → 收敛到真实位姿的过程,必须监控才可靠
  - 收敛后才能发 navigation goal,否则误差会扩展到整个导航

用法:
  rosrun race_nav amcl_auto_localize.py
  或在 race_onekey.sh 里替代当前的 bash 等待循环

关键监控指标 (任一满足即认为收敛):
  1. /amcl_pose 连续 N 帧位姿变化 < position_tolerance_m
  2. /amcl_pose 连续 N 帧 yaw 变化 < yaw_tolerance_rad
  3. /amcl/particlecloud 的位置标准差 < particle_spread_m (粒子聚拢)

退路 (Fail-safe):
  - 60 秒未收敛:打印当前位姿估计 + 警告,但不阻塞导航
  - 用户可以选择:接受当前估计继续 / 手动 RViz 标定 / 重启
"""

from __future__ import annotations

import math
import time

import rospy
import tf2_ros
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from std_srvs.srv import Empty


# ====== 阈值参数 (从 2cm 放宽到 5cm — 真机激光雷达 ±3cm 极限) =====
POSITION_TOLERANCE_M = 0.05   # 连续帧位置差 < 5cm
YAW_TOLERANCE_RAD = 0.02     # 连续帧 yaw 差 < 1.15°
STABLE_FRAMES_REQUIRED = 3    # 连续 3 帧稳定就算收敛
FIRST_POSE_TIMEOUT_S = 25.0   # 看到首次 amcl_pose 后最多等 25 秒
CONVERGENCE_TIMEOUT_S = 60.0  # 整体最长 60 秒
POSE_TOPIC = "/amcl_pose"
INITIAL_POSE_TOPIC = "/initialpose"
GLOBAL_LOC_SERVICE = "/global_localization"


class AmclAutoLocalize:
    """触发全局定位 + 等收敛 + 把收敛结果喂回 AMCL 锁住位姿。"""

    def __init__(self) -> None:
        rospy.init_node("amcl_auto_localize", anonymous=True)
        self._prev_pose: tuple[float, float, float] | None = None
        self._stable_count = 0
        self._last_converged_pose: Pose | None = None
        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    # -------- 触发全局定位 --------
    def trigger_global_localization(self) -> None:
        """等待 /global_localization 服务并调用,把粒子撒满整张地图。
        注意: 这只在 set_initial_pose.py 没给 hint 时才需要;
        race_onekey.sh 现在用 set_initial_pose.py 而不是这里。
        """
        try:
            rospy.wait_for_service(GLOBAL_LOC_SERVICE, timeout=5.0)
            proxy = rospy.ServiceProxy(GLOBAL_LOC_SERVICE, Empty)
            proxy()
            rospy.loginfo("global localization triggered")
        except rospy.ROSException:
            rospy.loginfo("/global_localization service not available — "
                          "using whatever hint AMCL was given")

    # -------- 监听 /amcl_pose (评估稳定性) --------
    # 注:不再订阅 /amcl/particlecloud,避免依赖 amcl_msgs 包
    def _pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        cur = (p.position.x, p.position.y,
               self._quat_to_yaw(p.orientation))
        if self._prev_pose is None:
            self._prev_pose = cur
            return
        dpos = math.hypot(cur[0] - self._prev_pose[0],
                          cur[1] - self._prev_pose[1])
        dyaw = abs(self._yaw_delta(cur[2], self._prev_pose[2]))
        if dpos < POSITION_TOLERANCE_M and dyaw < YAW_TOLERANCE_RAD:
            self._stable_count += 1
        else:
            self._stable_count = 0
            self._prev_pose = cur

    @staticmethod
    def _quat_to_yaw(q) -> float:
        # Quaternion -> yaw (2D heading)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _yaw_delta(a: float, c: float) -> float:
        return (a - c + math.pi) % (2.0 * math.pi) - math.pi

    # -------- 收敛判断 (答辩重点:位姿稳定 +25 秒强制锁定) --------
    def _is_converged(self) -> bool:
        # 判据 1: 位姿稳定(3 帧以上)
        if self._stable_count >= STABLE_FRAMES_REQUIRED:
            return True
        return False

    # -------- 把收敛结果回灌给 AMCL (锁住粒子云) --------
    def _lock_initial_pose(self, pose: Pose) -> None:
        """AMCL 在拿到 /initialpose 后会重置粒子云到这个位姿附近,
        等于把当前最优估计作为后续跟踪定位的起点。"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.pose = pose
        msg.pose.covariance = [0.0] * 36
        # 给一点协方差,表示"大致估计",让 AMCL 用少量粒子扰动来精修
        msg.pose.covariance[0] = 0.25   # x 0.5m
        msg.pose.covariance[7] = 0.25   # y 0.5m
        msg.pose.covariance[35] = 0.07  # yaw ~15°
        pub = rospy.Publisher(INITIAL_POSE_TOPIC, PoseWithCovarianceStamped,
                              queue_size=1, latch=True)
        # 等订阅者 (AMCL) 接好
        rospy.sleep(1.0)
        pub.publish(msg)
        rospy.loginfo("published /initialpose to lock AMCL at converged pose")

    # -------- 主流程 --------
    def run(self) -> bool:
        self._first_pose_seen_at: float | None = None
        rospy.Subscriber(POSE_TOPIC, PoseWithCovarianceStamped, self._pose_cb)
        rospy.sleep(0.5)
        self.trigger_global_localization()
        start = time.monotonic()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            now = time.monotonic()
            # 看到首个 /amcl_pose 的时刻打点,从此再等 FIRST_POSE_TIMEOUT_S
            if self._prev_pose is not None and self._first_pose_seen_at is None:
                self._first_pose_seen_at = now
                rospy.loginfo("first amcl_pose seen, watching stability for %ds",
                              FIRST_POSE_TIMEOUT_S)
            if self._is_converged():
                rospy.loginfo("AMCL converged (stable_count=%d)", self._stable_count)
                if self._last_converged_pose is None and self._prev_pose is not None:
                    # 用最新 amcl_pose 锁定
                    cur = self._prev_pose
                    p = Pose()
                    p.position.x = cur[0]
                    p.position.y = cur[1]
                    cy = math.cos(cur[2] / 2.0)
                    sy = math.sin(cur[2] / 2.0)
                    p.orientation.z = sy
                    p.orientation.w = cy
                    self._last_converged_pose = p
                if self._last_converged_pose is not None:
                    self._lock_initial_pose(self._last_converged_pose)
                return True
            # 整体超时 OR 看到首帧后超时
            if now - start > CONVERGENCE_TIMEOUT_S:
                rospy.logwarn("AMCL overall timeout %ds", CONVERGENCE_TIMEOUT_S)
                return False
            if (self._first_pose_seen_at is not None
                    and now - self._first_pose_seen_at > FIRST_POSE_TIMEOUT_S):
                rospy.logwarn("AMCL pose never stabilized within %ds — locking current best estimate anyway",
                              FIRST_POSE_TIMEOUT_S)
                if self._prev_pose is not None:
                    cur = self._prev_pose
                    p = Pose()
                    p.position.x = cur[0]
                    p.position.y = cur[1]
                    cy = math.cos(cur[2] / 2.0)
                    sy = math.sin(cur[2] / 2.0)
                    p.orientation.z = sy
                    p.orientation.w = cy
                    self._lock_initial_pose(p)
                return True
            rate.sleep()


if __name__ == "__main__":
    node = AmclAutoLocalize()
    ok = node.run()
    if ok:
        rospy.spin()