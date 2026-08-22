#!/usr/bin/env python3
"""Fail-closed ROS 1 ``move_base`` adapter for the ROS 2 pick-nav gate.

ROS imports are deliberately delayed so the contract can be inspected and
tested on a development PC without Noetic.  This program never publishes
``cmd_vel`` and never retries a goal automatically.
"""

from __future__ import annotations

import json
import math
import os
from typing import Any, Mapping


MAX_POSITION_ERROR_M = 0.05
MAX_YAW_ERROR_RAD = 0.08
MAX_LINEAR_SPEED_MPS = 0.01
MAX_ANGULAR_SPEED_RADPS = 0.02
STATIONARY_HOLD_SEC = 0.5
GOAL_TIMEOUT_SEC = 90.0


def _text(value: Any) -> str:
    return str(value).strip() if value is not None else ""


def _number(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _yaw_delta(left: float, right: float) -> float:
    return (left - right + math.pi) % (2.0 * math.pi) - math.pi


def pose_error(observed: Mapping[str, Any], target: Mapping[str, Any]) -> tuple[float, float] | None:
    """Return map XY and yaw errors; target schema is intentionally small."""
    if not isinstance(observed, Mapping) or not isinstance(target, Mapping):
        return None
    values = [_number(observed.get(key)) for key in ("x", "y", "yaw_rad")] + [_number(target.get(key)) for key in ("x", "y", "yaw_rad")]
    if any(value is None for value in values):
        return None
    ox, oy, oyaw, tx, ty, tyaw = values
    assert ox is not None and oy is not None and oyaw is not None and tx is not None and ty is not None and tyaw is not None
    return math.hypot(ox - tx, oy - ty), abs(_yaw_delta(oyaw, tyaw))


def valid_site_profile(profile: Any) -> tuple[dict[str, dict[str, Any]] | None, str]:
    if not isinstance(profile, Mapping) or profile.get("schema") != "pick_navigation_site/v1":
        return None, "site_profile_schema_invalid"
    targets = profile.get("allowed_targets")
    if not isinstance(targets, list) or not targets:
        return None, "site_profile_allowlist_empty"
    indexed: dict[str, dict[str, Any]] = {}
    for entry in targets:
        if not isinstance(entry, Mapping):
            return None, "site_profile_target_invalid"
        target_id = _text(entry.get("target_id"))
        pose = entry.get("pose")
        if not target_id or target_id in indexed or not isinstance(pose, Mapping) or pose.get("frame_id") != "map" or pose_error(pose, pose) is None:
            return None, "site_profile_target_invalid"
        indexed[target_id] = {"target_id": target_id, "pose": dict(pose)}
    return indexed, "ok"


def validate_mission(payload: Any, allowlist: Mapping[str, Mapping[str, Any]]) -> tuple[dict[str, Any] | None, str]:
    """Accept only a mission whose target ID and map pose exactly match site YAML."""
    if not isinstance(payload, Mapping):
        return None, "mission_not_object"
    mission_id, target_id = _text(payload.get("mission_id")), _text(payload.get("target_id"))
    try:
        nav_epoch = int(payload.get("nav_epoch"))
    except (TypeError, ValueError):
        nav_epoch = 0
    pose = payload.get("pose")
    if not mission_id or nav_epoch <= 0 or not target_id or not isinstance(pose, Mapping):
        return None, "mission_identity_or_target_missing"
    allowed = allowlist.get(target_id)
    if allowed is None:
        return None, "mission_target_not_allowlisted"
    canonical = allowed.get("pose")
    # Explicit byte-for-value field equality intentionally rejects near-but-not-
    # identical table poses.  Only the YAML owner may change a parking pose.
    if dict(pose) != canonical:
        return None, "mission_pose_not_exact_site_allowlist_match"
    return {"mission_id": mission_id, "nav_epoch": nav_epoch, "target_id": target_id, "pose": dict(pose)}, "ok"


def startup_gate(*, enable_navigation: bool, operator_confirmed: bool, site_profile_path: str) -> str:
    if not enable_navigation:
        return "enable_navigation_false"
    if not operator_confirmed:
        return "operator_confirmed_false"
    if not _text(site_profile_path):
        return "site_profile_path_missing"
    return "ok"


def navigation_evidence(mission: Mapping[str, Any], *, result: str, stamp_ns: int, position_error_m: float = 0.0, yaw_error_rad: float = 0.0, linear_speed_mps: float = 0.0, angular_speed_radps: float = 0.0, reason: str = "", goal_sent: bool = False) -> dict[str, Any]:
    """Shared JSON envelope consumed by the ROS 2 pick-nav coordinator."""
    return {
        "schema": "pick_navigation_evidence/v1", "mission_id": mission.get("mission_id", ""),
        "nav_epoch": mission.get("nav_epoch", 0), "result": result, "stamp_ns": int(stamp_ns),
        "position_error_m": float(position_error_m), "yaw_error_rad": float(yaw_error_rad),
        "linear_speed_mps": float(linear_speed_mps), "angular_speed_radps": float(angular_speed_radps),
        "reason": reason, "navigation_goal_sent": bool(goal_sent), "commands_emitted": bool(goal_sent),
    }


def main() -> None:
    # Importing this script must remain ROS-free; deployment imports begin here.
    import actionlib
    import rospy
    import yaml
    from actionlib_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String

    class Adapter:
        def __init__(self) -> None:
            rospy.init_node("pick_navigation_adapter_ros1")
            self.enable = bool(rospy.get_param("~enable_navigation", False))
            self.confirmed = bool(rospy.get_param("~operator_confirmed", False))
            self.site_path = str(rospy.get_param("~site_profile_path", ""))
            self._publisher = rospy.Publisher("/x1/pick/navigation_evidence", String, queue_size=1, latch=False)
            self._mission: dict[str, Any] | None = None
            self._goal_started = None
            self._stationary_since = None
            self._amcl: dict[str, float] | None = None; self._amcl_receipt = None
            self._odom: tuple[float, float] | None = None; self._odom_receipt = None
            self._goal_sent = False; self._success_started = None; self._last_success_publish = None; self._completed = False
            self._allowlist: dict[str, dict[str, Any]] = {}
            reason = startup_gate(enable_navigation=self.enable, operator_confirmed=self.confirmed, site_profile_path=self.site_path)
            if reason == "ok":
                try:
                    with open(self.site_path, "r", encoding="utf-8") as handle:
                        profile, profile_reason = valid_site_profile(yaml.safe_load(handle))
                    if profile is None:
                        reason = profile_reason
                    else:
                        self._allowlist = profile
                except (OSError, yaml.YAMLError) as exc:
                    reason = "site_profile_load_failed:" + type(exc).__name__
            self._enabled_reason = reason
            self._client = actionlib.SimpleActionClient("move_base", MoveBaseAction) if reason == "ok" else None
            rospy.Subscriber("/x1/pick/nav_mission", String, self._on_mission, queue_size=1)
            rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._on_amcl, queue_size=1)
            rospy.Subscriber("/odom", Odometry, self._on_odom, queue_size=1)
            rospy.Timer(rospy.Duration(0.05), self._tick)
            rospy.logwarn("pick navigation adapter reason=%s; no cmd_vel publisher and no automatic retry", reason)

        def _publish(self, result: str, reason: str, position=0.0, yaw=0.0, linear=0.0, angular=0.0, mission=None) -> None:
            identity = self._mission if mission is None else mission
            if identity is None:
                return
            payload = navigation_evidence(identity, result=result, stamp_ns=rospy.Time.now().to_nsec(), position_error_m=position, yaw_error_rad=yaw, linear_speed_mps=linear, angular_speed_radps=angular, reason=reason, goal_sent=self._goal_sent)
            self._publisher.publish(String(data=json.dumps(payload, separators=(",", ":"))))

        @staticmethod
        def _identity(payload: Any) -> dict[str, Any] | None:
            if not isinstance(payload, Mapping): return None
            mission_id = _text(payload.get("mission_id"))
            try: epoch = int(payload.get("nav_epoch"))
            except (TypeError, ValueError): epoch = 0
            return {"mission_id": mission_id, "nav_epoch": epoch} if mission_id and epoch > 0 else None

        def _on_mission(self, message: String) -> None:
            try:
                payload = json.loads(message.data)
            except ValueError:
                rospy.logwarn("mission JSON invalid")
                return
            identity = self._identity(payload)
            if self._completed:
                self._publish("rejected", "adapter_completed_latched", mission=identity)
                return
            if self._mission is not None:
                self._publish("rejected", "prior_mission_active", mission=identity)
                return
            mission, reason = validate_mission(payload, self._allowlist)
            if self._enabled_reason != "ok":
                rospy.logwarn("navigation disabled: %s", self._enabled_reason)
                self._publish("rejected", self._enabled_reason, mission=identity)
                return
            if mission is None:
                rospy.logwarn("mission rejected: %s", reason)
                self._publish("rejected", reason, mission=identity)
                return
            assert self._client is not None
            if not self._client.wait_for_server(rospy.Duration(2.0)):
                self._mission = mission; self._publish("failed", "move_base_server_unavailable"); self._mission = None; self._completed = True; return
            pose = mission["pose"]
            goal = MoveBaseGoal(); goal.target_pose.header.frame_id = "map"; goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = float(pose["x"]); goal.target_pose.pose.position.y = float(pose["y"])
            goal.target_pose.pose.orientation.z = math.sin(float(pose["yaw_rad"]) / 2.0); goal.target_pose.pose.orientation.w = math.cos(float(pose["yaw_rad"]) / 2.0)
            self._mission = mission; self._goal_started = rospy.Time.now(); self._stationary_since = None; self._goal_sent = True
            self._client.send_goal(goal)

        def _on_amcl(self, message: PoseWithCovarianceStamped) -> None:
            q = message.pose.pose.orientation
            self._amcl = {"x": message.pose.pose.position.x, "y": message.pose.pose.position.y, "yaw_rad": math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))}
            self._amcl_receipt = rospy.Time.now()

        def _on_odom(self, message: Odometry) -> None:
            linear, angular = message.twist.twist.linear, message.twist.twist.angular
            self._odom = (math.sqrt(linear.x ** 2 + linear.y ** 2 + linear.z ** 2), math.sqrt(angular.x ** 2 + angular.y ** 2 + angular.z ** 2))
            self._odom_receipt = rospy.Time.now()

        def _tick(self, _event: Any) -> None:
            if self._mission is None or self._client is None:
                return
            elapsed = (rospy.Time.now() - self._goal_started).to_sec()
            if elapsed > GOAL_TIMEOUT_SEC:
                self._client.cancel_goal(); self._publish("timeout", "move_base_timeout"); self._mission = None; self._completed = True; return
            state = self._client.get_state()
            if state in (GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED, GoalStatus.RECALLED, GoalStatus.LOST):
                self._publish("failed", "move_base_terminal_" + str(state)); self._mission = None; self._completed = True; return
            if state != GoalStatus.SUCCEEDED:
                return
            error = pose_error(self._amcl or {}, self._mission["pose"])
            now = rospy.Time.now()
            if error is None or self._odom is None or self._amcl_receipt is None or self._odom_receipt is None:
                self._publish("failed", "amcl_or_odom_missing_after_success"); self._mission = None; self._completed = True; return
            if (now - self._amcl_receipt).to_sec() > 0.5 or (now - self._odom_receipt).to_sec() > 0.5:
                self._publish("failed", "amcl_or_odom_stale_after_success"); self._mission = None; self._completed = True; return
            position, yaw = error; linear, angular = self._odom
            if position > MAX_POSITION_ERROR_M or yaw > MAX_YAW_ERROR_RAD:
                self._publish("failed", "arrival_pose_out_of_tolerance", position, yaw, linear, angular); self._mission = None; self._completed = True; return
            if linear > MAX_LINEAR_SPEED_MPS or angular > MAX_ANGULAR_SPEED_RADPS:
                self._stationary_since = None; self._success_started = None; self._last_success_publish = None; return
            if self._stationary_since is None:
                self._stationary_since = now; return
            if (now - self._stationary_since).to_sec() < STATIONARY_HOLD_SEC:
                return
            if self._success_started is None:
                self._success_started = now; self._last_success_publish = None
            if self._last_success_publish is None or (now - self._last_success_publish).to_sec() >= 0.1:
                self._publish("succeeded", "arrival_verified", position, yaw, linear, angular); self._last_success_publish = now
            if (now - self._success_started).to_sec() >= 0.7:
                self._mission = None; self._completed = True

    Adapter()
    rospy.spin()


if __name__ == "__main__":
    main()
