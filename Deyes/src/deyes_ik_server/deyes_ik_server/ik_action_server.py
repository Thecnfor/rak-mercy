"""ROS2 action server implementing deyes_interfaces/action/ExecuteCartesianStage.

The server lives on the venue robot in a ROS2 process. It is intentionally
small: it subscribes to ``/joint_states`` for the current pose, holds an
``IkpySolver7DOF`` per arm, and turns each goal into joint degrees.

Why no Jacobian / trajectory planning here? The ExecuteCartesianStage
contract is *one Cartesian waypoint*, not a path. Pymycobot handles the
interpolation when we feed it ``send_angles``. Adding trajectory
smoothing belongs in the calling state machine, not in IK.
"""

from __future__ import annotations

import math
from typing import Dict, Optional

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:  # deyes_interfaces lives in the same workspace
    from deyes_interfaces.action import ExecuteCartesianStage
except Exception as _exc:  # pragma: no cover - allows dry import elsewhere
    ExecuteCartesianStage = None
    _IMPORT_ERROR = _exc

from .ikpy_solver import IkpySolver7DOF
from .mock_solver import MockSolver
from .urdf_loader import arm_joint_names


def _make_solver(kind: str, arm_side: str):
    if kind == "mock":
        return MockSolver(arm_side=arm_side,
                          observation_pose_deg=IkpySolver7DOF.OBSERVATION_POSE_RIGHT_DEG)
    if kind == "ikpy":
        return IkpySolver7DOF(arm_side=arm_side)
    raise ValueError(f"unknown solver_type {kind!r}")


class ExecuteCartesianStageActionServer(Node):
    def __init__(self) -> None:
        super().__init__("execute_cartesian_stage_ik")
        if ExecuteCartesianStage is None:
            raise RuntimeError(
                "deyes_interfaces is not on PYTHONPATH / not built yet. "
                f"Original error: {_IMPORT_ERROR!r}"
            )

        # Parameters — sensible defaults so the launch file can stay terse.
        self.declare_parameter("solver_type", "ikpy")
        self.declare_parameter("arm_sides", ["right", "left"])
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("publish_result_topic", "/x1/ik/last_solution")
        self.declare_parameter("max_iter", 60)
        self.declare_parameter("tol_m", 0.005)

        solver_type = self.get_parameter("solver_type").value
        arm_sides: list[str] = list(self.get_parameter("arm_sides").value)
        self._solvers: Dict[str, object] = {
            side: _make_solver(solver_type, side) for side in arm_sides
        }
        self._current_deg: Dict[str, list[float]] = {side: list(IkpySolver7DOF.OBSERVATION_POSE_RIGHT_DEG) for side in arm_sides}

        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._on_joint_state,
            10,
        )
        self._result_pub = self.create_publisher(
            JointState,
            str(self.get_parameter("publish_result_topic").value),
            10,
        )

        self._server = ActionServer(
            self,
            ExecuteCartesianStage,
            "/x1/ik/execute_cartesian_stage",
            execute_callback=self._execute_callback,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.get_logger().info(
            f"ExecuteCartesianStage IK server up: solver_type={solver_type}, arms={arm_sides}"
        )

    # ---- joint state ------------------------------------------------

    def _on_joint_state(self, msg: JointState) -> None:
        """Cache the current joint angles per arm for the next IK seed."""
        # JointState name order matches URDF joint ordering (joint1_R..joint7_R then joint1_L..joint7_L).
        for side, names in arm_joint_names_dict().items():
            angles = []
            for jname in names:
                if jname in msg.name:
                    angles.append(math.degrees(float(msg.position[msg.name.index(jname)])))
            if len(angles) == 7:
                self._current_deg[side] = angles

    # ---- action execute --------------------------------------------

    def _execute_callback(self, goal_handle):
        goal = goal_handle.request
        arm_side = (goal.arm_side or "right").strip().lower()
        pose = list(goal.pose_base)
        if arm_side not in self._solvers:
            goal_handle.abort()
            return self._result(False, f"UNKNOWN_ARM_SIDE:{arm_side}", pose)

        solver = self._solvers[arm_side]
        seed = self._current_deg.get(arm_side)
        max_iter = int(self.get_parameter("max_iter").value)
        tol_m = float(self.get_parameter("tol_m").value)

        result_obj = solver.solve(pose, seed, max_iter=max_iter, tol_m=tol_m)
        if not result_obj.success:
            goal_handle.abort()
            return self._result(False, result_obj.failure_code or "IK_FAILED", pose,
                                joint_deg=result_obj.joint_deg)

        self._current_deg[arm_side] = result_obj.joint_deg
        self._publish_solution(arm_side, result_obj.joint_deg)
        goal_handle.succeed()
        return self._result(True, "", pose, joint_deg=result_obj.joint_deg)

    # ---- helpers ---------------------------------------------------

    def _result(self, success: bool, failure_code: str, pose: list[float],
                joint_deg: Optional[list[float]] = None):
        out = ExecuteCartesianStage.Result()
        out.success = success
        out.failure_code = failure_code
        out.final_pose_base = pose
        out.final_joint_deg = list(joint_deg) if joint_deg else [0.0] * 7
        return out

    def _publish_solution(self, arm_side: str, joint_deg: list[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        names = arm_joint_names(arm_side)
        msg.name = list(names)
        msg.position = [math.radians(float(v)) for v in joint_deg]
        self._result_pub.publish(msg)


def arm_joint_names_dict() -> Dict[str, tuple]:
    return {"left": arm_joint_names("left"), "right": arm_joint_names("right")}


def main(args=None):
    rclpy.init(args=args)
    node = ExecuteCartesianStageActionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()