"""Fail-closed interface contracts for Nav2 and Mercury Turing pick adapters.

No function in this module imports vendor SDKs, sends ROS messages, or opens a
serial device.  It describes the capabilities a future execution adapter must
prove before the pick state machine may hand off any motion intent.
"""

from __future__ import annotations

from typing import Any, Iterable


NAV2_ACTION_NAME = "/navigate_to_pose"
NAV2_ACTION_TYPE = "nav2_msgs/action/NavigateToPose"
DUAL_ARM_CAPABILITIES = (
    "goal_acceptance", "feedback", "cancellation", "timeout", "result",
    "collision_check", "left_arm_feedback", "right_arm_feedback", "gripper_feedback",
)


def _normalise_name(name: str) -> str:
    return name if name.startswith("/") else "/" + name


def _graph_types(entries: Iterable[tuple[str, Iterable[str]]]) -> dict[str, set[str]]:
    return {_normalise_name(str(name)): {str(value) for value in values} for name, values in entries}


def required_motion_adapter_contract() -> dict[str, Any]:
    """Static interface requirements embedded in every dry-run plan."""
    return {
        "nav2": {
            "action_name": NAV2_ACTION_NAME,
            "action_type": NAV2_ACTION_TYPE,
            "required_semantics": ["goal", "feedback", "cancel", "timeout", "result"],
        },
        "mercury_turing_dual_arm": {
            "model": "Turing six-axis dual-arm; head is not an arm axis",
            "required_capabilities": list(DUAL_ARM_CAPABILITIES),
            "rejected_legacy_path": "sensor_msgs/JointState -> slider_control_turing -> pymycobot send_angles",
        },
        "gripper": {"required_semantics": ["goal", "feedback", "timeout", "result"]},
    }


def inspect_ros_interface_presence(
    *, action_names_and_types: Iterable[tuple[str, Iterable[str]]],
    topic_names_and_types: Iterable[tuple[str, Iterable[str]]],
) -> dict[str, Any]:
    """Classify graph evidence without subscribing or publishing a command."""
    actions = _graph_types(action_names_and_types)
    topics = _graph_types(topic_names_and_types)
    nav_types = actions.get(NAV2_ACTION_NAME, set())
    nav_state = "interface_present" if NAV2_ACTION_TYPE in nav_types else ("type_mismatch" if nav_types else "absent")
    joint_types = topics.get("/joint_states", set())
    return {
        "check_interfaces_only": True,
        "execution_permitted": False,
        "nav2": {
            "state": nav_state,
            "action_name": NAV2_ACTION_NAME,
            "expected_action_type": NAV2_ACTION_TYPE,
            "observed_action_types": sorted(nav_types),
            "required_semantics": ["goal", "feedback", "cancel", "timeout", "result"],
        },
        "mercury_turing_dual_arm": {
            "state": "unimplemented_fail_closed",
            "reason": "official_slider_control_turing_is_a_joint_states_to_vendor_sdk_bridge_not_a_safe_pick_action",
            "joint_states_observed_types": sorted(joint_types),
            "required_capabilities": list(DUAL_ARM_CAPABILITIES),
        },
    }


def validate_dual_arm_adapter_contract(adapter: dict[str, Any] | None) -> dict[str, Any]:
    """Validate a future adapter's declared semantics; never authorize motion."""
    adapter = adapter if isinstance(adapter, dict) else {}
    missing = [name for name in DUAL_ARM_CAPABILITIES if adapter.get(name) is not True]
    return {
        "contract_valid": not missing,
        "execution_permitted": False,
        "state": "contract_complete_but_execution_inhibited" if not missing else "contract_incomplete",
        "missing_capabilities": missing,
        "reason": "requires_on_site_validation_and_a_separately_reviewed_motion_adapter",
    }
