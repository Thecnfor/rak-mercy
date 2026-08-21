# Pick execution state machine (offline only)

`pick_execution_state_machine.py` is the sole closed-loop orchestration path:
`validating -> pre_grasp -> approach -> grasp -> close_gripper -> lift -> safe_retreat`.
It accepts only a `dry_run_ready` plan based on a fresh, verified `base_link`
candidate and separately requires calibration verification.

The fake backend is the acceptance backend.  It exercises cancellation,
timeouts, tracking error, gripper failure, collision/workspace rejection and
serial contention, then makes a bounded recovery request only when that would
not mask an unsafe condition.  It never opens devices.

`MercurySafetyPickBackend` shares the state machine but delegates each intent
only to `mercury_arm_safety_contract` preview validation.  It always fails
before a ROS2/serial command; `dry_run` remains the default and the existing
three execution confirmations are not bypassed.  A physical adapter requires
separate review and on-site commissioning.
