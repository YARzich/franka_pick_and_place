"""
Orchestrate exact cancellation after controller execution starts on approach.

Responsibility:
- Submit the configured TransferObjectSequenceV2 and cancel its exact UUID.
- Require fresh controller EXECUTING evidence during the current approach phase.
- Collect typed terminal, coordinator, and PlanningScene evidence.
- Print the stable success marker only after every contract is satisfied.

Does not own:
- Scenario object geometry, motion execution, scene mutation, or validation rules.
"""

from __future__ import annotations

import uuid

from manipulation_msgs.action import TransferObjectSequenceV2
from rclpy.action import ActionClient
from rclpy.node import Node
from unique_identifier_msgs.msg import UUID

from franka_motion_runtime.tasks.active_cancel_evidence import (
    load_active_cancel_probe_config,
    wait_for_cleanup_evidence,
)
from franka_motion_runtime.tasks.active_cancel_motion import (
    _ActiveCancelFeedback,
    _wait_for_cancel_checkpoint,
)
from franka_motion_runtime.tasks.active_cancel_validation import (
    validate_canceled_sequence_result,
    validate_exact_cancel_ack,
)
from franka_motion_runtime.tasks.sequence_client_cancel import (
    cancel_exact_goal,
    finish_late_goal_guard,
)
from franka_motion_runtime.tasks.sequence_client_config import (
    load_sequence_client_inputs,
)
from franka_motion_runtime.tasks.sequence_client_runtime import resolve_action_name
from franka_motion_runtime.tasks.sequence_client_waits import (
    ShutdownRequest,
    goal_send_permitted,
    wait_for_nonzero_stamp,
    wait_for_server,
    wait_future,
    wait_until_future_or_shutdown,
)
from franka_motion_runtime.tasks.sequence_goal_guard import (
    read_goal_response,
    reconcile_late_goal_response,
)
from franka_motion_runtime.tasks.sequence_messages import (
    build_transfer_sequence_goal,
)
from franka_motion_runtime.tasks.sequence_result_decode import (
    decode_sequence_result,
)


def _contain_unexpected_active_goal(
    node: Node,
    goal_handle,
    result_future,
    *,
    shutdown: ShutdownRequest,
) -> None:
    """Retain ownership when the expected feedback checkpoint is unavailable."""
    cancel_exact_goal(
        node,
        goal_handle,
        shutdown=shutdown,
        result_future=result_future,
    )


def run_active_cancel_probe(node: Node, shutdown: ShutdownRequest) -> int:
    """Run the active-cancel evidence transaction against an existing stack."""
    try:
        config, steps = load_sequence_client_inputs(node)
        probe_config = load_active_cancel_probe_config(node)
    except (TypeError, ValueError) as error:
        node.get_logger().error(f"Invalid active-cancel parameters: {error}")
        return 2
    if len(steps) != 1:
        node.get_logger().error("Active-cancel probe requires exactly one step")
        return 2

    resolved_action_name = resolve_action_name(node, config.action_name)
    client = ActionClient(node, TransferObjectSequenceV2, resolved_action_name)
    if not wait_for_server(client, shutdown=shutdown):
        node.get_logger().warning(
            f"Shutdown while waiting for action: {resolved_action_name}"
        )
        return 5
    stamp = wait_for_nonzero_stamp(node, shutdown=shutdown)
    if stamp is None:
        node.get_logger().warning(
            "Shutdown while waiting for a non-zero ROS clock stamp"
        )
        return 5
    if not goal_send_permitted(shutdown):
        return 5

    feedback = _ActiveCancelFeedback(node)
    try:
        feedback.connect_controller(probe_config.controller_action_name)
        return _run_cancel_transaction(
            node, shutdown, client, feedback, resolved_action_name,
            steps, stamp, probe_config,
        )
    finally:
        feedback.close()


def _run_cancel_transaction(
    node, shutdown, client, feedback, resolved_action_name, steps, stamp, probe_config
) -> int:
    """Submit one sequence and preserve ownership through checkpoint failure."""
    goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
    goal = build_transfer_sequence_goal(
        steps,
        stamp=stamp,
    )
    feedback.mark_submission(node.get_clock().now().nanoseconds)
    send_future = client.send_goal_async(
        goal,
        feedback_callback=feedback,
        goal_uuid=goal_uuid,
    )
    response_completed = wait_until_future_or_shutdown(
        node,
        send_future,
        shutdown=shutdown,
    )
    response_received, goal_handle = read_goal_response(send_future)
    if not response_completed or not response_received:
        resolution = reconcile_late_goal_response(
            node,
            send_future,
            action_name=resolved_action_name,
            goal_uuid=goal_uuid,
            shutdown=shutdown,
            wait_future=wait_future,
            action_type=TransferObjectSequenceV2,
        )
        finish_late_goal_guard(
            node,
            resolution,
            shutdown=shutdown,
            reason=(
                "active-cancel shutdown"
                if shutdown.requested
                else "active-cancel goal response transport failure"
            ),
        )
        return 4
    if not goal_handle.accepted:
        node.get_logger().error("Active-cancel sequence goal was rejected")
        return 4

    result_future = goal_handle.get_result_async()
    executing_goal = _wait_for_cancel_checkpoint(
        node,
        feedback,
        result_future,
        timeout_sec=probe_config.feedback_timeout_sec,
        shutdown=shutdown,
    )
    if executing_goal is None:
        if not result_future.done():
            _contain_unexpected_active_goal(
                node,
                goal_handle,
                result_future,
                shutdown=shutdown,
            )
        node.get_logger().error(
            "Did not observe a fresh EXECUTING controller goal during current "
            "step 0 APPROACHING before termination"
        )
        return 5 if shutdown.requested else 7

    node.get_logger().info(
        "ACTIVE_CANCEL_CONTROLLER_EXECUTING "
        f"action={probe_config.controller_action_name} "
        f"goal_uuid={uuid.UUID(bytes=executing_goal.goal_uuid)} "
        f"accepted_stamp_ns={executing_goal.accepted_stamp_ns}"
    )
    outcome = cancel_exact_goal(
        node,
        goal_handle,
        shutdown=shutdown,
        result_future=result_future,
    )
    errors = list(
        validate_exact_cancel_ack(
            outcome.requested_goal_uuid,
            outcome.acknowledged_goal_uuids,
        )
    )
    try:
        result = decode_sequence_result(outcome.terminal_response.result)
    except ValueError as error:
        errors.append(str(error))
    else:
        errors.extend(
            validate_canceled_sequence_result(
                status=outcome.terminal_response.status,
                result=result,
            )
        )
    errors.extend(
        wait_for_cleanup_evidence(
            node,
            canceled_object_id=steps[0].object_id,
            probe_config=probe_config,
            shutdown=shutdown,
        )
    )
    if errors:
        for error in errors:
            node.get_logger().error(f"Active-cancel evidence failed: {error}")
        return 7

    node.get_logger().info("ACTIVE_CANCEL_PASS")
    return 0
