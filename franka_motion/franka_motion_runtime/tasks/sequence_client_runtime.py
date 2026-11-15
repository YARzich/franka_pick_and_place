"""
Orchestrate one Franka TransferObjectSequenceV2 action goal.

Responsibility:
- Compose validated inputs, feedback, normal result policy, and cancel helpers.
- Keep all transitions to action side effects explicit and fail closed.

Does not own:
- Parameter parsing, steady wait mechanics, cancel protocol, or server resolution.
"""

from __future__ import annotations

from typing import Callable
import uuid

from manipulation_msgs.action import TransferObjectSequenceV2
from rclpy.action import ActionClient
from rclpy.node import Node
from unique_identifier_msgs.msg import UUID

from franka_motion_runtime.tasks.sequence_client_cancel import (
    cancel_after_interruption,
    finish_late_goal_guard,
)
from franka_motion_runtime.tasks.sequence_client_config import (
    SequenceClientConfig,
    load_sequence_client_inputs,
)
from franka_motion_runtime.tasks.sequence_client_feedback import FeedbackLogger
from franka_motion_runtime.tasks.sequence_client_policy import (
    decide_terminal_result,
    should_cancel_exact,
)
from franka_motion_runtime.tasks.sequence_result_decode import decode_sequence_result
from franka_motion_runtime.tasks.sequence_client_waits import (
    ShutdownRequest,
    goal_send_permitted,
    wait_for_nonzero_stamp,
    wait_for_server,
    wait_for_terminal_result,
    wait_future,
    wait_until_future_or_shutdown,
)
from franka_motion_runtime.tasks.sequence_goal_guard import (
    read_goal_response,
    reconcile_late_goal_response,
)
from franka_motion_runtime.tasks.sequence_messages import build_transfer_sequence_goal
from franka_motion_runtime.tasks.model import TransferObjectIntent
from franka_motion_runtime.tasks.sequence_terminal import (
    TerminalReadState,
    read_terminal_future,
)


def resolve_action_name(node: Node, configured_name: str) -> str:
    """Apply ROS remapping once before action and raw protocol clients agree."""
    return node.resolve_topic_name(configured_name)


def execute_sequence_goal(
    node: Node,
    shutdown: ShutdownRequest,
    *,
    config: SequenceClientConfig,
    expected_step_count: int,
    goal_builder: Callable[[object], object],
) -> int:
    """Execute one V2 goal while retaining exact cancellation ownership."""
    if expected_step_count <= 0:
        raise ValueError("expected_step_count must be positive")
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
    goal = goal_builder(stamp)
    goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
    if not goal_send_permitted(shutdown):
        node.get_logger().warning("Shutdown requested before goal submission")
        return 5
    send_future = client.send_goal_async(
        goal,
        feedback_callback=FeedbackLogger(node),
        goal_uuid=goal_uuid,
    )
    response_wait_completed = wait_until_future_or_shutdown(
        node,
        send_future,
        shutdown=shutdown,
    )
    response_received, goal_handle = read_goal_response(send_future)
    if not response_wait_completed or not response_received:
        reason = (
            "client shutdown"
            if shutdown.requested
            else "goal response transport failure"
        )
        resolution = reconcile_late_goal_response(
            node,
            send_future,
            action_name=resolved_action_name,
            goal_uuid=goal_uuid,
            shutdown=shutdown,
            wait_future=wait_future,
            action_type=TransferObjectSequenceV2,
        )
        return finish_late_goal_guard(
            node,
            resolution,
            shutdown=shutdown,
            reason=reason,
        )
    if not goal_handle.accepted:
        node.get_logger().error("Transfer sequence goal was rejected")
        return 4

    node.get_logger().info(
        f"Accepted transfer sequence with {expected_step_count} step(s)"
    )
    try:
        result_future = goal_handle.get_result_async()
    except Exception:
        result_future = None
    if result_future is None:
        return cancel_after_interruption(
            node,
            goal_handle,
            result_future,
            shutdown=shutdown,
            reason="result transport unavailable",
        )
    terminal_received = wait_for_terminal_result(
        node,
        result_future,
        shutdown=shutdown,
    )
    terminal = read_terminal_future(result_future) if terminal_received else None
    if should_cancel_exact(
        shutdown_requested=shutdown.requested,
        goal_accepted=True,
        terminal_received=(
            terminal is not None and terminal.state is TerminalReadState.TERMINAL
        ),
    ):
        return cancel_after_interruption(
            node,
            goal_handle,
            result_future,
            shutdown=shutdown,
            reason="client shutdown",
        )
    if terminal is None or terminal.state is not TerminalReadState.TERMINAL:
        return cancel_after_interruption(
            node,
            goal_handle,
            result_future,
            shutdown=shutdown,
            reason="invalid result transport response",
        )

    response = terminal.response
    try:
        snapshot = decode_sequence_result(response.result)
    except ValueError as error:
        node.get_logger().error(str(error))
        return 8
    decision = decide_terminal_result(
        status=response.status,
        result=snapshot,
        expected_step_count=expected_step_count,
    )
    log: Callable[[str], None]
    log = node.get_logger().info if decision.success else node.get_logger().error
    log(decision.detail)
    return decision.exit_code


def execute_intent_sequence(
    node: Node,
    shutdown: ShutdownRequest,
    *,
    config: SequenceClientConfig,
    steps: tuple[TransferObjectIntent, ...],
) -> int:
    """Map validated client intent and execute one contained V2 action goal."""
    return execute_sequence_goal(
        node,
        shutdown,
        config=config,
        expected_step_count=len(steps),
        goal_builder=lambda stamp: build_transfer_sequence_goal(
            steps,
            stamp=stamp,
        ),
    )


def run_sequence_client(node: Node, shutdown: ShutdownRequest) -> int:
    """Load indexed step parameters and execute one contained action goal."""
    try:
        config, steps = load_sequence_client_inputs(node)
    except (TypeError, ValueError) as error:
        node.get_logger().error(f"Invalid transfer sequence parameters: {error}")
        return 2
    return execute_intent_sequence(
        node,
        shutdown,
        config=config,
        steps=steps,
    )
