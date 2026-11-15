"""
Finish exact cancellation and late-goal termination for the sequence client.

Responsibility:
- Retain accepted goals through bounded probes until exact terminal evidence.
Does not own:
- Goal submission, normal success policy, clocks, or signal installation.
"""

from __future__ import annotations

from dataclasses import dataclass

from action_msgs.msg import GoalStatus
from rclpy.node import Node

from franka_motion_runtime.tasks.sequence_client_waits import (
    CANCEL_TRANSPORT_PROBE_SEC,
    ShutdownRequest,
    wait_future,
)
from franka_motion_runtime.tasks.sequence_goal_guard import LateGoalResolution
from franka_motion_runtime.tasks.sequence_terminal import (
    TerminalReadState,
    read_terminal_future,
)


@dataclass(frozen=True)
class ExactCancelOutcome:
    """Retain exact-UUID acknowledgement and terminal action evidence."""

    requested_goal_uuid: bytes
    acknowledged_goal_uuids: tuple[bytes, ...]
    terminal_response: object

    @property
    def exact_goal_acknowledged(self) -> bool:
        """Return whether the server acknowledged this specific goal UUID."""
        return self.requested_goal_uuid in self.acknowledged_goal_uuids


def cancel_exact_goal(
    node: Node,
    goal_handle,
    *,
    shutdown: ShutdownRequest,
    result_future,
    acknowledged_initially: bool = False,
) -> ExactCancelOutcome:
    """Stay fail-stopped until exact cancel or terminal evidence is available."""
    goal_uuid = bytes(goal_handle.goal_id.uuid)
    acknowledged_goal_uuids = {goal_uuid} if acknowledged_initially else set()
    cycle = 0
    cancel_future = None
    while True:
        if cancel_future is None:
            try:
                cancel_future = goal_handle.cancel_goal_async()
            except Exception:
                cancel_future = None
        cancel_completed = False
        if cancel_future is not None:
            cancel_completed = wait_future(
                node,
                cancel_future,
                timeout_sec=CANCEL_TRANSPORT_PROBE_SEC,
                shutdown=shutdown,
                honor_shutdown=False,
            )
        if cancel_completed:
            try:
                response = cancel_future.result()
            except Exception:
                response = None
            if response is not None:
                acknowledged_goal_uuids.update(
                    bytes(info.goal_id.uuid)
                    for info in response.goals_canceling
                )
            cancel_future = None
        if result_future is None:
            try:
                result_future = goal_handle.get_result_async()
            except Exception:
                result_future = None
        if result_future is not None:
            wait_future(
                node,
                result_future,
                timeout_sec=CANCEL_TRANSPORT_PROBE_SEC,
                shutdown=shutdown,
                honor_shutdown=False,
            )
            terminal = read_terminal_future(result_future)
        else:
            terminal = None
        if terminal is not None and terminal.state is TerminalReadState.TERMINAL:
            return ExactCancelOutcome(
                requested_goal_uuid=goal_uuid,
                acknowledged_goal_uuids=tuple(sorted(acknowledged_goal_uuids)),
                terminal_response=terminal.response,
            )
        if terminal is not None and terminal.state in {
            TerminalReadState.INVALID,
            TerminalReadState.NONTERMINAL,
        }:
            try:
                result_future = goal_handle.get_result_async()
            except Exception:
                result_future = None
        cycle += 1
        node.get_logger().error(
            "Exact cancel/result unresolved after cycle %d; retaining goal ownership"
            % cycle
        )


def _log_terminal_after_cancel(node: Node, response, *, reason: str) -> int:
    """Report actual terminal status and disposition after a cancel request."""
    if response is None or response.result is None:
        node.get_logger().error("Cancel reached terminal state without a result")
        return 6
    disposition = response.result.result.final_disposition
    if response.status == GoalStatus.STATUS_CANCELED:
        node.get_logger().warning(
            "Transfer sequence goal canceled after %s; disposition=%d"
            % (reason, disposition)
        )
    else:
        node.get_logger().warning(
            "Transfer sequence became terminal after %s; status=%d disposition=%d"
            % (reason, response.status, disposition)
        )
    return 5


def cancel_after_interruption(
    node: Node,
    goal_handle,
    result_future,
    *,
    shutdown: ShutdownRequest,
    reason: str,
    cancel_already_acknowledged: bool = False,
) -> int:
    """Retain ownership through bounded probes until exact terminal evidence."""
    outcome = cancel_exact_goal(
        node,
        goal_handle,
        shutdown=shutdown,
        result_future=result_future,
        acknowledged_initially=cancel_already_acknowledged,
    )
    if outcome.terminal_response is not None:
        if not outcome.exact_goal_acknowledged:
            node.get_logger().warning(
                "Cancel acknowledgement was not confirmed, but exact terminal "
                "result evidence is available"
            )
        return _log_terminal_after_cancel(
            node,
            outcome.terminal_response,
            reason=reason,
        )
    if not outcome.exact_goal_acknowledged:
        node.get_logger().error("Exact-goal cancellation was not confirmed")
        return 6
    return 6


def finish_late_goal_guard(
    node: Node,
    resolution: LateGoalResolution,
    *,
    shutdown: ShutdownRequest,
    reason: str,
) -> int:
    """Finish an accepted late goal or report exact UUID reconciliation evidence."""
    goal_handle = resolution.goal_handle
    if resolution.response_received:
        if goal_handle is None:
            node.get_logger().error("Late goal response did not contain a goal handle")
            return 8
        if not goal_handle.accepted:
            node.get_logger().warning("Delayed transfer sequence goal was rejected")
            return 5 if shutdown.requested else 4
        try:
            result_future = goal_handle.get_result_async()
        except Exception:
            result_future = None
        return cancel_after_interruption(
            node,
            goal_handle,
            result_future,
            shutdown=shutdown,
            reason=reason,
            cancel_already_acknowledged=resolution.targeted_cancel_confirmed,
        )

    if resolution.terminal_received:
        if resolution.terminal_status == GoalStatus.STATUS_CANCELED:
            node.get_logger().warning(
                "Delayed goal canceled by exact UUID after %s; disposition=%s"
                % (reason, resolution.terminal_disposition)
            )
        else:
            node.get_logger().warning(
                "Delayed goal terminal after targeted cancel; status=%s "
                "disposition=%s"
                % (
                    resolution.terminal_status,
                    resolution.terminal_disposition,
                )
            )
        return 5
    if resolution.targeted_cancel_confirmed:
        node.get_logger().error(
            "Delayed goal cancel was accepted but terminal result is unknown"
        )
        return 6
    node.get_logger().error(
        "Goal response and exact UUID cancellation remain unconfirmed"
    )
    return 8
