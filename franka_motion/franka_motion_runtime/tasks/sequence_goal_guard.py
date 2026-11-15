"""
Reconcile a late sequence goal response by its preallocated action UUID.

Responsibility:
- Repeatedly issue targeted cancel while a send-goal response is delayed.
- Stay fail-stopped until an exact goal handle or terminal result is known.

Does not own:
- Goal construction, normal action execution, result policy, or process signals.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable

from action_msgs.srv import CancelGoal
from manipulation_msgs.action import TransferObjectSequenceV2
from unique_identifier_msgs.msg import UUID

from franka_motion_runtime.tasks.sequence_client_waits import (
    CANCEL_TRANSPORT_PROBE_SEC,
    RECONCILE_TRANSPORT_PROBE_SEC,
)
from franka_motion_runtime.tasks.sequence_terminal import (
    TerminalReadState,
    read_terminal_response,
)


class TargetedCancelEvidence(Enum):
    """Preserve distinct action cancellation outcomes for containment policy."""

    UNAVAILABLE = auto()
    NO_RESPONSE = auto()
    UNKNOWN_GOAL = auto()
    REJECTED = auto()
    GOAL_TERMINATED = auto()
    ACCEPTED = auto()


@dataclass(frozen=True)
class LateGoalResolution:
    """Report definitive evidence collected for one delayed goal response."""

    goal_handle: object | None
    response_received: bool
    cancel_evidence: TargetedCancelEvidence
    terminal_received: bool
    terminal_status: int | None = None
    terminal_disposition: int | None = None

    @property
    def targeted_cancel_confirmed(self) -> bool:
        """Return whether the server explicitly accepted exact UUID cancel."""
        return self.cancel_evidence is TargetedCancelEvidence.ACCEPTED


def _classify_cancel(response, goal_uuid: UUID) -> TargetedCancelEvidence:
    """Classify exact cancel without collapsing known-goal terminal states."""
    if response is None:
        return TargetedCancelEvidence.NO_RESPONSE
    if response.return_code == CancelGoal.Response.ERROR_GOAL_TERMINATED:
        return TargetedCancelEvidence.GOAL_TERMINATED
    if response.return_code == CancelGoal.Response.ERROR_REJECTED:
        return TargetedCancelEvidence.REJECTED
    if response.return_code == CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID:
        return TargetedCancelEvidence.UNKNOWN_GOAL
    expected = bytes(goal_uuid.uuid)
    if response.return_code == CancelGoal.Response.ERROR_NONE and any(
        bytes(goal_info.goal_id.uuid) == expected
        for goal_info in response.goals_canceling
    ):
        return TargetedCancelEvidence.ACCEPTED
    return TargetedCancelEvidence.NO_RESPONSE


def _cancel_request(goal_uuid: UUID) -> CancelGoal.Request:
    """Construct a request that can never cancel another client's goal."""
    request = CancelGoal.Request()
    request.goal_info.goal_id = goal_uuid
    return request


def _result_request(goal_uuid: UUID, action_type):
    """Construct an exact action get-result request without a goal handle."""
    request = action_type.Impl.GetResultService.Request()
    request.goal_id = goal_uuid
    return request


def _terminal_resolution(response, evidence) -> LateGoalResolution | None:
    """Decode a terminal raw get-result response or request another probe."""
    terminal = read_terminal_response(response)
    if terminal.state is not TerminalReadState.TERMINAL:
        return None
    response = terminal.response
    action_result = response.result
    disposition = None
    if action_result is not None and action_result.result is not None:
        disposition = action_result.result.final_disposition
    return LateGoalResolution(
        goal_handle=None,
        response_received=False,
        cancel_evidence=evidence,
        terminal_received=True,
        terminal_status=response.status,
        terminal_disposition=disposition,
    )


def read_goal_response(send_future) -> tuple[bool, object | None]:
    """Read a completed send future without treating exceptions as rejection."""
    if not send_future.done():
        return False, None
    try:
        goal_handle = send_future.result()
    except Exception:
        return False, None
    if goal_handle is None:
        return False, None
    return True, goal_handle


def reconcile_late_goal_response(
    node,
    send_future,
    *,
    action_name: str,
    goal_uuid: UUID,
    shutdown,
    wait_future: Callable[..., bool],
    action_type=TransferObjectSequenceV2,
) -> LateGoalResolution:
    """Remain fail-stopped until exact accepted/rejected/terminal evidence exists."""
    cancel_name = f"{action_name.rstrip('/')}/_action/cancel_goal"
    result_name = f"{action_name.rstrip('/')}/_action/get_result"
    cancel_client = node.create_client(CancelGoal, cancel_name)
    result_client = node.create_client(
        action_type.Impl.GetResultService,
        result_name,
    )
    result_future = None
    cancel_future = None
    evidence = TargetedCancelEvidence.UNAVAILABLE
    cycle = 0
    try:
        while True:
            response_received, goal_handle = read_goal_response(send_future)
            if response_received:
                return LateGoalResolution(
                    goal_handle=goal_handle,
                    response_received=True,
                    cancel_evidence=evidence,
                    terminal_received=False,
                )

            if cancel_future is None and cancel_client.wait_for_service(
                timeout_sec=CANCEL_TRANSPORT_PROBE_SEC
            ):
                cancel_future = cancel_client.call_async(_cancel_request(goal_uuid))
            if cancel_future is not None:
                if wait_future(
                    node,
                    cancel_future,
                    timeout_sec=CANCEL_TRANSPORT_PROBE_SEC,
                    shutdown=shutdown,
                    honor_shutdown=False,
                ):
                    try:
                        cancel_response = cancel_future.result()
                    except Exception:
                        cancel_response = None
                    evidence = _classify_cancel(cancel_response, goal_uuid)
                    cancel_future = None
                else:
                    if evidence is TargetedCancelEvidence.UNAVAILABLE:
                        evidence = TargetedCancelEvidence.NO_RESPONSE
            elif evidence is TargetedCancelEvidence.UNAVAILABLE:
                evidence = TargetedCancelEvidence.UNAVAILABLE

            # Cancel-service spinning may have completed the original response.
            response_received, goal_handle = read_goal_response(send_future)
            if response_received:
                return LateGoalResolution(
                    goal_handle=goal_handle,
                    response_received=True,
                    cancel_evidence=evidence,
                    terminal_received=False,
                )

            if evidence in {
                TargetedCancelEvidence.ACCEPTED,
                TargetedCancelEvidence.GOAL_TERMINATED,
                TargetedCancelEvidence.REJECTED,
            }:
                if result_future is None and result_client.wait_for_service(
                    timeout_sec=CANCEL_TRANSPORT_PROBE_SEC
                ):
                    result_future = result_client.call_async(
                        _result_request(goal_uuid, action_type)
                    )
                if result_future is not None and wait_future(
                    node,
                    result_future,
                    timeout_sec=RECONCILE_TRANSPORT_PROBE_SEC,
                    shutdown=shutdown,
                    honor_shutdown=False,
                ):
                    try:
                        result_response = result_future.result()
                    except Exception:
                        result_response = None
                    terminal = _terminal_resolution(result_response, evidence)
                    if terminal is not None:
                        return terminal
                    result_future = None

            wait_future(
                node,
                send_future,
                timeout_sec=RECONCILE_TRANSPORT_PROBE_SEC,
                shutdown=shutdown,
                honor_shutdown=False,
            )
            # Mandatory final read closes acceptance during the last wait/cancel.
            response_received, goal_handle = read_goal_response(send_future)
            if response_received:
                return LateGoalResolution(
                    goal_handle=goal_handle,
                    response_received=True,
                    cancel_evidence=evidence,
                    terminal_received=False,
                )
            cycle += 1
            node.get_logger().error(
                "Late goal containment still unresolved after cycle %d; "
                "evidence=%s, retaining process ownership"
                % (cycle, evidence.name)
            )
    finally:
        node.destroy_client(result_client)
        node.destroy_client(cancel_client)
