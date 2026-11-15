"""
Evaluate terminal state for the Franka transfer-sequence request client.

Responsibility:
- Convert action status and result evidence into a deterministic process outcome.
- Decide whether shutdown requires exact-goal cancellation.

Does not own:
- ROS futures, signal handling, message construction, or logging.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

from manipulation_msgs.msg import ManipulationTaskResult


class ActionStatus(IntEnum):
    """Mirror the stable action_msgs/GoalStatus numeric contract."""

    UNKNOWN = 0
    ACCEPTED = 1
    EXECUTING = 2
    CANCELING = 3
    SUCCEEDED = 4
    CANCELED = 5
    ABORTED = 6


@dataclass(frozen=True)
class StepResultSnapshot:
    """Contain the postcondition evidence for one started sequence step."""

    success: bool
    error_code: int
    final_disposition: int


@dataclass(frozen=True)
class SequenceResultSnapshot:
    """Contain only terminal fields needed by the process policy."""

    success: bool
    error_code: int
    error_message: str
    final_disposition: int
    completed_step_count: int
    failed_step_index: int
    step_results: tuple[StepResultSnapshot, ...]


@dataclass(frozen=True)
class TerminalDecision:
    """Describe one stable client exit decision."""

    success: bool
    exit_code: int
    detail: str


def should_cancel_exact(
    *,
    shutdown_requested: bool,
    goal_accepted: bool,
    terminal_received: bool,
) -> bool:
    """Request cancel only for this accepted, still-active action goal."""
    return shutdown_requested and goal_accepted and not terminal_received


def decide_terminal_result(
    *,
    status: int,
    result: SequenceResultSnapshot,
    expected_step_count: int,
) -> TerminalDecision:
    """Fail closed unless action status and all result evidence confirm success."""
    if status == ActionStatus.UNKNOWN:
        return TerminalDecision(False, 8, "action returned UNKNOWN status")
    if status != ActionStatus.SUCCEEDED:
        detail = f"action terminated with status={status}"
        if not result.success and result.error_message:
            detail += (
                f": {result.error_message} "
                f"(error_code={result.error_code})"
            )
        return TerminalDecision(False, 7, detail)
    if not result.success:
        detail = result.error_message or "sequence result reported failure"
        return TerminalDecision(
            False,
            7,
            f"{detail} (error_code={result.error_code})",
        )
    if result.error_code != 0:
        return TerminalDecision(
            False,
            8,
            "successful result carries a non-zero error_code",
        )
    if result.completed_step_count != expected_step_count:
        return TerminalDecision(
            False,
            8,
            "successful result has an incomplete step count",
        )
    if result.failed_step_index != -1:
        return TerminalDecision(
            False,
            8,
            "successful result identifies a failed step",
        )
    if len(result.step_results) != expected_step_count:
        return TerminalDecision(
            False,
            8,
            "successful result has inconsistent per-step evidence",
        )
    if result.final_disposition != ManipulationTaskResult.OBJECT_AT_TARGET:
        return TerminalDecision(
            False,
            8,
            "successful result does not confirm OBJECT_AT_TARGET",
        )
    for index, step_result in enumerate(result.step_results):
        if (
            not step_result.success
            or step_result.error_code != ManipulationTaskResult.NONE
            or step_result.final_disposition
            != ManipulationTaskResult.OBJECT_AT_TARGET
        ):
            return TerminalDecision(
                False,
                8,
                f"successful result has invalid evidence for step {index}",
            )
    return TerminalDecision(True, 0, "transfer sequence completed")
