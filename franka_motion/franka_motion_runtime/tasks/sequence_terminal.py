"""
Read terminal action evidence without equating Future.done with completion.

Responsibility:
- Accept only terminal GoalStatus values with a complete sequence payload.
- Preserve pending, non-terminal, and transport-error states for containment.

Does not own:
- Future spinning, cancellation, logging, or sequence result policy.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto

from action_msgs.msg import GoalStatus


class TerminalReadState(Enum):
    """Classify exact result evidence without losing uncertainty."""

    PENDING = auto()
    INVALID = auto()
    NONTERMINAL = auto()
    TERMINAL = auto()


@dataclass(frozen=True)
class TerminalRead:
    """Contain one safe result read and its optional terminal response."""

    state: TerminalReadState
    response: object | None = None


def read_terminal_response(response) -> TerminalRead:
    """Validate status and nested action payload of one result response."""
    if response is None or response.result is None:
        return TerminalRead(TerminalReadState.INVALID)
    if response.status not in {
        GoalStatus.STATUS_SUCCEEDED,
        GoalStatus.STATUS_CANCELED,
        GoalStatus.STATUS_ABORTED,
    }:
        return TerminalRead(TerminalReadState.NONTERMINAL)
    if response.result.result is None:
        return TerminalRead(TerminalReadState.INVALID)
    return TerminalRead(TerminalReadState.TERMINAL, response)


def read_terminal_future(future) -> TerminalRead:
    """Read a future safely; exceptions remain invalid containment evidence."""
    if not future.done():
        return TerminalRead(TerminalReadState.PENDING)
    try:
        response = future.result()
    except Exception:
        return TerminalRead(TerminalReadState.INVALID)
    return read_terminal_response(response)
