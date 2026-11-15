"""
Deterministic budgets derived from immutable runtime command contracts.

Responsibility:
- Bound retries, repetitions, fault transactions and final validation.
- Reserve later recovery and validation work before a fault is injected.

Does not own:
- Process execution, retry policy, configuration parsing or evidence output.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Callable, TYPE_CHECKING

from franka_motion_runtime.validation.runtime_deadline import (
    Deadline,
    DeadlineExpired,
)
if TYPE_CHECKING:
    from franka_motion_runtime.validation.runtime_types import (
        CommandSpec,
        FaultProfile,
        RuntimeProfile,
    )


def command_sequence_budget(commands: Sequence[CommandSpec]) -> float:
    """Return the maximum configured time for all command repetitions."""
    return sum(
        command.timeout_sec * command.repetitions
        for command in commands
    )


def fault_recovery_budget(fault: FaultProfile) -> float:
    """Return the budget that remains available after fault observation."""
    return command_sequence_budget(
        fault.recover + fault.verify_recovered
    )


def fault_transaction_budget(fault: FaultProfile) -> float:
    """Return the complete configured budget for one fault transaction."""
    return command_sequence_budget(
        fault.inject
        + fault.observe
        + fault.recover
        + fault.verify_recovered
    )


def profile_foreground_budget(profile: RuntimeProfile) -> float:
    """Return the minimum deadline needed by all foreground operations."""
    return (
        command_sequence_budget(profile.preflight)
        + command_sequence_budget(profile.readiness)
        + sum(fault_transaction_budget(fault) for fault in profile.faults)
        + command_sequence_budget(profile.validation)
    )


def command_deadline(
    command: CommandSpec,
    parent: Deadline,
    clock: Callable[[], float],
) -> Deadline:
    """Give one repetition a bounded timeout shared by all retry attempts."""
    now = clock()
    expires_at = min(parent.expires_at, now + command.timeout_sec)
    deadline = Deadline(expires_at=expires_at, clock=clock)
    deadline.clip(0.001)
    return deadline


def fault_deadline(
    fault: FaultProfile,
    parent: Deadline,
    *,
    reserve_after_sec: float,
) -> Deadline:
    """Bound a fault while reserving later recoveries and validation."""
    now = parent.clock()
    reserved_expiry = parent.expires_at - reserve_after_sec
    expires_at = min(
        reserved_expiry,
        now + fault_transaction_budget(fault),
    )
    if expires_at <= now:
        raise DeadlineExpired(
            f"insufficient budget before fault {fault.fault_id!r}"
        )
    return Deadline(expires_at=expires_at, clock=parent.clock)
