"""
Deterministic fault injection and recovery transaction policy.

Responsibility:
- Attempt recovery after every injection attempt, including partial failures.
- Preserve the first failure while reporting an independent recovery failure.

Does not own:
- Command retries, subprocesses, profile selection or cleanup.
"""

from __future__ import annotations

from typing import Callable, Sequence

from franka_motion_runtime.validation.runtime_budget import fault_recovery_budget
from franka_motion_runtime.validation.runtime_process import (
    Deadline,
    ProcessError,
    ProcessIsolationError,
)
from franka_motion_runtime.validation.runtime_types import (
    CommandSpec,
    FaultProfile,
)

StageRunner = Callable[
    [str, Sequence[CommandSpec], Deadline],
    None,
]


class RecoveryFailed(ProcessError):
    """Report that motion safety could not be re-established and verified."""


def run_fault_transaction(
    fault: FaultProfile,
    deadline: Deadline,
    run_stage: StageRunner,
) -> None:
    """Run one transaction and attempt every recovery phase after injection."""
    recovery_budget = fault_recovery_budget(fault)
    fault_deadline = Deadline(
        expires_at=deadline.expires_at - recovery_budget,
        clock=deadline.clock,
    )
    fault_deadline.clip(0.001)
    injection_error = _capture(
        lambda: run_stage(_stage(fault, "inject"), fault.inject, fault_deadline)
    )
    observation_error = None
    if injection_error is None:
        observation_error = _capture(
            lambda: run_stage(
                _stage(fault, "observe"),
                fault.observe,
                fault_deadline,
            )
        )
    recovery_deadline = Deadline(
        expires_at=min(
            deadline.expires_at,
            deadline.clock() + recovery_budget,
        ),
        clock=deadline.clock,
    )
    recovery_error = _capture(
        lambda: _recover_and_verify(fault, recovery_deadline, run_stage)
    )

    primary_error = injection_error or observation_error
    if primary_error is not None and recovery_error is not None:
        raise RecoveryFailed(
            f"fault stage failed ({primary_error}); "
            f"recovery also failed ({recovery_error})"
        ) from primary_error
    if primary_error is not None:
        raise ProcessError(f"fault stage failed: {primary_error}") from primary_error
    if recovery_error is not None:
        raise RecoveryFailed(f"recovery failed: {recovery_error}") from recovery_error


def _recover_and_verify(
    fault: FaultProfile,
    deadline: Deadline,
    run_stage: StageRunner,
) -> None:
    """Attempt every recovery command and verify even after recovery failure."""
    errors = []
    errors.extend(
        _run_all(_stage(fault, "recover"), fault.recover, deadline, run_stage)
    )
    errors.extend(
        _run_all(
            _stage(fault, "verify_recovered"),
            fault.verify_recovered,
            deadline,
            run_stage,
        )
    )
    if errors:
        raise ProcessError("; ".join(str(error) for error in errors)) from errors[0]


def _run_all(
    stage: str,
    commands: Sequence[CommandSpec],
    deadline: Deadline,
    run_stage: StageRunner,
) -> list[Exception]:
    """Run recovery commands independently so one failure cannot hide another."""
    errors = []
    for command in commands:
        error = _capture(lambda command=command: run_stage(stage, (command,), deadline))
        if error is not None:
            errors.append(error)
    return errors


def _stage(fault: FaultProfile, phase: str) -> str:
    """Qualify stage evidence with its stable fault transaction ID."""
    return f"fault.{fault.fault_id}.{phase}"


def _capture(operation: Callable[[], None]) -> Exception | None:
    """Capture any ordinary stage failure so recovery remains reachable."""
    try:
        operation()
    except ProcessIsolationError:
        raise
    except Exception as error:
        return error
    return None
