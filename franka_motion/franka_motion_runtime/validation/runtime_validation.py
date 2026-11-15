"""
Ordered fault and post-recovery validation orchestration.

Responsibility:
- Run configured fault transactions in order while recovery remains proven.
- Run final canonical validation only from a confirmed recovered baseline.
- Preserve observed failures for one auditable result.

Does not own:
- Fault phase semantics, subprocess execution or evidence persistence.
"""

from __future__ import annotations

from typing import Sequence

from franka_motion_runtime.validation.runtime_budget import (
    command_sequence_budget,
    fault_deadline,
    fault_transaction_budget,
)
from franka_motion_runtime.validation.runtime_fault import (
    RecoveryFailed,
    StageRunner,
    run_fault_transaction,
)
from franka_motion_runtime.validation.runtime_process import (
    Deadline,
    ProcessError,
    ProcessIsolationError,
)
from franka_motion_runtime.validation.runtime_types import CommandSpec, FaultProfile

_STAGE_ERRORS = (ProcessError, OSError, ValueError)


def run_runtime_validation(
    faults: Sequence[FaultProfile],
    validation: Sequence[CommandSpec],
    deadline: Deadline,
    run_stage: StageRunner,
) -> None:
    """Continue through faults only while recovery and isolation stay proven."""
    errors = []
    unsafe_to_continue = False
    validation_budget = command_sequence_budget(validation)
    for index, fault in enumerate(faults):
        reserve_after = validation_budget + sum(
            fault_transaction_budget(remaining_fault)
            for remaining_fault in faults[index + 1:]
        )
        try:
            transaction_deadline = fault_deadline(
                fault,
                deadline,
                reserve_after_sec=reserve_after,
            )
            run_fault_transaction(fault, transaction_deadline, run_stage)
        except (RecoveryFailed, ProcessIsolationError) as error:
            errors.append(f"{fault.fault_id}: {error}")
            unsafe_to_continue = True
            break
        except _STAGE_ERRORS as error:
            errors.append(f"{fault.fault_id}: {error}")

    if not unsafe_to_continue:
        try:
            run_stage("validation", validation, deadline)
        except _STAGE_ERRORS as error:
            errors.append(f"validation: {error}")

    if errors:
        raise ProcessError("runtime validation failed: " + "; ".join(errors))
