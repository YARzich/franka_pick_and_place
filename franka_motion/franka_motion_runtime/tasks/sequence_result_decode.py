"""
Decode TransferObjectSequenceV2 wire results into pure terminal-policy data.

Responsibility:
- Validate the nested result payload and copy its policy-relevant evidence.

Does not own:
- Result waiting, terminal status classification, logging, or cancellation.
"""

from __future__ import annotations

from franka_motion_runtime.tasks.sequence_client_policy import (
    SequenceResultSnapshot,
    StepResultSnapshot,
)


def decode_sequence_result(action_result) -> SequenceResultSnapshot:
    """Copy one complete action payload into immutable pure snapshots."""
    if action_result is None or action_result.result is None:
        raise ValueError("sequence action returned an empty result")
    aggregate = action_result.result
    return SequenceResultSnapshot(
        success=aggregate.success,
        error_code=aggregate.error_code,
        error_message=aggregate.error_message,
        final_disposition=aggregate.final_disposition,
        completed_step_count=action_result.completed_step_count,
        failed_step_index=action_result.failed_step_index,
        step_results=tuple(
            StepResultSnapshot(
                success=step_result.success,
                error_code=step_result.error_code,
                final_disposition=step_result.final_disposition,
            )
            for step_result in action_result.step_results
        ),
    )
