"""
Test fail-closed transfer-sequence client terminal policy.

Responsibility:
- Preserve exact-goal cancellation criteria and strict success evidence.
"""

import pytest
from manipulation_msgs.action import TransferObjectSequenceV2
from manipulation_msgs.msg import ManipulationTaskResult

from franka_motion_runtime.tasks.sequence_client_policy import (
    ActionStatus,
    SequenceResultSnapshot,
    StepResultSnapshot,
    decide_terminal_result,
    should_cancel_exact,
)
from franka_motion_runtime.tasks.sequence_result_decode import decode_sequence_result


def _result(**overrides) -> SequenceResultSnapshot:
    """Return a valid two-step result with selected field overrides."""
    values = {
        "success": True,
        "error_code": 0,
        "error_message": "",
        "final_disposition": 4,
        "completed_step_count": 2,
        "failed_step_index": -1,
        "step_results": (
            StepResultSnapshot(True, 0, 4),
            StepResultSnapshot(True, 0, 4),
        ),
    }
    values.update(overrides)
    return SequenceResultSnapshot(**values)


def test_only_complete_succeeded_evidence_returns_zero() -> None:
    """Transport success alone is insufficient without all postconditions."""
    decision = decide_terminal_result(
        status=ActionStatus.SUCCEEDED,
        result=_result(),
        expected_step_count=2,
    )
    assert decision.success
    assert decision.exit_code == 0


@pytest.mark.parametrize(
    ("status", "result"),
    [
        (ActionStatus.UNKNOWN, _result()),
        (ActionStatus.ABORTED, _result()),
        (ActionStatus.SUCCEEDED, _result(success=False, error_code=107)),
        (ActionStatus.SUCCEEDED, _result(error_code=255)),
        (ActionStatus.SUCCEEDED, _result(completed_step_count=1)),
        (ActionStatus.SUCCEEDED, _result(failed_step_index=1)),
        (
            ActionStatus.SUCCEEDED,
            _result(step_results=(StepResultSnapshot(True, 0, 4),)),
        ),
        (
            ActionStatus.SUCCEEDED,
            _result(
                step_results=(
                    StepResultSnapshot(True, 0, 4),
                    StepResultSnapshot(False, 107, 0),
                )
            ),
        ),
        (ActionStatus.SUCCEEDED, _result(final_disposition=0)),
    ],
)
def test_failure_and_unknown_evidence_is_nonzero(status, result) -> None:
    """Every inconsistent, failed, or unknown terminal state fails closed."""
    decision = decide_terminal_result(
        status=status,
        result=result,
        expected_step_count=2,
    )
    assert not decision.success
    assert decision.exit_code != 0


def test_aborted_result_preserves_typed_backend_failure_detail() -> None:
    """Operator output names the typed cause without treating it as success evidence."""
    decision = decide_terminal_result(
        status=ActionStatus.ABORTED,
        result=_result(
            success=False,
            error_code=114,
            error_message="system health does not permit starting this transfer step",
        ),
        expected_step_count=2,
    )

    assert not decision.success
    assert decision.exit_code == 7
    assert "system health does not permit starting this transfer step" in decision.detail
    assert "error_code=114" in decision.detail


def test_cancel_policy_targets_only_one_accepted_active_goal() -> None:
    """No broad cancel is requested before acceptance or after a result."""
    assert should_cancel_exact(
        shutdown_requested=True,
        goal_accepted=True,
        terminal_received=False,
    )
    assert not should_cancel_exact(
        shutdown_requested=True,
        goal_accepted=False,
        terminal_received=False,
    )
    assert not should_cancel_exact(
        shutdown_requested=True,
        goal_accepted=True,
        terminal_received=True,
    )


def test_wire_result_is_decoded_into_immutable_policy_evidence() -> None:
    """ROS result nesting stays outside the terminal decision policy."""
    action_result = TransferObjectSequenceV2.Result()
    action_result.result.success = True
    action_result.result.error_code = ManipulationTaskResult.NONE
    action_result.result.final_disposition = ManipulationTaskResult.OBJECT_AT_TARGET
    action_result.completed_step_count = 1
    action_result.failed_step_index = TransferObjectSequenceV2.Result.NO_FAILED_STEP
    step_result = ManipulationTaskResult()
    step_result.success = True
    step_result.error_code = ManipulationTaskResult.NONE
    step_result.final_disposition = ManipulationTaskResult.OBJECT_AT_TARGET
    action_result.step_results = [step_result]

    snapshot = decode_sequence_result(action_result)

    assert snapshot.success
    assert snapshot.completed_step_count == 1
    assert snapshot.failed_step_index == -1
    assert snapshot.step_results == (StepResultSnapshot(True, 0, 4),)
