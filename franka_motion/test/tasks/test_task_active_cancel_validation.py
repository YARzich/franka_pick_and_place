"""
Test pure active-cancel evidence validation without ROS runtime.

Responsibility:
- Pin the exact feedback, UUID, result, coordinator, and scene contracts.

Does not own:
- Action transport, service calls, simulator startup, or MoveIt execution.
"""

from dataclasses import replace

from action_msgs.msg import GoalStatus
from manipulation_msgs.msg import ManipulationTaskResult
from manipulation_msgs.msg import ManipulationTaskState
import pytest

from franka_motion_runtime.tasks.active_cancel_validation import (
    ActiveCancelCheckpoint,
    ControllerGoalSnapshot,
    CoordinatorStateSnapshot,
    PlanningSceneSnapshot,
    is_active_cancel_feedback,
    validate_canceled_sequence_result,
    validate_coordinator_quiescence,
    validate_exact_cancel_ack,
    validate_scene_cleanup,
)
from franka_motion_runtime.tasks.sequence_client_policy import (
    SequenceResultSnapshot,
    StepResultSnapshot,
)


def _canceled_result() -> SequenceResultSnapshot:
    """Return the exact one-step canceled-at-source terminal contract."""
    step = StepResultSnapshot(
        success=False,
        error_code=ManipulationTaskResult.CANCELED,
        final_disposition=ManipulationTaskResult.OBJECT_AT_SOURCE,
    )
    return SequenceResultSnapshot(
        success=False,
        error_code=ManipulationTaskResult.CANCELED,
        error_message="canceled during approach",
        final_disposition=ManipulationTaskResult.OBJECT_AT_SOURCE,
        completed_step_count=0,
        failed_step_index=0,
        step_results=(step,),
    )


def _quiescent_state() -> CoordinatorStateSnapshot:
    """Return a ready coordinator with no retained cancel ownership."""
    return CoordinatorStateSnapshot(
        active_lease_count=0,
        operation_in_progress=False,
        backend_service_ready=True,
        reconciliation_required=False,
        unreconciled_lease_expiry_count=0,
        operation_capacity_exhausted=False,
        quarantined_resource_count=0,
        quarantined_resource_keys=(),
    )


def test_feedback_trigger_requires_first_step_approaching() -> None:
    """Planning feedback or another step must never trigger active cancel."""
    assert is_active_cancel_feedback(
        sequence_step_index=0,
        phase=ManipulationTaskState.APPROACHING,
    )
    assert not is_active_cancel_feedback(
        sequence_step_index=0,
        phase=ManipulationTaskState.PLANNING,
    )
    assert not is_active_cancel_feedback(
        sequence_step_index=1,
        phase=ManipulationTaskState.APPROACHING,
    )


def test_exact_cancel_ack_rejects_another_goal_uuid() -> None:
    """Any-goal acknowledgement cannot satisfy exact-goal containment."""
    requested = bytes([7] * 16)
    assert not validate_exact_cancel_ack(requested, (requested,))
    assert validate_exact_cancel_ack(requested, (bytes([8] * 16),))


def _checkpoint():
    """Start one approach with a fresh controller goal after submission."""
    checkpoint = ActiveCancelCheckpoint(submitted_stamp_ns=100)
    checkpoint.observe_feedback(
        sequence_step_index=0, phase=ManipulationTaskState.APPROACHING
    )
    goal = ControllerGoalSnapshot(bytes([7] * 16), 101, GoalStatus.STATUS_EXECUTING)
    checkpoint.observe_status((goal,), received_at=5.0)
    return checkpoint, goal


def _execution(checkpoint, now=5.0):
    """Use a bounded steady-clock status receipt age."""
    return checkpoint.execution_evidence(now=now, max_status_age_sec=1.0)


def test_fresh_post_submission_executing_goal_proves_checkpoint():
    """Return the exact controller UUID only after both current observations."""
    checkpoint, goal = _checkpoint()
    assert _execution(checkpoint) == goal


@pytest.mark.parametrize("stamp", [0, 99, 100])
def test_retained_pre_submission_controller_goals_do_not_prove_execution(stamp):
    """A transient-local status from before this sequence is not evidence."""
    checkpoint, goal = _checkpoint()
    checkpoint.observe_status((replace(goal, accepted_stamp_ns=stamp),), received_at=5.0)
    assert _execution(checkpoint) is None


@pytest.mark.parametrize("status", [
    GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_CANCELING,
    GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
])
def test_later_nonexecuting_status_removes_execution_evidence(status):
    """A terminal update cannot leave earlier EXECUTING evidence latched."""
    checkpoint, goal = _checkpoint()
    checkpoint.observe_status((replace(goal, status=status),), received_at=5.1)
    assert _execution(checkpoint, now=5.1) is None


def test_empty_status_removes_execution_evidence():
    """An empty full status snapshot also clears the old active goal."""
    checkpoint, _ = _checkpoint()
    checkpoint.observe_status((), received_at=5.1)
    assert _execution(checkpoint, now=5.1) is None


@pytest.mark.parametrize(("step", "phase"), [
    (0, ManipulationTaskState.PLANNING), (1, ManipulationTaskState.APPROACHING),
])
def test_current_phase_must_still_be_first_approach(step, phase):
    """Controller execution cannot revive an earlier approach checkpoint."""
    checkpoint, _ = _checkpoint()
    checkpoint.observe_feedback(sequence_step_index=step, phase=phase)
    assert _execution(checkpoint) is None


@pytest.mark.parametrize("now", [4.9, 6.01])
def test_status_must_be_recent_on_steady_clock(now):
    """Unknown future receipt or stale status cannot authorize cancellation."""
    checkpoint, _ = _checkpoint()
    assert _execution(checkpoint, now=now) is None


def test_two_new_executing_goals_are_ambiguous():
    """Unrelated concurrent controller work cannot produce one UUID proof."""
    checkpoint, goal = _checkpoint()
    checkpoint.observe_status(
        (goal, replace(goal, goal_uuid=bytes([8] * 16))), received_at=5.0
    )
    assert _execution(checkpoint) is None


@pytest.mark.parametrize(
    "invalid_result",
    [
        replace(_canceled_result(), success=True),
        replace(_canceled_result(), completed_step_count=1),
        replace(_canceled_result(), failed_step_index=-1),
        replace(_canceled_result(), step_results=()),
        replace(
            _canceled_result(),
            step_results=(
                replace(_canceled_result().step_results[0], success=True),
            ),
        ),
    ],
)
def test_terminal_validation_rejects_incomplete_cancel_evidence(
    invalid_result: SequenceResultSnapshot,
) -> None:
    """Every aggregate and per-step typed field is required."""
    assert validate_canceled_sequence_result(
        status=GoalStatus.STATUS_CANCELED,
        result=invalid_result,
    )


def test_terminal_validation_accepts_exact_canceled_at_source_contract() -> None:
    """One canceled first step with no successful prefix is valid evidence."""
    assert not validate_canceled_sequence_result(
        status=GoalStatus.STATUS_CANCELED,
        result=_canceled_result(),
    )


@pytest.mark.parametrize(
    "state",
    [
        replace(_quiescent_state(), active_lease_count=1),
        replace(_quiescent_state(), operation_in_progress=True),
        replace(_quiescent_state(), backend_service_ready=False),
        replace(_quiescent_state(), reconciliation_required=True),
        replace(_quiescent_state(), unreconciled_lease_expiry_count=1),
        replace(_quiescent_state(), operation_capacity_exhausted=True),
        replace(
            _quiescent_state(),
            quarantined_resource_count=1,
            quarantined_resource_keys=("world:cube",),
        ),
    ],
)
def test_coordinator_validation_rejects_retained_or_uncertain_state(
    state: CoordinatorStateSnapshot,
) -> None:
    """No leak, recovery gate, quarantine, or backend outage is tolerated."""
    assert validate_coordinator_quiescence(state)


def test_scene_validation_preserves_source_object_and_table() -> None:
    """Releasing a lease must preserve real world geometry after approach cancel."""
    clean = PlanningSceneSnapshot(
        world_object_ids=frozenset({"cube", "table"}),
        attached_object_ids=frozenset(),
    )
    assert not validate_scene_cleanup(
        clean,
        canceled_object_id="cube",
        persistent_world_object_ids=("table",),
    )
    dirty = PlanningSceneSnapshot(
        world_object_ids=frozenset(),
        attached_object_ids=frozenset({"cube"}),
    )
    errors = validate_scene_cleanup(
        dirty,
        canceled_object_id="cube",
        persistent_world_object_ids=("table",),
    )
    assert len(errors) == 3


def test_scene_validation_rejects_missing_source_geometry() -> None:
    """A canceled physical object cannot disappear from collision checking."""
    missing_object = PlanningSceneSnapshot(
        world_object_ids=frozenset({"table"}),
        attached_object_ids=frozenset(),
    )
    assert validate_scene_cleanup(
        missing_object,
        canceled_object_id="cube",
        persistent_world_object_ids=("table",),
    )
