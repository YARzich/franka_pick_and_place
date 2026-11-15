"""
Validate typed evidence from the active transfer-cancellation runtime probe.

Responsibility:
- Describe immutable coordinator and PlanningScene snapshots.
- Check the exact feedback, cancel, terminal-result, and cleanup contracts.

Does not own:
- ROS clients, action transport, waiting, logging, or simulator geometry.
"""

from __future__ import annotations

from dataclasses import dataclass

from action_msgs.msg import GoalStatus
from manipulation_msgs.msg import ManipulationTaskResult
from manipulation_msgs.msg import ManipulationTaskState

from franka_motion_runtime.tasks.sequence_client_policy import (
    SequenceResultSnapshot,
)


@dataclass(frozen=True)
class CoordinatorStateSnapshot:
    """Contain coordinator fields that prove post-cancel quiescence."""

    active_lease_count: int
    operation_in_progress: bool
    backend_service_ready: bool
    reconciliation_required: bool
    unreconciled_lease_expiry_count: int
    operation_capacity_exhausted: bool
    quarantined_resource_count: int
    quarantined_resource_keys: tuple[str, ...]


@dataclass(frozen=True)
class PlanningSceneSnapshot:
    """Contain only world and attached object identities."""

    world_object_ids: frozenset[str]
    attached_object_ids: frozenset[str]


def is_active_cancel_feedback(*, sequence_step_index: int, phase: int) -> bool:
    """Match only step zero after the backend has entered APPROACHING."""
    return (
        sequence_step_index == 0
        and phase == ManipulationTaskState.APPROACHING
    )


@dataclass(frozen=True)
class ControllerGoalSnapshot:
    """Identify a controller goal and its most recently observed status."""

    goal_uuid: bytes
    accepted_stamp_ns: int
    status: int


class ActiveCancelCheckpoint:
    """Combine current task phase with fresh post-submission execution proof."""

    def __init__(self, submitted_stamp_ns: int = 0) -> None:
        """Start unarmed until the sequence submission establishes a baseline."""
        self.submitted_stamp_ns = submitted_stamp_ns
        self._approaching = False
        self._goals: tuple[ControllerGoalSnapshot, ...] = ()
        self._received_at: float | None = None

    def observe_feedback(self, *, sequence_step_index: int, phase: int) -> None:
        """Replace phase evidence instead of latching a previous approach."""
        self._approaching = is_active_cancel_feedback(
            sequence_step_index=sequence_step_index, phase=phase
        )

    def observe_status(
        self,
        goals: tuple[ControllerGoalSnapshot, ...],
        *,
        received_at: float,
    ) -> None:
        """Replace the status snapshot, including terminal and empty updates."""
        self._goals = goals
        self._received_at = received_at

    def execution_evidence(
        self, *, now: float, max_status_age_sec: float
    ) -> ControllerGoalSnapshot | None:
        """Require exactly one fresh EXECUTING goal accepted after submission."""
        if (
            self.submitted_stamp_ns <= 0
            or not self._approaching
            or self._received_at is None
            or not 0.0 <= now - self._received_at <= max_status_age_sec
        ):
            return None
        executing = tuple(
            goal for goal in self._goals
            if goal.status == GoalStatus.STATUS_EXECUTING
            and goal.accepted_stamp_ns > self.submitted_stamp_ns
            and len(goal.goal_uuid) == 16
            and any(goal.goal_uuid)
        )
        return executing[0] if len(executing) == 1 else None


def validate_exact_cancel_ack(
    requested_goal_uuid: bytes,
    acknowledged_goal_uuids: tuple[bytes, ...],
) -> tuple[str, ...]:
    """Require cancel acknowledgement for the submitted UUID, not any goal."""
    if requested_goal_uuid in acknowledged_goal_uuids:
        return ()
    return ("cancel acknowledgement does not contain the submitted goal UUID",)


def validate_canceled_sequence_result(
    *,
    status: int,
    result: SequenceResultSnapshot,
) -> tuple[str, ...]:
    """Check the one-step canceled-at-source sequence terminal contract."""
    errors: list[str] = []
    expected_error = ManipulationTaskResult.CANCELED
    expected_disposition = ManipulationTaskResult.OBJECT_AT_SOURCE
    if status != GoalStatus.STATUS_CANCELED:
        errors.append(f"terminal status is {status}, expected CANCELED")
    if result.success:
        errors.append("aggregate result unexpectedly reports success")
    if result.error_code != expected_error:
        errors.append(
            f"aggregate error_code is {result.error_code}, expected CANCELED"
        )
    if result.final_disposition != expected_disposition:
        errors.append(
            "aggregate final_disposition does not report OBJECT_AT_SOURCE"
        )
    if result.completed_step_count != 0:
        errors.append("completed_step_count is not zero")
    if result.failed_step_index != 0:
        errors.append("failed_step_index is not zero")
    if len(result.step_results) != 1:
        errors.append("step_results does not contain exactly one result")
        return tuple(errors)

    step = result.step_results[0]
    if step.success:
        errors.append("step result unexpectedly reports success")
    if step.error_code != expected_error:
        errors.append("step error_code does not report CANCELED")
    if step.final_disposition != expected_disposition:
        errors.append("step final_disposition does not report OBJECT_AT_SOURCE")
    return tuple(errors)


def validate_coordinator_quiescence(
    state: CoordinatorStateSnapshot,
) -> tuple[str, ...]:
    """Reject leases, unresolved operations, quarantine, or backend loss."""
    errors: list[str] = []
    if state.active_lease_count != 0:
        errors.append("coordinator still has active leases")
    if state.operation_in_progress:
        errors.append("coordinator still has an operation in progress")
    if not state.backend_service_ready:
        errors.append("PlanningScene backend is not ready")
    if state.reconciliation_required:
        errors.append("coordinator still requires reconciliation")
    if state.unreconciled_lease_expiry_count != 0:
        errors.append("coordinator reports an unreconciled lease expiry")
    if state.operation_capacity_exhausted:
        errors.append("coordinator operation capacity is exhausted")
    if state.quarantined_resource_count != 0:
        errors.append("coordinator still has quarantined resources")
    if state.quarantined_resource_keys:
        errors.append("coordinator still reports quarantined resource keys")
    return tuple(errors)


def validate_scene_cleanup(
    scene: PlanningSceneSnapshot,
    *,
    canceled_object_id: str,
    persistent_world_object_ids: tuple[str, ...],
) -> tuple[str, ...]:
    """Keep the source object in world geometry after its lease is released."""
    errors: list[str] = []
    if canceled_object_id not in scene.world_object_ids:
        errors.append(f"'{canceled_object_id}' is missing from the world scene")
    if canceled_object_id in scene.attached_object_ids:
        errors.append(f"'{canceled_object_id}' remains attached")
    missing = sorted(
        set(persistent_world_object_ids) - set(scene.world_object_ids)
    )
    if missing:
        errors.append(
            "persistent world objects are missing: " + ", ".join(missing)
        )
    return tuple(errors)
