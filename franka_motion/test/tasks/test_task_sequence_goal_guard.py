"""
Test delayed goal-response containment by exact action UUID.

Responsibility:
- Cover late acceptance, raw cancel evidence, result races, and retry ownership.
"""

from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from manipulation_msgs.action import TransferObjectSequenceV2
from manipulation_msgs.msg import ManipulationTaskResult

from franka_motion_runtime.tasks.sequence_client_waits import ShutdownRequest
from franka_motion_runtime.tasks.sequence_goal_guard import (
    TargetedCancelEvidence,
    _classify_cancel,
    reconcile_late_goal_response,
)
from sequence_client_test_support import (
    FakeFuture,
    FakeNode,
    FakeServiceClient,
    make_cancel_response,
    make_terminal_response,
    make_uuid,
)


def test_delayed_accepted_response_is_reconciled_after_targeted_cancel() -> None:
    """A response arriving during cancel wait returns its exact accepted handle."""
    goal_uuid = make_uuid()
    goal_handle = SimpleNamespace(accepted=True)
    send_future = FakeFuture(done=False, kind="send")
    cancel_future = FakeFuture(
        make_cancel_response(goal_uuid, accepted=True),
        kind="cancel",
    )
    node = FakeNode(cancel_client=FakeServiceClient([cancel_future]))

    def wait_future(_node, future, **_kwargs):
        if future.kind == "cancel":
            send_future.value = goal_handle
            send_future.completed = True
        return future.done()

    resolution = reconcile_late_goal_response(
        node,
        send_future,
        action_name="/transfer_object_sequence_v2",
        goal_uuid=goal_uuid,
        shutdown=ShutdownRequest(),
        wait_future=wait_future,
    )

    assert resolution.response_received
    assert resolution.goal_handle is goal_handle
    assert resolution.targeted_cancel_confirmed
    assert bytes(node.cancel_client.requests[0].goal_info.goal_id.uuid) == bytes(
        goal_uuid.uuid
    )


def test_diagnostic_timeout_retries_until_exact_terminal_evidence() -> None:
    """Unknown admission never permits process exit with an orphanable goal."""
    goal_uuid = make_uuid()
    cancel_futures = [
        FakeFuture(make_cancel_response(goal_uuid, accepted=False), kind="cancel"),
        FakeFuture(make_cancel_response(goal_uuid, accepted=True), kind="cancel"),
    ]
    result_client = FakeServiceClient(
        [FakeFuture(make_terminal_response(), kind="result")]
    )
    node = FakeNode(
        cancel_client=FakeServiceClient(cancel_futures),
        result_client=result_client,
    )

    resolution = reconcile_late_goal_response(
        node,
        FakeFuture(done=False, kind="send"),
        action_name="/transfer_object_sequence_v2",
        goal_uuid=goal_uuid,
        shutdown=ShutdownRequest(),
        wait_future=lambda _node, future, **_kwargs: future.done(),
    )

    assert not resolution.response_received
    assert resolution.targeted_cancel_confirmed
    assert resolution.terminal_received
    assert len(node.cancel_client.requests) == 2


def test_acceptance_during_second_cancel_wait_returns_late_handle() -> None:
    """A final future read retains a goal accepted during the retry cancel."""
    goal_uuid = make_uuid()
    goal_handle = SimpleNamespace(accepted=True)
    send_future = FakeFuture(done=False, kind="send")
    cancel_client = FakeServiceClient(
        [
            FakeFuture(
                make_cancel_response(goal_uuid, accepted=False),
                kind="cancel1",
            ),
            FakeFuture(
                make_cancel_response(goal_uuid, accepted=True),
                kind="cancel2",
            ),
        ]
    )
    node = FakeNode(cancel_client=cancel_client)

    def wait_future_stub(_node, future, **_kwargs):
        if future.kind == "cancel2":
            send_future.value = goal_handle
            send_future.completed = True
        return future.done()

    resolution = reconcile_late_goal_response(
        node,
        send_future,
        action_name="/transfer_object_sequence_v2",
        goal_uuid=goal_uuid,
        shutdown=ShutdownRequest(),
        wait_future=wait_future_stub,
    )

    assert resolution.response_received
    assert resolution.goal_handle is goal_handle
    assert resolution.targeted_cancel_confirmed


def test_cancel_result_race_is_resolved_by_exact_uuid_result_service() -> None:
    """Confirmed cancel remains observable without a delayed goal handle."""
    goal_uuid = make_uuid()
    cancel_client = FakeServiceClient(
        [
            FakeFuture(
                make_cancel_response(goal_uuid, accepted=True),
                kind="cancel",
            )
        ]
    )
    result_client = FakeServiceClient(
        [FakeFuture(make_terminal_response(), kind="result")]
    )
    node = FakeNode(cancel_client=cancel_client, result_client=result_client)

    resolution = reconcile_late_goal_response(
        node,
        FakeFuture(done=False, kind="send"),
        action_name="/transfer_object_sequence_v2",
        goal_uuid=goal_uuid,
        shutdown=ShutdownRequest(),
        wait_future=lambda _node, future, **_kwargs: future.done(),
    )

    assert resolution.targeted_cancel_confirmed
    assert resolution.terminal_received
    assert resolution.terminal_status == GoalStatus.STATUS_CANCELED
    assert resolution.terminal_disposition == ManipulationTaskResult.OBJECT_AT_SOURCE


def test_raw_guard_uses_the_selected_action_get_result_contract() -> None:
    """Reusable containment never probes a different action's result service."""
    goal_uuid = make_uuid()
    node = FakeNode(
        cancel_client=FakeServiceClient(
            [FakeFuture(make_cancel_response(goal_uuid, accepted=True))]
        ),
        result_client=FakeServiceClient([FakeFuture(make_terminal_response())]),
    )

    resolution = reconcile_late_goal_response(
        node,
        FakeFuture(done=False),
        action_name="/transfer_object_sequence_v2",
        goal_uuid=goal_uuid,
        shutdown=ShutdownRequest(),
        wait_future=lambda _node, future, **_kwargs: future.done(),
        action_type=TransferObjectSequenceV2,
    )

    assert resolution.terminal_received
    request = node.result_client.requests[0]
    assert isinstance(
        request,
        TransferObjectSequenceV2.Impl.GetResultService.Request,
    )


def test_completed_send_future_exception_still_enters_uuid_containment() -> None:
    """A transport exception is unknown evidence, never an implicit rejection."""
    goal_uuid = make_uuid()
    cancel_client = FakeServiceClient(
        [FakeFuture(make_cancel_response(goal_uuid, accepted=True), kind="cancel")]
    )
    result_client = FakeServiceClient(
        [FakeFuture(make_terminal_response(), kind="result")]
    )
    node = FakeNode(cancel_client=cancel_client, result_client=result_client)

    resolution = reconcile_late_goal_response(
        node,
        FakeFuture(done=True, error=RuntimeError("transport")),
        action_name="/transfer_object_sequence_v2",
        goal_uuid=goal_uuid,
        shutdown=ShutdownRequest(),
        wait_future=lambda _node, future, **_kwargs: future.done(),
    )

    assert not resolution.response_received
    assert resolution.targeted_cancel_confirmed
    assert resolution.terminal_received


def test_cancel_return_codes_remain_typed_evidence() -> None:
    """Known terminal, rejected, and admission-race responses stay distinct."""
    goal_uuid = make_uuid()
    expected = {
        CancelGoal.Response.ERROR_GOAL_TERMINATED:
            TargetedCancelEvidence.GOAL_TERMINATED,
        CancelGoal.Response.ERROR_REJECTED: TargetedCancelEvidence.REJECTED,
        CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID:
            TargetedCancelEvidence.UNKNOWN_GOAL,
    }
    for return_code, evidence in expected.items():
        response = CancelGoal.Response(return_code=return_code)
        assert _classify_cancel(response, goal_uuid) is evidence


def test_raw_guard_keeps_at_most_one_pending_cancel_request() -> None:
    """Diagnostic cycles reuse one pending raw CancelGoal service future."""
    goal_uuid = make_uuid()
    cancel_future = FakeFuture(done=False, kind="cancel")
    cancel_client = FakeServiceClient([cancel_future])
    node = FakeNode(cancel_client=cancel_client)
    send_future = FakeFuture(done=False, kind="send")
    send_waits = 0
    goal_handle = SimpleNamespace(accepted=True)

    def wait_stub(_node, future, **_kwargs):
        nonlocal send_waits
        if future.kind == "cancel":
            return False
        send_waits += 1
        if send_waits == 2:
            send_future.value = goal_handle
            send_future.completed = True
        return future.done()

    resolution = reconcile_late_goal_response(
        node,
        send_future,
        action_name="/transfer_object_sequence_v2",
        goal_uuid=goal_uuid,
        shutdown=ShutdownRequest(),
        wait_future=wait_stub,
    )

    assert resolution.goal_handle is goal_handle
    assert len(cancel_client.requests) == 1
