"""
Test accepted sequence-goal runtime, exact cancellation, and terminal handling.

Responsibility:
- Cover shutdown admission, terminal validation, handle cancellation, and naming.
"""

from types import SimpleNamespace

from action_msgs.msg import GoalStatus
import pytest

from franka_motion_runtime.tasks.sequence_client_cancel import (
    cancel_after_interruption,
    cancel_exact_goal,
)
from franka_motion_runtime.tasks.sequence_client_runtime import resolve_action_name
from franka_motion_runtime.tasks.sequence_client_waits import (
    ShutdownRequest,
)
from franka_motion_runtime.tasks.sequence_terminal import (
    TerminalReadState,
    read_terminal_future,
)
from sequence_client_test_support import (
    FakeFuture,
    FakeNode,
    make_cancel_response,
    make_terminal_response,
    make_uuid,
)


def test_terminal_reader_rejects_exception_and_nonterminal_done_future() -> None:
    """Future completion alone never releases normal-path goal ownership."""
    failed = read_terminal_future(
        FakeFuture(done=True, error=RuntimeError("result transport"))
    )
    executing = read_terminal_future(
        FakeFuture(make_terminal_response(status=GoalStatus.STATUS_EXECUTING))
    )

    assert failed.state is TerminalReadState.INVALID
    assert executing.state is TerminalReadState.NONTERMINAL


def test_accepted_exact_cancel_waits_terminal_and_logs_actual_status() -> None:
    """Cancel acknowledgement alone is insufficient without terminal evidence."""
    goal_uuid = make_uuid()
    goal_handle = SimpleNamespace(
        goal_id=goal_uuid,
        cancel_goal_async=lambda: FakeFuture(
            make_cancel_response(goal_uuid, accepted=True)
        ),
    )
    node = FakeNode()

    exit_code = cancel_after_interruption(
        node,
        goal_handle,
        FakeFuture(make_terminal_response()),
        shutdown=ShutdownRequest(),
        reason="unit test",
    )

    assert exit_code == 5
    assert any("disposition=2" in message for _, message in node.logger.messages)


def test_cancel_loop_reissues_result_after_nonterminal_done_future() -> None:
    """A done EXECUTING result is probed again until exact terminal evidence."""
    goal_uuid = make_uuid()
    terminal_future = FakeFuture(make_terminal_response())
    result_requests = []
    goal_handle = SimpleNamespace(
        goal_id=goal_uuid,
        cancel_goal_async=lambda: FakeFuture(
            make_cancel_response(goal_uuid, accepted=True)
        ),
        get_result_async=lambda: result_requests.append(True) or terminal_future,
    )

    outcome = cancel_exact_goal(
        FakeNode(),
        goal_handle,
        shutdown=ShutdownRequest(),
        result_future=FakeFuture(
            make_terminal_response(status=GoalStatus.STATUS_EXECUTING)
        ),
    )

    assert outcome.exact_goal_acknowledged
    assert outcome.requested_goal_uuid == bytes(goal_uuid.uuid)
    assert outcome.terminal_response.status == GoalStatus.STATUS_CANCELED
    assert len(result_requests) == 1


def test_handle_cancel_keeps_at_most_one_pending_request(monkeypatch) -> None:
    """A lost cancel response is not duplicated while its future stays pending."""
    goal_uuid = make_uuid()
    cancel_future = FakeFuture(done=False, kind="cancel")
    result_future = FakeFuture(done=False, kind="result")
    cancel_calls = 0
    result_waits = 0

    def cancel_goal_async():
        nonlocal cancel_calls
        cancel_calls += 1
        return cancel_future

    def wait_stub(_node, future, **_kwargs):
        nonlocal result_waits
        if future.kind == "cancel":
            return False
        result_waits += 1
        if result_waits == 2:
            future.value = make_terminal_response()
            future.completed = True
        return future.done()

    monkeypatch.setattr(
        "franka_motion_runtime.tasks.sequence_client_cancel.wait_future",
        wait_stub,
    )
    goal_handle = SimpleNamespace(
        goal_id=goal_uuid,
        cancel_goal_async=cancel_goal_async,
        get_result_async=lambda: pytest.fail("result future was replaced"),
    )
    outcome = cancel_exact_goal(
        FakeNode(),
        goal_handle,
        shutdown=ShutdownRequest(),
        result_future=result_future,
    )

    assert outcome.terminal_response.status == GoalStatus.STATUS_CANCELED
    assert cancel_calls == 1


def test_action_name_resolution_is_shared_with_raw_guard_services() -> None:
    """ROS remapping is resolved before deriving internal action services."""
    node = SimpleNamespace(
        resolve_topic_name=lambda name: "/remapped_sequence"
        if name == "/transfer_object_sequence_v2"
        else name
    )
    assert resolve_action_name(node, "/transfer_object_sequence_v2") == (
        "/remapped_sequence"
    )
