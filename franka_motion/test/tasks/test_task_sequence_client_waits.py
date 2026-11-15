"""
Test interruptible sequence-client waits without ROS runtime delays.

Responsibility:
- Prove discovery, clock, goal-response, and terminal waits have no elapsed deadline.
"""

from types import SimpleNamespace

import pytest

from franka_motion_runtime.tasks.sequence_client_config import (
    load_sequence_client_config,
)
from franka_motion_runtime.tasks.sequence_client_waits import (
    ShutdownRequest,
    goal_send_permitted,
    wait_for_nonzero_stamp,
    wait_for_server,
    wait_for_terminal_result,
    wait_future,
    wait_until_future_or_shutdown,
)
from sequence_client_test_support import FakeFuture


def test_sigint_is_recorded_while_goal_response_reconciliation_continues(
    monkeypatch,
) -> None:
    """A KeyboardInterrupt cannot abandon a send future before reconciliation."""
    shutdown = ShutdownRequest()
    future = FakeFuture(done=False)
    calls = 0

    def spin_once(_node, *, timeout_sec: float) -> None:
        nonlocal calls
        del timeout_sec
        calls += 1
        if calls == 1:
            raise KeyboardInterrupt
        future.completed = True

    monkeypatch.setattr(
        "franka_motion_runtime.tasks.sequence_client_waits.rclpy.spin_once",
        spin_once,
    )
    assert wait_future(
        object(),
        future,
        timeout_sec=1.0,
        shutdown=shutdown,
        honor_shutdown=False,
    )
    assert shutdown.requested


def test_accepted_result_wait_has_no_elapsed_deadline_or_wall_clock_delay(
    monkeypatch,
) -> None:
    """An accepted goal stays owned through arbitrary instant wait slices."""
    shutdown = ShutdownRequest()
    future = FakeFuture(done=False)
    spin_count = 0
    virtual_elapsed_sec = 0.0
    former_terminal_limit_sec = 660.0

    def spin_once(_node, *, timeout_sec: float) -> None:
        nonlocal spin_count, virtual_elapsed_sec
        assert timeout_sec > 0.0
        spin_count += 1
        virtual_elapsed_sec += timeout_sec
        if virtual_elapsed_sec > former_terminal_limit_sec:
            future.completed = True

    monkeypatch.setattr(
        "franka_motion_runtime.tasks.sequence_client_waits.rclpy.spin_once",
        spin_once,
    )
    monkeypatch.setattr(
        "franka_motion_runtime.tasks.sequence_client_waits.time.monotonic",
        lambda: pytest.fail("terminal wait must not consult elapsed time"),
    )

    assert wait_for_terminal_result(
        object(),
        future,
        shutdown=shutdown,
    )
    assert virtual_elapsed_sec > former_terminal_limit_sec
    assert spin_count > 13_000
    assert not shutdown.requested


def test_discovery_goal_response_and_clock_have_no_elapsed_deadline(
    monkeypatch,
) -> None:
    """Non-physical startup waits continue until evidence without wall time."""
    shutdown = ShutdownRequest()
    discovery_calls = 0
    discovery_elapsed_sec = 0.0
    former_preflight_limit_sec = 10.0

    class FakeActionClient:
        def wait_for_server(self, *, timeout_sec: float) -> bool:
            nonlocal discovery_calls, discovery_elapsed_sec
            assert timeout_sec > 0.0
            discovery_calls += 1
            discovery_elapsed_sec += timeout_sec
            return discovery_elapsed_sec > former_preflight_limit_sec

    monkeypatch.setattr(
        "franka_motion_runtime.tasks.sequence_client_waits.time.monotonic",
        lambda: pytest.fail("unbounded waits must not consult elapsed time"),
    )
    assert wait_for_server(FakeActionClient(), shutdown=shutdown)
    assert discovery_elapsed_sec > former_preflight_limit_sec
    assert discovery_calls >= 200

    response_future = FakeFuture(done=False)
    response_spins = 0
    response_elapsed_sec = 0.0
    former_goal_response_limit_sec = 5.0

    def spin_response(_node, *, timeout_sec: float) -> None:
        nonlocal response_spins, response_elapsed_sec
        assert timeout_sec > 0.0
        response_spins += 1
        response_elapsed_sec += timeout_sec
        if response_elapsed_sec > former_goal_response_limit_sec:
            response_future.completed = True

    monkeypatch.setattr(
        "franka_motion_runtime.tasks.sequence_client_waits.rclpy.spin_once",
        spin_response,
    )
    assert wait_until_future_or_shutdown(
        object(),
        response_future,
        shutdown=shutdown,
    )
    assert response_elapsed_sec > former_goal_response_limit_sec
    assert response_spins >= 100

    clock_reads = 0
    clock_elapsed_sec = 0.0

    class FakeClock:
        def now(self):
            nonlocal clock_reads
            clock_reads += 1
            nonzero = clock_elapsed_sec > former_preflight_limit_sec
            return SimpleNamespace(
                to_msg=lambda: SimpleNamespace(
                    sec=1 if nonzero else 0,
                    nanosec=0,
                )
            )

    def spin_clock(_node, *, timeout_sec: float) -> None:
        nonlocal clock_elapsed_sec
        assert timeout_sec > 0.0
        clock_elapsed_sec += timeout_sec

    monkeypatch.setattr(
        "franka_motion_runtime.tasks.sequence_client_waits.rclpy.spin_once",
        spin_clock,
    )
    clock_node = SimpleNamespace(get_clock=lambda: FakeClock())
    assert wait_for_nonzero_stamp(clock_node, shutdown=shutdown).sec == 1
    assert clock_elapsed_sec > former_preflight_limit_sec
    assert clock_reads > 200


def test_sequence_client_declares_no_wait_or_timeout_parameters() -> None:
    """The canonical client exposes intent and endpoint, not wait policy."""
    declared = []

    def declare_parameter(name, default):
        declared.append((name, default))
        return SimpleNamespace(value=default)

    config = load_sequence_client_config(
        SimpleNamespace(declare_parameter=declare_parameter)
    )

    assert config.action_name == "/transfer_object_sequence_v2"
    assert declared == [("action.name", "/transfer_object_sequence_v2")]


def test_shutdown_before_send_does_not_wait_for_or_contact_server() -> None:
    """The final admission gate forbids a new goal after preparation shutdown."""
    shutdown = ShutdownRequest()
    shutdown.request()
    assert not goal_send_permitted(shutdown)
