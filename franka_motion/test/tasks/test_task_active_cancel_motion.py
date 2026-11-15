"""
Verify the active-cancel controller observer and exact-goal containment.

Responsibility:
- Reject phase-only evidence and validate controller message/clock wiring.
- Keep the accepted sequence owned when the execution checkpoint times out.

Does not own:
- Simulator startup, physical motion, or the pure checkpoint policy.
"""

import importlib
from types import SimpleNamespace

from action_msgs.msg import GoalStatus, GoalStatusArray
from manipulation_msgs.msg import ManipulationTaskState
from rclpy.qos import qos_profile_action_status_default
from rclpy.task import Future
from rclpy.time import Time

from franka_motion_runtime.tasks import active_cancel_probe as probe
from franka_motion_runtime.tasks import active_cancel_motion as motion
from franka_motion_runtime.tasks.active_cancel_probe import _ActiveCancelFeedback
from franka_motion_runtime.tasks.active_cancel_validation import ControllerGoalSnapshot
from franka_motion_runtime.tasks.sequence_client_waits import ShutdownRequest


def _feedback(phase: int, step: int = 0):
    """Build only the task feedback fields consumed by the observer."""
    return SimpleNamespace(
        feedback=SimpleNamespace(
            sequence_step_index=step,
            current_state=SimpleNamespace(phase=phase),
        )
    )


def _observer(monkeypatch):
    """Keep logging outside the evidence regression."""
    module = importlib.import_module(_ActiveCancelFeedback.__module__)
    monkeypatch.setattr(module, "FeedbackLogger", lambda node: lambda value: None)
    return _ActiveCancelFeedback(SimpleNamespace())


def test_approaching_without_controller_execution_does_not_trigger(monkeypatch):
    """The pre-planning APPROACHING message is not physical execution proof."""
    observer = _observer(monkeypatch)
    observer(_feedback(ManipulationTaskState.APPROACHING))
    assert not observer.triggered


def test_departed_approach_phase_is_not_latched(monkeypatch):
    """An earlier approach phase must not authorize cancellation later."""
    observer = _observer(monkeypatch)
    observer(_feedback(ManipulationTaskState.APPROACHING))
    observer(_feedback(ManipulationTaskState.PLANNING))
    assert not observer.triggered


def test_controller_subscription_wires_uuid_status_and_terminal_update(monkeypatch):
    """Use real ROS messages and action QoS without starting a ROS graph."""
    subscription = object()
    calls = []
    destroyed = []

    def subscribe(message_type, topic, callback, qos):
        calls.append((message_type, topic, callback, qos))
        return subscription

    observer = _observer(monkeypatch)
    observer._node = SimpleNamespace(
        create_subscription=subscribe, destroy_subscription=destroyed.append
    )
    observer.connect_controller("/example_arm/follow_joint_trajectory")
    message_type, topic, callback, qos = calls[0]
    assert message_type is GoalStatusArray
    assert topic == "/example_arm/follow_joint_trajectory/_action/status"
    assert qos is qos_profile_action_status_default
    observer.mark_submission(1_000_000_000)
    observer(_feedback(ManipulationTaskState.APPROACHING))
    goal = GoalStatus(status=GoalStatus.STATUS_EXECUTING)
    goal.goal_info.goal_id.uuid = [7] * 16
    goal.goal_info.stamp = Time(seconds=2).to_msg()
    callback(GoalStatusArray(status_list=[goal]))
    assert observer.execution_evidence() == ControllerGoalSnapshot(
        bytes([7] * 16), 2_000_000_000, GoalStatus.STATUS_EXECUTING
    )
    goal.status = GoalStatus.STATUS_SUCCEEDED
    callback(GoalStatusArray(status_list=[goal]))
    assert not observer.triggered
    observer.close()
    observer.close()
    assert destroyed == [subscription]


def test_checkpoint_timeout_is_bounded_and_never_uses_stale_phase(monkeypatch):
    """A missing physical checkpoint consumes only the configured wait budget."""
    ticks = iter([0.0, 0.0, 0.1])
    monkeypatch.setattr(motion.time, "monotonic", lambda: next(ticks))
    waits = []
    monkeypatch.setattr(
        motion.rclpy, "spin_once", lambda node, *, timeout_sec: waits.append(timeout_sec)
    )
    evidence = motion._wait_for_cancel_checkpoint(
        SimpleNamespace(), SimpleNamespace(execution_evidence=lambda: None), Future(),
        timeout_sec=0.1, shutdown=ShutdownRequest(),
    )
    assert evidence is None
    assert waits == [0.05]


def test_checkpoint_timeout_contains_the_exact_accepted_sequence(monkeypatch):
    """Read a real Time property immediately before send and retain goal ownership."""
    events = []
    result_future = Future()
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: result_future)
    response = Future()
    response.set_result(handle)

    def send_goal(goal, **kwargs):
        events.append(("send", kwargs["goal_uuid"]))
        return response

    def cancel(node, goal_handle, **kwargs):
        events.append(("cancel", goal_handle, kwargs["result_future"]))

    logger = SimpleNamespace(error=lambda message: events.append(("error", message)))
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=3)),
        get_logger=lambda: logger,
    )
    feedback = SimpleNamespace(
        mark_submission=lambda stamp: events.append(("baseline", stamp))
    )
    monkeypatch.setattr(probe, "build_transfer_sequence_goal", lambda *a, **kw: object())
    monkeypatch.setattr(probe, "_wait_for_cancel_checkpoint", lambda *a, **kw: None)
    monkeypatch.setattr(probe, "cancel_exact_goal", cancel)
    result = probe._run_cancel_transaction(
        node, ShutdownRequest(), SimpleNamespace(send_goal_async=send_goal),
        feedback, "/sequence", (), Time(seconds=1).to_msg(),
        SimpleNamespace(feedback_timeout_sec=0.1),
    )
    assert result == 7
    assert events[0] == ("baseline", 3_000_000_000)
    assert events[1][0] == "send"
    assert events[2] == ("cancel", handle, result_future)
    assert events[3][0] == "error"
