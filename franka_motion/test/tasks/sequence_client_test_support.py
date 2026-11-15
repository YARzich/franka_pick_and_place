"""
Provide deterministic ROS action/service fakes for sequence-client tests.

Responsibility:
- Build shared UUID, cancellation, terminal-result, node, and future fixtures.

Does not own:
- Lifecycle assertions or production behavior.
"""

from types import SimpleNamespace

from action_msgs.msg import GoalInfo, GoalStatus
from action_msgs.srv import CancelGoal
from manipulation_msgs.action import TransferObjectSequenceV2
from manipulation_msgs.msg import ManipulationTaskResult
from unique_identifier_msgs.msg import UUID


class FakeFuture:
    """Provide the minimal rclpy Future surface used by lifecycle helpers."""

    def __init__(
        self,
        result=None,
        *,
        done: bool = True,
        kind: str = "",
        error: Exception | None = None,
    ) -> None:
        self.value = result
        self.completed = done
        self.kind = kind
        self.error = error

    def done(self) -> bool:
        """Return deterministic completion state."""
        return self.completed

    def result(self):
        """Return the configured response or raise its transport error."""
        if self.error is not None:
            raise self.error
        return self.value


class FakeLogger:
    """Capture operator messages without a ROS logging backend."""

    def __init__(self) -> None:
        self.messages = []

    def info(self, message: str) -> None:
        """Record an informational message."""
        self.messages.append(("info", message))

    def warning(self, message: str) -> None:
        """Record a warning message."""
        self.messages.append(("warning", message))

    def error(self, message: str) -> None:
        """Record an error message."""
        self.messages.append(("error", message))


class FakeServiceClient:
    """Return queued service futures and retain exact requests."""

    def __init__(self, futures) -> None:
        self.futures = list(futures)
        self.requests = []

    def wait_for_service(self, *, timeout_sec: float) -> bool:
        """Report the fake service ready within any positive deadline."""
        del timeout_sec
        return True

    def service_is_ready(self) -> bool:
        """Report the fake service ready."""
        return True

    def call_async(self, request):
        """Record a request and return the next configured future."""
        self.requests.append(request)
        return self.futures.pop(0)


class FakeNode:
    """Route cancel/result service types to deterministic fake clients."""

    def __init__(self, cancel_client=None, result_client=None) -> None:
        self.cancel_client = cancel_client
        self.result_client = result_client
        self.destroyed = []
        self.logger = FakeLogger()

    def create_client(self, service_type, service_name):
        """Return the fake client matching the requested service type."""
        del service_name
        if service_type is CancelGoal:
            return self.cancel_client
        return self.result_client

    def destroy_client(self, client) -> None:
        """Record client destruction for ownership assertions."""
        self.destroyed.append(client)

    def get_logger(self) -> FakeLogger:
        """Return the capturing logger."""
        return self.logger


def make_uuid(seed: int = 7) -> UUID:
    """Return one non-zero deterministic goal UUID."""
    return UUID(uuid=[seed] * 16)


def make_cancel_response(
    goal_uuid: UUID,
    *,
    accepted: bool,
) -> CancelGoal.Response:
    """Build exact accepted or unknown-goal cancel evidence."""
    response = CancelGoal.Response()
    if accepted:
        response.return_code = CancelGoal.Response.ERROR_NONE
        response.goals_canceling = [GoalInfo(goal_id=goal_uuid)]
    else:
        response.return_code = CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID
    return response


def make_terminal_response(*, status: int = GoalStatus.STATUS_CANCELED):
    """Build one complete action result wrapper with a known disposition."""
    result = TransferObjectSequenceV2.Result()
    result.result.success = False
    result.result.error_code = ManipulationTaskResult.CANCELED
    result.result.final_disposition = ManipulationTaskResult.OBJECT_AT_SOURCE
    return SimpleNamespace(status=status, result=result)
