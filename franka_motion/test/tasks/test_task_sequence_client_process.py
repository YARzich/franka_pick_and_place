"""
Test the common transfer-client process lifecycle without a ROS graph.

Responsibility:
- Preserve cooperative signal ownership and deterministic context cleanup.
"""

from franka_motion_runtime.tasks import sequence_client_process as process


class _Logger:
    """Capture process-shell errors."""

    def __init__(self) -> None:
        self.errors = []

    def error(self, message: str) -> None:
        """Record one error."""
        self.errors.append(message)


class _Node:
    """Track lifecycle calls made by the common process shell."""

    instances = []

    def __init__(self, name: str) -> None:
        self.name = name
        self.destroyed = False
        self.logger = _Logger()
        self.instances.append(self)

    def get_logger(self):
        """Return the capturing logger."""
        return self.logger

    def destroy_node(self) -> None:
        """Record deterministic node cleanup."""
        self.destroyed = True


def _patch_process_environment(monkeypatch):
    """Replace signals, rclpy, and Node with deterministic fakes."""
    events = []
    monkeypatch.setattr(process, "Node", _Node)
    monkeypatch.setattr(process.signal, "getsignal", lambda signum: f"old-{signum}")
    monkeypatch.setattr(
        process.signal,
        "signal",
        lambda signum, handler: events.append((signum, handler)),
    )
    monkeypatch.setattr(
        process.rclpy,
        "init",
        lambda **kwargs: events.append(("init", kwargs)),
    )
    monkeypatch.setattr(
        process.rclpy,
        "shutdown",
        lambda: events.append(("shutdown", None)),
    )
    _Node.instances.clear()
    return events


def test_process_shell_restores_signals_and_ros_context(monkeypatch) -> None:
    """Normal completion destroys the node and restores both handlers."""
    events = _patch_process_environment(monkeypatch)
    seen = {}

    def runner(node, shutdown):
        seen.update(node=node, shutdown=shutdown)
        return 7

    assert process.run_client_process("test_client", runner, args=["arg"]) == 7
    node = _Node.instances[0]
    assert seen["node"] is node
    assert not seen["shutdown"].requested
    assert node.destroyed
    assert ("shutdown", None) in events
    assert events[-2:] == [
        (process.signal.SIGINT, f"old-{process.signal.SIGINT}"),
        (process.signal.SIGTERM, f"old-{process.signal.SIGTERM}"),
    ]


def test_process_shell_reports_runner_exception_and_cleans_up(monkeypatch) -> None:
    """Unexpected adapter errors remain non-zero without leaking process state."""
    _patch_process_environment(monkeypatch)

    def runner(_node, _shutdown):
        raise RuntimeError("broken adapter")

    assert process.run_client_process("test_client", runner) == 9
    node = _Node.instances[0]
    assert node.destroyed
    assert "broken adapter" in node.logger.errors[-1]
