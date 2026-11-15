"""
Test exact process-death targeting and recovery without ROS or a simulator.

Responsibility:
- Prove one role child can die/restart while unrelated processes survive.
- Prove duplicate role ownership fails before any signal is sent.
"""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

from franka_motion_runtime.validation.runtime_process_fault import (
    RoleProcessControl,
)
from franka_motion_runtime.validation.runtime_process_role import (
    CONTROL_DIRECTORY_VARIABLE,
    PROCESS_ROLE_VARIABLE,
    ProcessRoleError,
    RoleStateStore,
)
from franka_motion_runtime.validation.runtime_registry import RUN_TOKEN_VARIABLE

_ROLE = "test_action_backend"
_SLEEP_COMMAND = (
    sys.executable,
    "-c",
    "import time; time.sleep(60)",
)


class _ControllerFixture:
    """Own a real controller and every extra test process."""

    def __init__(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        """Start one role controller with a unique test token."""
        self.run_token = f"cpu-process-fault-{os.getpid()}-{time.monotonic_ns()}"
        self.control_directory = tmp_path / "control"
        monkeypatch.setenv(RUN_TOKEN_VARIABLE, self.run_token)
        monkeypatch.setenv(
            CONTROL_DIRECTORY_VARIABLE,
            str(self.control_directory),
        )
        environment = dict(os.environ)
        self.controller = subprocess.Popen(
            (
                sys.executable,
                "-m",
                "franka_motion_runtime.validation.runtime_process_fault",
                "serve",
                "--role",
                _ROLE,
                "--",
                *_SLEEP_COMMAND,
            ),
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.extras: list[subprocess.Popen[bytes]] = []
        self.store = RoleStateStore(self.control_directory, _ROLE)
        self._wait_until_running()

    def start_extra(self, *, role: str) -> subprocess.Popen[bytes]:
        """Start one token-owned process that the controller does not own."""
        environment = dict(os.environ)
        environment[RUN_TOKEN_VARIABLE] = self.run_token
        environment[PROCESS_ROLE_VARIABLE] = role
        process = subprocess.Popen(
            _SLEEP_COMMAND,
            env=environment,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        self.extras.append(process)
        return process

    def close(self) -> None:
        """Terminate all test-owned sessions without name-based cleanup."""
        for process in reversed(self.extras):
            _terminate_session(process)
        _terminate_session(self.controller)

    def _wait_until_running(self) -> None:
        """Wait for the first atomic controller snapshot."""
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if self.controller.poll() is not None:
                output = self.controller.stdout.read().decode("utf-8", errors="replace")
                raise AssertionError(f"controller exited early: {output}")
            if self.store.path.exists():
                state = self.store.read()
                if state.status == "running" and state.generation == 1:
                    return
            time.sleep(0.02)
        raise AssertionError("controller did not publish running state")


@pytest.fixture
def controller_fixture(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    """Yield one real CPU-only role controller and guarantee cleanup."""
    fixture = _ControllerFixture(tmp_path, monkeypatch)
    try:
        yield fixture
    finally:
        fixture.close()


def test_exact_role_child_dies_and_restarts_without_touching_other_role(
    controller_fixture: _ControllerFixture,
) -> None:
    """SIGKILL and restart must affect only the exact marked backend child."""
    unrelated = controller_fixture.start_extra(role="unrelated_backend")
    control = RoleProcessControl(role=_ROLE, timeout_sec=3.0)

    stopped = control.kill()

    assert stopped.status == "stopped"
    assert stopped.return_code == -signal.SIGKILL
    assert unrelated.poll() is None

    restarted = control.restart()

    assert restarted.status == "running"
    assert restarted.generation == 2
    assert restarted.child is not None
    assert stopped.child is not None
    assert restarted.child.pid != stopped.child.pid
    assert unrelated.poll() is None


def test_duplicate_exact_role_fails_before_signal(
    controller_fixture: _ControllerFixture,
) -> None:
    """Ambiguous token/role ownership must leave both candidate PIDs alive."""
    duplicate = controller_fixture.start_extra(role=_ROLE)
    original = controller_fixture.store.read()
    assert original.child is not None
    control = RoleProcessControl(role=_ROLE, timeout_sec=1.0)

    with pytest.raises(ProcessRoleError, match="ambiguous"):
        control.kill()

    assert duplicate.poll() is None
    assert Path(f"/proc/{original.child.pid}").exists()
    assert controller_fixture.store.read().status == "running"


def test_role_duplicated_after_pidfd_open_fails_before_signal(
    controller_fixture: _ControllerFixture,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A candidate appearing after pidfd_open must make injection fail closed."""
    original = controller_fixture.store.read()
    assert original.child is not None
    real_pidfd_open = os.pidfd_open
    duplicate: subprocess.Popen[bytes] | None = None

    def open_then_duplicate(pid: int, flags: int) -> int:
        """Open the real target pidfd, then create the ambiguity under test."""
        nonlocal duplicate
        descriptor = real_pidfd_open(pid, flags)
        duplicate = controller_fixture.start_extra(role=_ROLE)
        return descriptor

    monkeypatch.setattr(os, "pidfd_open", open_then_duplicate)

    with pytest.raises(ProcessRoleError, match="ambiguous"):
        RoleProcessControl(role=_ROLE, timeout_sec=1.0).kill()

    assert duplicate is not None
    assert duplicate.poll() is None
    assert Path(f"/proc/{original.child.pid}").exists()
    assert controller_fixture.store.read().status == "running"


def test_status_rejects_state_from_another_run(
    controller_fixture: _ControllerFixture,
) -> None:
    """A valid PID is insufficient when the persisted run token differs."""
    state = controller_fixture.store.read()
    controller_fixture.store.write(
        type(state)(
            role=state.role,
            run_token="different-run-token",
            controller=state.controller,
            child=state.child,
            generation=state.generation,
            status=state.status,
            return_code=state.return_code,
        )
    )

    with pytest.raises(ProcessRoleError, match="does not belong"):
        RoleProcessControl(role=_ROLE, timeout_sec=1.0).status("running")


def _terminate_session(process: subprocess.Popen[bytes]) -> None:
    """Join one explicitly owned test session with bounded escalation."""
    if process.poll() is not None:
        process.wait()
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=3.0)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait(timeout=2.0)
