"""
Lifecycle controller for one intentionally killable runtime backend.

Responsibility:
- Start the marked backend in its own process session and publish exact state.
- Keep the controller alive across an injected child death and restart request.
- Stop and join the child with bounded TERM/KILL escalation on controller exit.

Does not own:
- Target selection, pidfd signaling, ROS contracts or runtime evidence policy.
"""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import threading
from typing import Mapping, Sequence

from franka_motion_runtime.validation.runtime_process_role import (
    PROCESS_ROLE_VARIABLE,
    ProcessRoleError,
    RoleProcessState,
    RoleStateStore,
    current_process_identity,
    environment_contract,
)
from franka_motion_runtime.validation.runtime_registry import (
    ProcessIdentity,
    read_start_ticks,
)


class FaultableProcessController:
    """Own one replaceable child while remaining a stable launch process."""

    def __init__(
        self,
        *,
        role: str,
        argv: Sequence[str],
        environment: Mapping[str, str],
        poll_period_sec: float = 0.05,
    ) -> None:
        """Validate immutable child inputs and bind controller state."""
        run_token, control_directory = environment_contract(role, environment)
        if not argv or any(not isinstance(argument, str) or not argument for argument in argv):
            raise ProcessRoleError("faultable child argv must contain non-empty strings")
        if poll_period_sec <= 0.0:
            raise ProcessRoleError("poll_period_sec must be positive")
        self._role = role
        self._argv = tuple(argv)
        self._run_token = run_token
        self._environment = dict(environment)
        self._environment[PROCESS_ROLE_VARIABLE] = role
        self._poll_period_sec = poll_period_sec
        self._store = RoleStateStore(control_directory, role)
        self._controller_identity = current_process_identity()
        self._child: subprocess.Popen[bytes] | None = None
        self._child_identity = None
        self._generation = 0
        self._stop_requested = threading.Event()
        self._restart_requested = threading.Event()

    def run(self) -> int:
        """Serve until shutdown while making child death and restart observable."""
        previous_handlers = self._install_signal_handlers()
        try:
            self._start_child()
            while not self._stop_requested.wait(self._poll_period_sec):
                self._observe_child_exit()
                if self._child is None and self._restart_requested.is_set():
                    self._restart_requested.clear()
                    self._start_child()
            self._write_state(status="terminating", return_code=self._poll_child())
            self._stop_child()
            return 0
        finally:
            self._restore_signal_handlers(previous_handlers)

    def _install_signal_handlers(self) -> dict[int, object]:
        """Map launch shutdown and exact restart signals to local events."""
        previous = {}
        for signal_number, handler in (
            (signal.SIGINT, self._request_stop),
            (signal.SIGTERM, self._request_stop),
            (signal.SIGUSR1, self._request_restart),
        ):
            previous[signal_number] = signal.getsignal(signal_number)
            signal.signal(signal_number, handler)
        return previous

    @staticmethod
    def _restore_signal_handlers(previous: Mapping[int, object]) -> None:
        """Restore caller handlers for direct unit-test use."""
        for signal_number, handler in previous.items():
            signal.signal(signal_number, handler)

    def _request_stop(self, *_unused) -> None:
        """Request bounded child cleanup from SIGINT or SIGTERM."""
        self._stop_requested.set()

    def _request_restart(self, *_unused) -> None:
        """Request one new generation after a confirmed child exit."""
        self._restart_requested.set()

    def _start_child(self) -> None:
        """Create exactly one role-marked child in a new process session."""
        if self._child is not None:
            raise ProcessRoleError(f"role {self._role!r} already has a child")
        child = subprocess.Popen(
            self._argv,
            env=self._environment,
            start_new_session=True,
        )
        try:
            identity = _child_identity(child.pid)
        except Exception:
            _terminate_process_group(child)
            raise
        self._child = child
        self._child_identity = identity
        self._generation += 1
        self._write_state(status="running", return_code=None)

    def _observe_child_exit(self) -> None:
        """Join an exited child and retain its typed return code in state."""
        if self._child is None:
            return
        return_code = self._child.poll()
        if return_code is None:
            return
        self._child.wait()
        self._child = None
        self._write_state(status="stopped", return_code=return_code)

    def _poll_child(self) -> int | None:
        """Return the current child code without changing controller state."""
        return None if self._child is None else self._child.poll()

    def _stop_child(self) -> None:
        """Terminate and join a live child before the controller exits."""
        if self._child is None:
            return
        _terminate_process_group(self._child)
        return_code = self._child.returncode
        self._child = None
        self._write_state(status="stopped", return_code=return_code)

    def _write_state(self, *, status: str, return_code: int | None) -> None:
        """Publish one atomic lifecycle snapshot for fail-closed control."""
        self._store.write(
            RoleProcessState(
                role=self._role,
                run_token=self._run_token,
                controller=self._controller_identity,
                child=self._child_identity,
                generation=self._generation,
                status=status,
                return_code=return_code,
            )
        )


def _child_identity(pid: int) -> ProcessIdentity:
    """Read the new-session identity after Popen returns."""
    return ProcessIdentity(
        pid=pid,
        process_group_id=os.getpgid(pid),
        start_ticks=read_start_ticks(Path(f"/proc/{pid}/stat")),
    )


def _terminate_process_group(
    child: subprocess.Popen[bytes],
    *,
    terminate_timeout_sec: float = 5.0,
    kill_timeout_sec: float = 2.0,
) -> None:
    """Join one known child session through bounded TERM/KILL escalation."""
    if child.poll() is not None:
        child.wait()
        return
    try:
        os.killpg(child.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        child.wait(timeout=terminate_timeout_sec)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(child.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    try:
        child.wait(timeout=kill_timeout_sec)
    except subprocess.TimeoutExpired as error:
        raise ProcessRoleError("faultable child process group could not be joined") from error


def run_controller(role: str, argv: Sequence[str]) -> int:
    """Run one controller using the immutable current process environment."""
    controller = FaultableProcessController(
        role=role,
        argv=argv,
        environment=os.environ,
    )
    return controller.run()
