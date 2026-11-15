"""
Process-group helpers for pose backend subprocesses.

Responsibility:
- Start backend subprocesses in their own process group.
- Terminate a whole worker process group, including renderer/helper children.
- Keep bounded SIGTERM/SIGKILL cleanup behavior shared by pose subprocess clients.

Does not own:
- Backend command construction.
- Worker JSON protocol.
- ROS node shutdown orchestration.
"""

from __future__ import annotations

from collections.abc import Sequence
import os
import signal
import subprocess
import threading
import time
from typing import Any, TextIO


DEFAULT_TERMINATE_TIMEOUT_SEC = 2.0
DEFAULT_KILL_TIMEOUT_SEC = 2.0


class OwnedProcessGroup:
    """
    Keep the waitable leader as the identity anchor until group signaling ends.

    Only this owner may reap the child. Observing its exit uses WNOWAIT, and
    cleanup closes signaling before the final Popen.wait. A separate lock
    serializes cleanup with observers and repeated/concurrent cleanup calls.
    """

    def __init__(self, process: subprocess.Popen[str]) -> None:
        self._process = process
        self._lock = threading.Lock()
        self._returncode: int | None = None
        self._signals_closed = False
        self._ownership_lost = False
        self._reaped = False

    @property
    def pid(self) -> int:
        """Return the leader PID, retained for diagnostics after cleanup."""
        return self._process.pid

    @property
    def stdin(self) -> TextIO | None:
        return self._process.stdin

    @property
    def stdout(self) -> TextIO | None:
        return self._process.stdout

    @property
    def stderr(self) -> TextIO | None:
        return self._process.stderr

    @property
    def returncode(self) -> int | None:
        """Return the observed exit status without releasing the leader PID."""
        with self._lock:
            return self._returncode

    def poll_exit(self) -> int | None:
        """Observe exit without reaping; report lost child ownership as an error."""
        with self._lock:
            return self._poll_exit_locked()

    def wait_for_exit(self, timeout: float) -> int:
        """Wait for leader exit without blocking concurrent group cleanup."""
        deadline = time.monotonic() + max(0.0, float(timeout))
        while True:
            returncode = self.poll_exit()
            if returncode is not None:
                return returncode
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise subprocess.TimeoutExpired(self._process.args, timeout)
            time.sleep(min(0.02, remaining))

    def terminate(self, *, terminate_timeout_sec: float, kill_timeout_sec: float) -> bool:
        """
        Close group signaling and reap the leader within bounded waits.

        SIGTERM gives a live leader its grace period. After leader exit or the
        deadline, remaining group members receive SIGKILL before the leader is
        reaped. True confirms leader reap, not disappearance of every descendant.
        A later call may retry a timed-out reap, but never signal after closure.
        """
        terminate_timeout = max(0.0, float(terminate_timeout_sec))
        kill_timeout = max(0.0, float(kill_timeout_sec))
        with self._lock:
            if self._reaped:
                return True
            if self._ownership_lost:
                return False
            if not self._signals_closed:
                try:
                    if self._poll_exit_locked() is None:
                        self._signal_group_locked(signal.SIGTERM)
                        deadline = time.monotonic() + terminate_timeout
                        while self._poll_exit_locked() is None:
                            remaining = deadline - time.monotonic()
                            if remaining <= 0.0:
                                break
                            time.sleep(min(0.02, remaining))
                    self._signal_group_locked(signal.SIGKILL)
                except ChildProcessError:
                    return False
                finally:
                    # No caller may signal the numeric PGID once wait() can
                    # release its leader, including while wait() is in progress.
                    self._signals_closed = True
            try:
                self._returncode = self._process.wait(timeout=kill_timeout)
            except subprocess.TimeoutExpired:
                return False
            self._reaped = True
            return True

    def _poll_exit_locked(self) -> int | None:
        if self._ownership_lost:
            raise ChildProcessError('Process-group leader is no longer owned')
        if self._returncode is not None:
            return self._returncode
        try:
            status = os.waitid(
                os.P_PID, self.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT,
            )
        except OSError:
            # An external reap or failed observation cannot authorize a signal
            # to an unverified numeric process-group ID.
            self._ownership_lost = True
            self._signals_closed = True
            raise
        if status is not None:
            self._returncode = (
                status.si_status if status.si_code == os.CLD_EXITED else -status.si_status
            )
        return self._returncode

    def _signal_group_locked(self, signum: int) -> None:
        if self._signals_closed:
            return
        try:
            os.killpg(self.pid, signum)
        except ProcessLookupError:
            pass


def start_process_group(command: Sequence[str], **popen_kwargs: Any) -> OwnedProcessGroup:
    """Start a command with exclusive ownership of its POSIX group and reap."""
    return OwnedProcessGroup(subprocess.Popen(
        list(command),
        start_new_session=True,
        **popen_kwargs,
    ))


def terminate_process_group(
    process: OwnedProcessGroup,
    *,
    terminate_timeout_sec: float = DEFAULT_TERMINATE_TIMEOUT_SEC,
    kill_timeout_sec: float = DEFAULT_KILL_TIMEOUT_SEC,
) -> bool:
    """
    Terminate an owned group and reap its leader with bounded monotonic waits.

    ``True`` confirms leader reap after the final group signal. ``False`` means
    reap timed out or exclusive ownership was lost. Escaped descendants which
    create another process group are outside this owner's contract.
    """
    return process.terminate(
        terminate_timeout_sec=terminate_timeout_sec,
        kill_timeout_sec=kill_timeout_sec,
    )
