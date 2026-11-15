"""
Shared process-group runner for ROS launch helper wrappers.

Responsibility:
- Start one child command in a separate process session.
- Convert wrapper SIGINT/SIGTERM into a configured child-group signal.
- Escalate to SIGKILL after a bounded timeout.
- Optionally clean up helper descendants after the direct child exits.

Does not own:
- Command-line parsing.
- ROS/Gazebo/Isaac-specific command construction.
- Runtime topic or controller semantics.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
import os
import signal
import subprocess
import sys
import time


@dataclass(frozen=True)
class ProcessGroupRunnerConfig:
    """Immutable process ownership and bounded-shutdown policy."""

    command: Sequence[str]
    child_name: str
    shutdown_signal: signal.Signals
    shutdown_timeout_sec: float
    kill_timeout_sec: float
    cleanup_on_child_exit: bool = True
    poll_interval_sec: float = 0.05


class ProcessGroupRunner:
    """Runs one command in its own process group and owns bounded shutdown."""

    def __init__(self, config: ProcessGroupRunnerConfig) -> None:
        """Store the policy and initialize exclusive child ownership state."""
        self._config = config
        self._child: subprocess.Popen[bytes] | None = None
        self._child_pgid: int | None = None
        self._signaling_closed = False
        self._shutdown_requested = False
        self._shutdown_started_at: float | None = None
        self._shutdown_sent = False
        self._sigkill_sent_at: float | None = None

    def run(self) -> int:
        """Run the child until normal exit or a bounded shutdown request."""
        signal.signal(signal.SIGINT, self._request_shutdown)
        signal.signal(signal.SIGTERM, self._request_shutdown)
        self._start_child()

        try:
            while True:
                child_return_code = self._child_return_code()
                if self._shutdown_requested:
                    if self._advance_shutdown():
                        return 0
                elif child_return_code is not None:
                    if not self._config.cleanup_on_child_exit:
                        return child_return_code
                    if self._advance_shutdown():
                        return child_return_code

                time.sleep(self._config.poll_interval_sec)
        finally:
            # A /proc snapshot may miss a descendant born during its scan.
            # Finish cleanup on the pinned group before retiring its identity.
            if self._shutdown_requested or self._config.cleanup_on_child_exit:
                self._send_sigkill()
            self._close_signaling()
            if self._child is not None:
                try:
                    self._child.wait(timeout=0)
                except subprocess.TimeoutExpired:
                    pass  # The bounded shutdown deadline has already expired.

    def _start_child(self) -> None:
        self._child = subprocess.Popen(
            self._config.command,
            start_new_session=True,
        )
        try:
            self._child_pgid = os.getpgid(self._child.pid)
        except ProcessLookupError:
            self._child_pgid = self._child.pid

    def _request_shutdown(self, _signum: int, _frame) -> None:
        self._shutdown_requested = True
        if self._shutdown_started_at is None:
            self._shutdown_started_at = time.monotonic()

    def _child_return_code(self) -> int | None:
        if self._child is None:
            return None
        if self._child.returncode is not None:
            self._close_signaling()
            return self._child.returncode
        try:
            status = os.waitid(
                os.P_PID, self._child.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT,
            )
        except ChildProcessError:
            # Someone else reaped the child: ownership can no longer be proven.
            self._close_signaling()
            raise RuntimeError("Child was reaped outside its process-group owner") from None
        if status is None:
            return None
        if status.si_code == os.CLD_EXITED:
            return status.si_status
        return -status.si_status

    def _close_signaling(self) -> None:
        """Permanently retire the owned group before reaping its leader."""
        self._signaling_closed = True
        self._child_pgid = None

    def _advance_shutdown(self) -> bool:
        if self._shutdown_started_at is None:
            self._shutdown_started_at = time.monotonic()
        self._send_shutdown_signal()

        if not self._process_group_exists():
            return True

        now = time.monotonic()
        if self._sigkill_sent_at is None:
            elapsed = now - self._shutdown_started_at
            if elapsed >= self._config.shutdown_timeout_sec:
                self._send_sigkill()
            return not self._process_group_exists()

        if not self._process_group_exists():
            return True

        kill_elapsed = now - self._sigkill_sent_at
        if kill_elapsed >= self._config.kill_timeout_sec:
            print(
                f"{self._config.child_name}: process group did not exit after SIGKILL; "
                "wrapper exits anyway.",
                file=sys.stderr,
                flush=True,
            )
            return True
        return False

    def _send_shutdown_signal(self) -> None:
        if self._shutdown_sent:
            return
        if (
            self._send_process_group_signal(self._config.shutdown_signal)
            and self._config.shutdown_signal == signal.SIGKILL
        ):
            self._sigkill_sent_at = time.monotonic()
        self._shutdown_sent = True

    def _send_sigkill(self) -> None:
        if self._sigkill_sent_at is not None:
            return
        if self._send_process_group_signal(signal.SIGKILL):
            self._sigkill_sent_at = time.monotonic()

    def _send_process_group_signal(self, signum: signal.Signals) -> bool:
        pgid = self._child_pgid
        if self._signaling_closed or pgid is None:
            return False
        try:
            os.killpg(pgid, signum)
        except ProcessLookupError:
            return False
        return True

    def _process_group_exists(self) -> bool:
        """Check live members; the intentionally pinned zombie is not work."""
        pgid = self._child_pgid
        if self._signaling_closed or pgid is None:
            return False
        for entry in Path("/proc").iterdir():
            if not entry.name.isdecimal():
                continue
            try:
                # comm may contain spaces and parentheses; fields start after it.
                fields = (entry / "stat").read_text().rsplit(")", 1)[1].split()
                if int(fields[2]) == pgid and fields[0] not in {"Z", "X"}:
                    return True
            except (FileNotFoundError, ProcessLookupError):
                continue
            except (PermissionError, IndexError, ValueError):
                # Fail conservatively while the unreaped leader pins this PGID.
                return True
        return False
