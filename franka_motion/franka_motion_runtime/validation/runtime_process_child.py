"""
Creation and bounded joining of direct runtime subprocesses.

Responsibility:
- Start one argv-only child session with an owned log stream.
- Join a direct child through bounded TERM/KILL escalation.

Does not own:
- Process registries, runtime evidence, retries or escaped descendants.
"""

from __future__ import annotations

import os
from pathlib import Path
import re
import signal
import subprocess
import time
from typing import Callable, Mapping

from franka_motion_runtime.validation.runtime_child import ManagedProcess
from franka_motion_runtime.validation.runtime_deadline import (
    Deadline,
    ProcessIsolationError,
)


class DirectChildRuntime:
    """Create and join direct children without interpreting command results."""

    def __init__(
        self,
        *,
        artifact_directory: Path,
        environment: Mapping[str, str],
        popen_factory: Callable[..., subprocess.Popen[bytes]],
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """Store explicit process resources and create the log directory."""
        self._artifact_directory = artifact_directory
        self._environment = dict(environment)
        self._popen_factory = popen_factory
        self._clock = clock
        self._artifact_directory.mkdir(parents=True, exist_ok=True)

    def start(
        self,
        name: str,
        argv: tuple[str, ...],
        *,
        attempt: int,
    ) -> ManagedProcess:
        """Start one command in a new session with a unique attempt log."""
        safe_name = re.sub(r"[^A-Za-z0-9_.-]", "_", name)
        log_path = self._artifact_directory / (
            f"{safe_name}.attempt-{attempt}.log"
        )
        log_stream = log_path.open("ab")
        try:
            process = self._popen_factory(
                argv,
                env=self._environment,
                stdout=log_stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except Exception:
            log_stream.close()
            raise
        return ManagedProcess(
            name=name,
            argv=argv,
            process=process,
            log_stream=log_stream,
            log_path=log_path,
            started_at=self._clock(),
            clock=self._clock,
        )

    def terminate(
        self,
        process: ManagedProcess,
        deadline: Deadline,
    ) -> None:
        """Join one child after bounded TERM and KILL escalation."""
        if process.process.poll() is not None:
            process.process.wait()
            return
        try:
            os.killpg(process.process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            process.process.wait(timeout=deadline.clip(10.0))
            return
        except subprocess.TimeoutExpired:
            pass
        try:
            os.killpg(process.process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            process.process.wait(timeout=deadline.clip(5.0))
        except subprocess.TimeoutExpired as error:
            raise ProcessIsolationError(
                f"cannot join process group for {process.name}"
            ) from error

    def terminate_with_cleanup_budget(self, process: ManagedProcess) -> None:
        """Join a failed foreground child independently from profile time."""
        cleanup_deadline = Deadline.after(15.0, clock=self._clock)
        self.terminate(process, cleanup_deadline)
