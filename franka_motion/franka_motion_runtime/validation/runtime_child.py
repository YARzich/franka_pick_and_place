"""
Direct-child handle owned by the runtime process supervisor.

Responsibility:
- Bind one Popen child to its argv, log stream and Linux identity.
- Close output exactly once and produce an immutable command result.

Does not own:
- Child creation, signals, deadlines, registry persistence or retries.
"""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
from typing import BinaryIO, Callable

from franka_motion_runtime.validation.runtime_deadline import CommandResult
from franka_motion_runtime.validation.runtime_registry import (
    ProcessIdentity,
    read_start_ticks,
)


class ManagedProcess:
    """Own one direct child, its log stream and join operation."""

    def __init__(
        self,
        *,
        name: str,
        argv: tuple[str, ...],
        process: subprocess.Popen[bytes],
        log_stream: BinaryIO,
        log_path: Path,
        started_at: float,
        clock: Callable[[], float],
    ) -> None:
        """Bind one already-started child to its owned resources."""
        self.name = name
        self.argv = argv
        self.process = process
        self.log_path = log_path
        self.started_at = started_at
        self._log_stream = log_stream
        self._clock = clock

    @property
    def identity(self) -> ProcessIdentity:
        """Return the current direct-child identity."""
        return ProcessIdentity(
            pid=self.process.pid,
            process_group_id=os.getpgid(self.process.pid),
            start_ticks=read_start_ticks(
                Path(f"/proc/{self.process.pid}/stat")
            ),
        )

    def result(self, return_code: int) -> CommandResult:
        """Close output and create the immutable command result."""
        self.close_log()
        return CommandResult(
            name=self.name,
            argv=self.argv,
            return_code=return_code,
            elapsed_sec=max(0.0, self._clock() - self.started_at),
            log_path=self.log_path,
        )

    def close_log(self) -> None:
        """Flush and close output exactly once."""
        if not self._log_stream.closed:
            self._log_stream.flush()
            self._log_stream.close()
