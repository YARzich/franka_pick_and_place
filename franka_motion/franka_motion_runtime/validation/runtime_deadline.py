"""
Monotonic deadline and result values shared by runtime infrastructure.

Responsibility:
- Clip every stage to one absolute steady-clock deadline.
- Carry command and cleanup outcomes without interpreting evidence.

Does not own:
- Processes, retries, registry persistence or workflow semantics.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time
from typing import Callable


class ProcessError(RuntimeError):
    """Base error for process ownership or execution failures."""


class DeadlineExpired(ProcessError):
    """Report exhaustion of an absolute monotonic deadline."""


class ProcessIsolationError(ProcessError):
    """Report a direct child that could not be joined before continuing."""


@dataclass(frozen=True)
class Deadline:
    """Absolute monotonic deadline shared by every stage of one profile."""

    expires_at: float
    clock: Callable[[], float] = time.monotonic

    @classmethod
    def after(
        cls,
        timeout_sec: float,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> "Deadline":
        """Create a deadline from one positive duration."""
        if timeout_sec <= 0.0:
            raise ValueError("deadline timeout must be positive")
        return cls(expires_at=clock() + timeout_sec, clock=clock)

    def remaining(self) -> float:
        """Return non-negative time remaining on the steady clock."""
        return max(0.0, self.expires_at - self.clock())

    def clip(self, requested_sec: float) -> float:
        """Bound a stage timeout by the profile deadline."""
        remaining = self.remaining()
        if remaining <= 0.0:
            raise DeadlineExpired("runtime profile deadline expired")
        return min(requested_sec, remaining)


@dataclass(frozen=True)
class CommandResult:
    """Auditable outcome of one foreground command attempt."""

    name: str
    argv: tuple[str, ...]
    return_code: int
    elapsed_sec: float
    log_path: Path


class CommandAttemptError(ProcessError):
    """Report a completed command attempt that violated its output contract."""

    def __init__(self, message: str, result: CommandResult) -> None:
        """Preserve the failed attempt so evidence can audit it."""
        super().__init__(message)
        self.result = result


class CommandTimeoutError(DeadlineExpired):
    """Report a joined timed-out command together with its final result."""

    def __init__(self, message: str, result: CommandResult) -> None:
        """Preserve timeout evidence after bounded process termination."""
        super().__init__(message)
        self.result = result


@dataclass(frozen=True)
class CleanupReport:
    """Signals sent and independent errors produced by one cleanup pass."""

    run_token: str
    terminated_groups: tuple[int, ...]
    killed_groups: tuple[int, ...]
    errors: tuple[str, ...]

    @property
    def clean(self) -> bool:
        """Return whether all owned processes disappeared without cleanup errors."""
        return not self.errors
