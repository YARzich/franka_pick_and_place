"""
Bounded command-stage execution and per-attempt evidence collection.

Responsibility:
- Expand argv, apply per-command retry budgets and audit every attempt.
- Start configured background commands and verify their liveness.

Does not own:
- Profile state transitions, fault ordering, cleanup or result persistence.
"""

from __future__ import annotations

from collections.abc import Sequence
import threading
import time
from typing import Callable, Mapping, Protocol

from franka_motion_runtime.validation.runtime_budget import command_deadline
from franka_motion_runtime.validation.runtime_config import CommandSpec
from franka_motion_runtime.validation.runtime_evidence import StageResult
from franka_motion_runtime.validation.runtime_process import (
    CleanupReport,
    CommandResult,
    Deadline,
    DeadlineExpired,
    ProcessError,
)


class Supervisor(Protocol):
    """Process operations required by the stage executor and harness."""

    def start_background(
        self,
        command: CommandSpec,
        argv: tuple[str, ...],
    ) -> None:
        """Start one background process."""

    def run_command(
        self,
        command: CommandSpec,
        argv: tuple[str, ...],
        *,
        deadline: Deadline,
        attempt: int,
    ) -> CommandResult:
        """Execute one bounded foreground process."""

    def assert_background_alive(self) -> None:
        """Fail when a required background process exited."""

    def cleanup(self) -> CleanupReport:
        """Stop and join all owned process groups."""


class StageExecutor:
    """Run bounded commands and retain an append-only attempt audit."""

    def __init__(
        self,
        *,
        environment: Mapping[str, str],
        supervisor: Supervisor,
        clock: Callable[[], float] = time.monotonic,
        waiter: Callable[[float], object] | None = None,
    ) -> None:
        """Inject process operations, environment and steady-time services."""
        self._environment = dict(environment)
        self._supervisor = supervisor
        self._clock = clock
        self._waiter = waiter or threading.Event().wait
        self._results: list[StageResult] = []
        self._attempts: dict[str, int] = {}

    @property
    def results(self) -> tuple[StageResult, ...]:
        """Return the immutable attempt audit accumulated so far."""
        return tuple(self._results)

    def start_background(self, commands: Sequence[CommandSpec]) -> None:
        """Start configured services without treating startup as readiness."""
        for command in commands:
            argv = command.expanded_argv(self._environment)
            self._supervisor.start_background(command, argv)
            self._supervisor.assert_background_alive()

    def run_commands(
        self,
        stage: str,
        commands: Sequence[CommandSpec],
        deadline: Deadline,
    ) -> None:
        """Run repetitions sequentially under per-command retry budgets."""
        for command in commands:
            for repetition in range(1, command.repetitions + 1):
                repetition_deadline = command_deadline(
                    command,
                    deadline,
                    self._clock,
                )
                self._run_with_retry(
                    stage,
                    command,
                    repetition,
                    repetition_deadline,
                )
                self._supervisor.assert_background_alive()

    def _run_with_retry(
        self,
        stage: str,
        command: CommandSpec,
        repetition: int,
        deadline: Deadline,
    ) -> None:
        """Retry only commands that explicitly opt into bounded polling."""
        while True:
            attempt = self._attempts.get(command.name, 0) + 1
            self._attempts[command.name] = attempt
            argv = command.expanded_argv(self._environment)
            try:
                result = self._supervisor.run_command(
                    command,
                    argv,
                    deadline=deadline,
                    attempt=attempt,
                )
            except ProcessError as error:
                self._record_failure(
                    stage, command, repetition, attempt, error
                )
                if not command.retry_until_pass:
                    raise
                self._wait_before_retry(command, deadline)
                continue
            self._record_success(
                stage, command, repetition, attempt, result
            )
            return

    def _wait_before_retry(
        self,
        command: CommandSpec,
        deadline: Deadline,
    ) -> None:
        """Wait only for the bounded retry interval remaining."""
        wait_duration = min(command.retry_interval_sec, deadline.remaining())
        if wait_duration <= 0.0:
            raise DeadlineExpired(
                f"{command.name} did not pass before command deadline"
            )
        self._waiter(wait_duration)

    def _record_success(
        self,
        stage: str,
        command: CommandSpec,
        repetition: int,
        attempt: int,
        result: CommandResult,
    ) -> None:
        """Append one successful command result to the evidence audit."""
        self._results.append(
            StageResult(
                stage=stage,
                command=command.name,
                repetition=repetition,
                attempt=attempt,
                outcome="passed",
                return_code=result.return_code,
                elapsed_sec=result.elapsed_sec,
                log_path=str(result.log_path),
                error=None,
            )
        )

    def _record_failure(
        self,
        stage: str,
        command: CommandSpec,
        repetition: int,
        attempt: int,
        error: ProcessError,
    ) -> None:
        """Append a failed attempt without requiring a completed result."""
        result = getattr(error, "result", None)
        self._results.append(
            StageResult(
                stage=stage,
                command=command.name,
                repetition=repetition,
                attempt=attempt,
                outcome="failed",
                return_code=None if result is None else result.return_code,
                elapsed_sec=0.0 if result is None else result.elapsed_sec,
                log_path="" if result is None else str(result.log_path),
                error=f"{type(error).__name__}: {error}",
            )
        )
