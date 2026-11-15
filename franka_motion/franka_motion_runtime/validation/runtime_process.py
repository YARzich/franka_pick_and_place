"""
Joinable subprocess supervisor for one explicit runtime run token.

Responsibility:
- Start argv-only commands in new sessions and join every direct child.
- Validate exit/output contracts and delegate escaped descendants to cleanup.

Does not own:
- Configuration parsing, retries, evidence semantics or stage ordering.
"""

from __future__ import annotations

from pathlib import Path
import subprocess
import time
from typing import Callable, Mapping

from franka_motion_runtime.validation.runtime_child import ManagedProcess
from franka_motion_runtime.validation.runtime_cleanup import (
    OwnedGroupCleaner,
    cleanup_registered_run,
)
from franka_motion_runtime.validation.runtime_config import CommandSpec
from franka_motion_runtime.validation.runtime_deadline import (
    CleanupReport,
    CommandAttemptError,
    CommandResult,
    CommandTimeoutError,
    Deadline,
    DeadlineExpired,
    ProcessError,
    ProcessIsolationError,
)
from franka_motion_runtime.validation.runtime_output import validate_command_result
from franka_motion_runtime.validation.runtime_process_child import DirectChildRuntime
from franka_motion_runtime.validation.runtime_registry import (
    ProcessIdentity,
    ProcessRegistry,
    RUN_TOKEN_VARIABLE,
)

__all__ = [
    "CleanupReport",
    "CommandAttemptError",
    "CommandResult",
    "CommandTimeoutError",
    "Deadline",
    "DeadlineExpired",
    "OwnedGroupCleaner",
    "ProcessError",
    "ProcessIsolationError",
    "ProcessIdentity",
    "ProcessRegistry",
    "ProcessSupervisor",
    "cleanup_registered_run",
]


class ProcessSupervisor:
    """Start, join and clean commands associated with one run token."""

    def __init__(
        self,
        *,
        run_token: str,
        state_path: Path,
        artifact_directory: Path,
        environment: Mapping[str, str],
        cleaner: OwnedGroupCleaner | None = None,
        adopt_registered_token: bool = False,
        popen_factory: Callable[..., subprocess.Popen[bytes]] = subprocess.Popen,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """Create isolated ownership state for one explicit run token."""
        if not run_token:
            raise ValueError("run_token must not be empty")
        self.run_token = run_token
        self._registry = ProcessRegistry(state_path)
        self._environment = dict(environment)
        self._environment[RUN_TOKEN_VARIABLE] = run_token
        self._cleaner = cleaner or OwnedGroupCleaner()
        self._clock = clock
        self._processes: list[ManagedProcess] = []
        self._children = DirectChildRuntime(
            artifact_directory=artifact_directory,
            environment=self._environment,
            popen_factory=popen_factory,
            clock=clock,
        )
        self._registry.claim(
            run_token,
            adopt_existing=adopt_registered_token,
        )

    def start_background(
        self,
        command: CommandSpec,
        argv: tuple[str, ...],
    ) -> None:
        """Start one long-lived command and retain its joinable child handle."""
        process = self._children.start(command.name, argv, attempt=1)
        self._processes.append(process)
        self._write_registry()

    def run_command(
        self,
        command: CommandSpec,
        argv: tuple[str, ...],
        *,
        deadline: Deadline,
        attempt: int,
    ) -> CommandResult:
        """Run one foreground command under command and profile deadlines."""
        # Fail before spawn: an expired profile must never create a child that
        # can overlap a fault recovery command.
        timeout_sec = deadline.clip(command.timeout_sec)
        process = self._children.start(command.name, argv, attempt=attempt)
        self._processes.append(process)
        try:
            self._write_registry()
            try:
                return_code = process.process.wait(timeout=timeout_sec)
            except subprocess.TimeoutExpired as error:
                self._children.terminate_with_cleanup_budget(process)
                return_code = process.process.returncode
                if return_code is None:
                    raise ProcessIsolationError(
                        f"joined command {command.name} has no return code"
                    )
                result = process.result(return_code)
                raise CommandTimeoutError(
                    f"{command.name} exceeded its {timeout_sec:.3f}s deadline",
                    result,
                ) from error

            result = process.result(return_code)
            try:
                validate_command_result(command, result)
            except ProcessError as error:
                raise CommandAttemptError(str(error), result) from error
            return result
        except (CommandAttemptError, CommandTimeoutError):
            raise
        except (OSError, ProcessError, ValueError) as error:
            if process.process.poll() is None:
                try:
                    self._children.terminate_with_cleanup_budget(process)
                except ProcessIsolationError as cleanup_error:
                    raise ProcessIsolationError(
                        f"{error}; foreground cleanup also failed: "
                        f"{cleanup_error}"
                    ) from error
            raise
        finally:
            if process.process.poll() is not None:
                self._remove_process(process)

    def assert_background_alive(self) -> None:
        """Fail if a required background process exited before cleanup."""
        exited = [
            f"{process.name}={process.process.returncode}"
            for process in self._processes
            if process.process.poll() is not None
        ]
        if exited:
            raise ProcessError(
                "background process exited unexpectedly: " + ", ".join(exited)
            )

    def cleanup(self) -> CleanupReport:
        """Terminate direct children, join them and catch escaped descendants."""
        direct_errors = []
        direct_deadline = Deadline.after(15.0, clock=self._clock)
        for process in reversed(self._processes):
            if process.process.poll() is not None:
                direct_errors.append(
                    f"{process.name} exited before cleanup with "
                    f"{process.process.returncode}"
                )
            try:
                self._children.terminate(process, direct_deadline)
            except (ProcessError, OSError) as error:
                direct_errors.append(str(error))
            finally:
                process.close_log()
        self._processes.clear()

        report = self._cleaner.cleanup(self.run_token)
        combined_report = CleanupReport(
            run_token=self.run_token,
            terminated_groups=report.terminated_groups,
            killed_groups=report.killed_groups,
            errors=tuple(direct_errors) + report.errors,
        )
        return self._clear_registry_after_clean(combined_report)

    def _remove_process(self, process: ManagedProcess) -> None:
        """Close and unregister one joined direct child."""
        if process in self._processes:
            self._processes.remove(process)
        process.close_log()
        self._write_registry()

    def _write_registry(self) -> None:
        """Persist identities for currently live direct-child roots."""
        roots = []
        for process in self._processes:
            if process.process.poll() is not None:
                continue
            try:
                roots.append(process.identity)
            except (FileNotFoundError, ProcessLookupError):
                continue
        self._registry.write(self.run_token, tuple(roots))

    def _clear_registry_after_clean(
        self,
        report: CleanupReport,
    ) -> CleanupReport:
        """Remove ownership state only after independently verified cleanup."""
        if not report.clean:
            return report
        try:
            self._registry.clear_if_token(self.run_token)
        except ProcessError as error:
            return CleanupReport(
                run_token=self.run_token,
                terminated_groups=report.terminated_groups,
                killed_groups=report.killed_groups,
                errors=(str(error),),
            )
        return report
