"""
Evidence-aware stage orchestration independent from concrete subprocesses.

Responsibility:
- Execute preflight, readiness and validation stages.
- Delegate fault transactions and evidence serialization to focused modules.

Does not own:
- JSON configuration, process creation, registry persistence or CLI signals.
"""

from __future__ import annotations

from pathlib import Path
import time
from typing import Callable, Mapping

from franka_motion_runtime.validation.runtime_config import (
    RuntimeProfile,
)
from franka_motion_runtime.validation.runtime_evidence import (
    HarnessOutcome,
    evidence_exit_code,
    utc_now,
    write_runtime_result,
)
from franka_motion_runtime.validation.runtime_process import (
    Deadline,
    ProcessError,
)
from franka_motion_runtime.validation.runtime_stage import StageExecutor, Supervisor
from franka_motion_runtime.validation.runtime_validation import run_runtime_validation


class RuntimeHarness:
    """Run one validated profile using an injected process supervisor."""

    def __init__(
        self,
        *,
        matrix_id: str,
        profile: RuntimeProfile,
        run_token: str,
        artifact_directory: Path,
        environment: Mapping[str, str],
        supervisor: Supervisor,
        clock: Callable[[], float] = time.monotonic,
        waiter: Callable[[float], object] | None = None,
    ) -> None:
        """Inject immutable profile state and explicit process ownership."""
        self._matrix_id = matrix_id
        self._profile = profile
        self._run_token = run_token
        self._artifact_directory = artifact_directory
        self._environment = dict(environment)
        self._environment["FRANKA_RUNTIME_RUN_TOKEN"] = run_token
        self._supervisor = supervisor
        self._clock = clock
        self._stages = StageExecutor(
            environment=self._environment,
            supervisor=supervisor,
            clock=clock,
            waiter=waiter,
        )

    def run(self) -> HarnessOutcome:
        """Execute the profile and always clean its owned processes."""
        started_at = utc_now()
        deadline = Deadline.after(
            self._profile.deadline_sec,
            clock=self._clock,
        )
        progress = {
            "preflight_completed": False,
            "readiness_completed": False,
            "runtime_validation_completed": False,
        }
        evidence_status = "not_produced"
        primary_error = None
        cleanup_errors = []

        try:
            self._stages.run_commands(
                "preflight", self._profile.preflight, deadline
            )
            progress["preflight_completed"] = True
            self._stages.start_background(self._profile.background)
            self._stages.run_commands(
                "readiness", self._profile.readiness, deadline
            )
            progress["readiness_completed"] = True
            self._run_runtime_validation(deadline)
            progress["runtime_validation_completed"] = True
            evidence_status = "passed"
        except (ProcessError, OSError, ValueError) as error:
            primary_error = f"{type(error).__name__}: {error}"
        finally:
            cleanup_errors.extend(self._cleanup_processes())

        if cleanup_errors:
            evidence_status = "invalidated_by_cleanup_failure"
        exit_code = evidence_exit_code(
            primary_error,
            cleanup_errors,
            progress["runtime_validation_completed"],
        )
        result_path = write_runtime_result(
            matrix_id=self._matrix_id,
            profile=self._profile,
            run_token=self._run_token,
            artifact_directory=self._artifact_directory,
            started_at=started_at,
            evidence_status=evidence_status,
            primary_error=primary_error,
            cleanup_errors=cleanup_errors,
            progress=progress,
            stages=self._stages.results,
            environment=self._environment,
        )
        return HarnessOutcome(
            exit_code=exit_code,
            result_path=result_path,
            evidence_status=evidence_status,
            primary_error=primary_error,
            cleanup_errors=tuple(cleanup_errors),
        )

    def _run_runtime_validation(self, deadline: Deadline) -> None:
        """Run ordered faults and validate only after confirmed recoveries."""
        run_runtime_validation(
            self._profile.faults,
            self._profile.validation,
            deadline,
            self._stages.run_commands,
        )

    def _cleanup_processes(self) -> tuple[str, ...]:
        """Convert cleanup exceptions into independent result fields."""
        try:
            return self._supervisor.cleanup().errors
        except (ProcessError, OSError, ValueError) as error:
            return (
                f"process cleanup failed: {type(error).__name__}: {error}",
            )
