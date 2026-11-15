"""
Concrete runtime service wiring for normal and cleanup-only invocations.

Responsibility:
- Combine validated profiles with explicit supervisor and registry instances.
- Clean token-scoped process groups for interrupted simulation runs.

Does not own:
- CLI parsing, signal handling, stage semantics or process implementation.
"""

from __future__ import annotations

from pathlib import Path
from typing import Mapping

from franka_motion_runtime.validation.runtime_config import load_runtime_matrix
from franka_motion_runtime.validation.runtime_evidence import (
    HarnessOutcome,
    RESULT_SCHEMA_VERSION,
    write_json,
)
from franka_motion_runtime.validation.runtime_orchestrator import RuntimeHarness
from franka_motion_runtime.validation.runtime_process import (
    CleanupReport,
    OwnedGroupCleaner,
    ProcessRegistry,
    ProcessSupervisor,
    cleanup_registered_run,
)
from franka_motion_runtime.validation.runtime_process_role import (
    CONTROL_DIRECTORY_VARIABLE,
)


ARTIFACT_DIRECTORY_VARIABLE = "FRANKA_RUNTIME_ARTIFACT_DIR"


def run_profile(
    *,
    matrix_path: Path,
    profile_id: str,
    artifact_directory: Path,
    state_path: Path,
    run_token: str,
    environment: Mapping[str, str],
) -> HarnessOutcome:
    """Create concrete process infrastructure and run one matrix profile."""
    matrix = load_runtime_matrix(matrix_path)
    profile = matrix.profile(profile_id)
    run_environment = _profile_environment(
        profile.environment,
        environment,
        run_token,
        artifact_directory,
    )
    supervisor = ProcessSupervisor(
        run_token=run_token,
        state_path=state_path,
        artifact_directory=artifact_directory / "commands",
        environment=run_environment,
    )
    harness = RuntimeHarness(
        matrix_id=matrix.matrix_id,
        profile=profile,
        run_token=run_token,
        artifact_directory=artifact_directory,
        environment=run_environment,
        supervisor=supervisor,
    )
    return harness.run()


def cleanup_profile(
    *,
    matrix_path: Path,
    profile_id: str,
    artifact_directory: Path,
    state_path: Path,
    requested_run_token: str | None,
    environment: Mapping[str, str],
) -> HarnessOutcome:
    """Clean an interrupted run using registered token ownership when present."""
    matrix = load_runtime_matrix(matrix_path)
    profile = matrix.profile(profile_id)
    registered_token = ProcessRegistry(state_path).read_token()
    run_token = registered_token or requested_run_token
    artifact_directory.mkdir(parents=True, exist_ok=True)
    if run_token is None:
        return _nothing_registered(profile.profile_id, artifact_directory)

    report = _cleanup_simulator_run(
        registered_token,
        run_token,
        state_path,
    )
    cleanup_errors = []
    cleanup_errors.extend(report.errors)
    return _write_cleanup_result(
        profile.profile_id,
        run_token,
        artifact_directory,
        report,
        cleanup_errors,
    )


def _profile_environment(
    profile_environment: Mapping[str, str],
    environment: Mapping[str, str],
    run_token: str,
    artifact_directory: Path,
) -> dict[str, str]:
    """Create an explicit child environment without mutating process globals."""
    result = dict(environment)
    result.update(profile_environment)
    result["FRANKA_RUNTIME_RUN_TOKEN"] = run_token
    result[ARTIFACT_DIRECTORY_VARIABLE] = str(artifact_directory.resolve())
    process_control_directory = artifact_directory / "process-control"
    process_control_directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    result[CONTROL_DIRECTORY_VARIABLE] = str(process_control_directory)
    ros_log_directory = artifact_directory / "ros"
    ros_log_directory.mkdir(parents=True, exist_ok=True)
    result["ROS_LOG_DIR"] = str(ros_log_directory)
    return result


def _cleanup_simulator_run(
    registered_token: str | None,
    run_token: str,
    state_path: Path,
) -> CleanupReport:
    """Clean registered or explicitly requested simulator ownership."""
    cleaner = OwnedGroupCleaner()
    if registered_token is None:
        return cleaner.cleanup(run_token)
    report = cleanup_registered_run(state_path, cleaner=cleaner)
    return report or CleanupReport(run_token, (), (), ())


def _nothing_registered(
    profile_id: str,
    artifact_directory: Path,
) -> HarnessOutcome:
    """Record an idempotent no-op cleanup without claiming evidence."""
    result_path = artifact_directory / "cleanup-result.v2.json"
    write_json(
        result_path,
        {
            "schema_version": RESULT_SCHEMA_VERSION,
            "profile_id": profile_id,
            "evidence_status": "not_produced",
            "cleanup_status": "nothing_registered",
            "cleanup_errors": [],
        },
    )
    return HarnessOutcome(0, result_path, "not_produced", None, ())


def _write_cleanup_result(
    profile_id: str,
    run_token: str,
    artifact_directory: Path,
    report: CleanupReport,
    cleanup_errors: list[str],
) -> HarnessOutcome:
    """Persist cleanup evidence and map independent errors to exit status."""
    result_path = artifact_directory / "cleanup-result.v2.json"
    write_json(
        result_path,
        {
            "schema_version": RESULT_SCHEMA_VERSION,
            "profile_id": profile_id,
            "run_token": run_token,
            "evidence_status": "not_produced",
            "cleanup_status": "passed" if not cleanup_errors else "failed",
            "cleanup_errors": cleanup_errors,
            "terminated_groups": report.terminated_groups,
            "killed_groups": report.killed_groups,
        },
    )
    exit_code = 0 if not cleanup_errors else 2
    return HarnessOutcome(
        exit_code,
        result_path,
        "not_produced",
        None,
        tuple(cleanup_errors),
    )
