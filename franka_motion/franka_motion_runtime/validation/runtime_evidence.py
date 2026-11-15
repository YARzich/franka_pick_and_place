"""
Structured evidence values and deterministic runtime result persistence.

Responsibility:
- Keep prerequisite, runtime and cleanup outcomes independently visible.
- Serialize auditable stages and an explicit provenance allowlist.

Does not own:
- Stage execution, process cleanup, configuration or CLI exit handling.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Mapping, Sequence

from franka_motion_runtime.validation.runtime_config import RuntimeProfile

RESULT_SCHEMA_VERSION = 2

_PROVENANCE_ENVIRONMENT = {
    "github_sha": "GITHUB_SHA",
    "github_run_id": "GITHUB_RUN_ID",
    "github_run_attempt": "GITHUB_RUN_ATTEMPT",
    "runner_name": "RUNNER_NAME",
    "ros_distro": "ROS_DISTRO",
    "rmw_implementation": "RMW_IMPLEMENTATION",
}


@dataclass(frozen=True)
class StageResult:
    """Serializable result of one command attempt, successful or failed."""

    stage: str
    command: str
    repetition: int
    attempt: int
    outcome: str
    return_code: int | None
    elapsed_sec: float
    log_path: str
    error: str | None


@dataclass(frozen=True)
class HarnessOutcome:
    """Final status with runtime evidence and cleanup kept independent."""

    exit_code: int
    result_path: Path
    evidence_status: str
    primary_error: str | None
    cleanup_errors: tuple[str, ...]


def write_runtime_result(
    *,
    matrix_id: str,
    profile: RuntimeProfile,
    run_token: str,
    artifact_directory: Path,
    started_at: str,
    evidence_status: str,
    primary_error: str | None,
    cleanup_errors: Sequence[str],
    progress: Mapping[str, bool],
    stages: Sequence[StageResult],
    environment: Mapping[str, str],
) -> Path:
    """Write a result with provenance allowlisted from process environment."""
    result = {
        "schema_version": RESULT_SCHEMA_VERSION,
        "matrix_id": matrix_id,
        "profile_id": profile.profile_id,
        "platform": profile.platform.value,
        "evidence_kind": profile.evidence_kind.value,
        "evidence_status": evidence_status,
        "run_token": run_token,
        "provenance": {
            field: environment.get(environment_name)
            for field, environment_name in _PROVENANCE_ENVIRONMENT.items()
        },
        "started_at": started_at,
        "finished_at": utc_now(),
        **progress,
        "primary_error": primary_error,
        "cleanup_errors": list(cleanup_errors),
        "stages": [asdict(stage) for stage in stages],
    }
    result_path = artifact_directory / "runtime-result.v2.json"
    write_json(result_path, result)
    return result_path


def evidence_exit_code(
    primary_error: str | None,
    cleanup_errors: Sequence[str],
    runtime_validation_completed: bool,
) -> int:
    """Map independent evidence dimensions to a stable process status."""
    if primary_error is not None:
        return 1
    if cleanup_errors:
        return 2
    if not runtime_validation_completed:
        return 3
    return 0


def write_json(path: Path, value: object) -> None:
    """Write deterministic metadata without process environment."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def utc_now() -> str:
    """Return an ISO 8601 UTC timestamp for human correlation only."""
    return datetime.now(timezone.utc).isoformat()
