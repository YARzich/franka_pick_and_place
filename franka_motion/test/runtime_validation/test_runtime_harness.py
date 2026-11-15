"""
Test runtime-validation orchestration without subprocesses, ROS graph, or simulator.

Responsibility: Verify preflight, fault, recovery, validation, evidence, and cleanup ordering.
"""

from dataclasses import replace
import json
from pathlib import Path
from types import MappingProxyType

from franka_motion_runtime.validation.runtime_config import (
    CommandSpec,
    EvidenceKind,
    FaultProfile,
    Platform,
    RuntimeProfile,
)
from franka_motion_runtime.validation.runtime_harness import RuntimeHarness
from franka_motion_runtime.validation.runtime_process import (
    CleanupReport,
    CommandResult,
    ProcessError,
)


def _command(
    name: str,
    *,
    repetitions: int = 1,
    timeout_sec: float = 1.0,
    retry_until_pass: bool = False,
) -> CommandSpec:
    return CommandSpec(
        name=name,
        argv=("program", name),
        timeout_sec=timeout_sec,
        repetitions=repetitions,
        retry_until_pass=retry_until_pass,
        retry_interval_sec=0.001,
    )


def _simulator_profile(
    *,
    faults: tuple[FaultProfile, ...] = (),
    soak: bool = False,
) -> RuntimeProfile:
    evidence_kind = (
        EvidenceKind.DIAGNOSTIC_FAULT
        if faults
        else (
            EvidenceKind.SIMULATOR_SOAK
            if soak
            else EvidenceKind.SIMULATOR_MOTION
        )
    )
    validation = (_command("validate", repetitions=2 if soak else 1),)
    return RuntimeProfile(
        profile_id="test_profile",
        platform=Platform.GAZEBO,
        evidence_kind=evidence_kind,
        deadline_sec=30.0,
        requirements=("self-hosted",),
        environment=MappingProxyType({}),
        preflight=(_command("preflight"),),
        background=(_command("background", timeout_sec=30.0),),
        readiness=(_command("readiness"),),
        validation=validation,
        faults=faults,
    )


class _FakeSupervisor:
    def __init__(
        self,
        artifact_directory: Path,
        *,
        failures: set[str] | None = None,
        failure_counts: dict[str, int] | None = None,
        cleanup_errors: tuple[str, ...] = (),
    ) -> None:
        self.artifact_directory = artifact_directory
        self.failures = failures or set()
        self.failure_counts = dict(failure_counts or {})
        self.cleanup_errors = cleanup_errors
        self.started = []
        self.ran = []
        self.cleanup_called = False

    def start_background(
        self,
        command: CommandSpec,
        argv: tuple[str, ...],
    ) -> None:
        self.started.append((command.name, argv))

    def run_command(
        self,
        command: CommandSpec,
        argv: tuple[str, ...],
        *,
        deadline,
        attempt: int,
    ) -> CommandResult:
        del deadline
        self.ran.append(command.name)
        remaining_failures = self.failure_counts.get(command.name, 0)
        if command.name in self.failures or remaining_failures > 0:
            if remaining_failures > 0:
                self.failure_counts[command.name] = remaining_failures - 1
            raise ProcessError(f"injected failure: {command.name}")
        return CommandResult(
            name=command.name,
            argv=argv,
            return_code=0,
            elapsed_sec=0.01,
            log_path=self.artifact_directory / f"{command.name}-{attempt}.log",
        )

    def assert_background_alive(self) -> None:
        return None

    def cleanup(self) -> CleanupReport:
        self.cleanup_called = True
        return CleanupReport(
            run_token="run-token",
            terminated_groups=(),
            killed_groups=(),
            errors=self.cleanup_errors,
        )


def _run(
    tmp_path: Path,
    profile: RuntimeProfile,
    supervisor: _FakeSupervisor,
    *,
    environment: dict[str, str] | None = None,
):
    harness = RuntimeHarness(
        matrix_id="test-matrix",
        profile=profile,
        run_token="run-token",
        artifact_directory=tmp_path,
        environment=environment or {},
        supervisor=supervisor,
    )
    return harness.run()


def test_preflight_failure_never_becomes_motion_evidence(tmp_path: Path) -> None:
    """Preflight success or failure is never a substitute for motion."""
    supervisor = _FakeSupervisor(tmp_path, failures={"preflight"})

    outcome = _run(tmp_path, _simulator_profile(), supervisor)
    result = json.loads(outcome.result_path.read_text(encoding="utf-8"))

    assert outcome.exit_code == 1
    assert outcome.evidence_status == "not_produced"
    assert not result["preflight_completed"]
    assert not result["runtime_validation_completed"]
    assert "validate" not in supervisor.ran
    assert supervisor.cleanup_called


def test_motion_evidence_requires_completed_validation(tmp_path: Path) -> None:
    """Evidence passes only after the configured validation command."""
    supervisor = _FakeSupervisor(tmp_path)

    outcome = _run(
        tmp_path,
        _simulator_profile(),
        supervisor,
        environment={
            "GITHUB_SHA": "0123456789abcdef",
            "GITHUB_RUN_ID": "42",
            "GITHUB_RUN_ATTEMPT": "3",
            "RUNNER_NAME": "runtime-runner",
            "ROS_DISTRO": "jazzy",
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        },
    )
    result = json.loads(outcome.result_path.read_text(encoding="utf-8"))

    assert outcome.exit_code == 0
    assert outcome.evidence_status == "passed"
    assert supervisor.ran == ["preflight", "readiness", "validate"]
    assert result["schema_version"] == 2
    assert result["provenance"] == {
        "github_sha": "0123456789abcdef",
        "github_run_attempt": "3",
        "github_run_id": "42",
        "rmw_implementation": "rmw_fastrtps_cpp",
        "ros_distro": "jazzy",
        "runner_name": "runtime-runner",
    }


def _fault(fault_id: str) -> FaultProfile:
    """Create one compact four-phase transaction for orchestration tests."""
    return FaultProfile(
        fault_id=fault_id,
        inject=(_command(f"{fault_id}_inject"),),
        observe=(_command(f"{fault_id}_observe"),),
        recover=(_command(f"{fault_id}_recover"),),
        verify_recovered=(_command(f"{fault_id}_verify"),),
    )


def test_all_faults_run_in_order_before_validation(tmp_path: Path) -> None:
    """A diagnostic profile must execute every transaction then validation."""
    supervisor = _FakeSupervisor(tmp_path)

    outcome = _run(
        tmp_path,
        _simulator_profile(faults=(_fault("first"), _fault("second"))),
        supervisor,
    )
    result = json.loads(outcome.result_path.read_text(encoding="utf-8"))

    assert outcome.exit_code == 0
    assert supervisor.ran == [
        "preflight",
        "readiness",
        "first_inject",
        "first_observe",
        "first_recover",
        "first_verify",
        "second_inject",
        "second_observe",
        "second_recover",
        "second_verify",
        "validate",
    ]
    assert [stage["stage"] for stage in result["stages"]] == [
        "preflight",
        "readiness",
        "fault.first.inject",
        "fault.first.observe",
        "fault.first.recover",
        "fault.first.verify_recovered",
        "fault.second.inject",
        "fault.second.observe",
        "fault.second.recover",
        "fault.second.verify_recovered",
        "validation",
    ]


def test_fault_failure_does_not_skip_recovery_later_faults_or_validation(
    tmp_path: Path,
) -> None:
    """Failures are accumulated only after recovery and remaining validation."""
    supervisor = _FakeSupervisor(tmp_path, failures={"first_observe"})

    outcome = _run(
        tmp_path,
        _simulator_profile(faults=(_fault("first"), _fault("second"))),
        supervisor,
    )

    assert outcome.exit_code == 1
    assert supervisor.ran == [
        "preflight",
        "readiness",
        "first_inject",
        "first_observe",
        "first_recover",
        "first_verify",
        "second_inject",
        "second_observe",
        "second_recover",
        "second_verify",
        "validate",
    ]


def test_recovery_failure_stops_new_faults_and_motion(tmp_path: Path) -> None:
    """Failed recovery permits verification but no new injection or motion."""
    supervisor = _FakeSupervisor(tmp_path, failures={"first_recover"})

    outcome = _run(
        tmp_path,
        _simulator_profile(faults=(_fault("first"), _fault("second"))),
        supervisor,
    )

    assert outcome.exit_code == 1
    assert supervisor.ran[-2:] == ["first_recover", "first_verify"]
    assert "second_inject" not in supervisor.ran
    assert "validate" not in supervisor.ran


def test_injection_failure_skips_observation_but_not_recovery(
    tmp_path: Path,
) -> None:
    """A partial injection must be cleared before later validation continues."""
    supervisor = _FakeSupervisor(tmp_path, failures={"first_inject"})

    outcome = _run(
        tmp_path,
        _simulator_profile(faults=(_fault("first"),)),
        supervisor,
    )

    assert outcome.exit_code == 1
    assert "first_observe" not in supervisor.ran
    assert supervisor.ran[-3:] == ["first_recover", "first_verify", "validate"]


def test_failed_retry_attempt_is_preserved_in_evidence(tmp_path: Path) -> None:
    """A passing poll must retain the failed attempt that preceded it."""
    profile = _simulator_profile()
    profile = replace(
        profile,
        readiness=(_command("readiness", retry_until_pass=True),),
    )
    supervisor = _FakeSupervisor(
        tmp_path,
        failure_counts={"readiness": 1},
    )

    outcome = _run(tmp_path, profile, supervisor)
    result = json.loads(outcome.result_path.read_text(encoding="utf-8"))
    readiness_attempts = [
        stage for stage in result["stages"]
        if stage["command"] == "readiness"
    ]

    assert outcome.exit_code == 0
    assert [stage["outcome"] for stage in readiness_attempts] == [
        "failed",
        "passed",
    ]
    assert readiness_attempts[0]["return_code"] is None
    assert "injected failure" in readiness_attempts[0]["error"]


def test_cleanup_failure_invalidates_success_without_hiding_it(
    tmp_path: Path,
) -> None:
    """Successful motion with failed cleanup is not valid runtime evidence."""
    supervisor = _FakeSupervisor(
        tmp_path,
        cleanup_errors=("owned group survived cleanup",),
    )

    outcome = _run(tmp_path, _simulator_profile(), supervisor)

    assert outcome.exit_code == 2
    assert outcome.primary_error is None
    assert outcome.evidence_status == "invalidated_by_cleanup_failure"
    assert outcome.cleanup_errors == ("owned group survived cleanup",)
