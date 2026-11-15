"""
Test deadlines, process registry, and bounded process-group ownership.

Responsibility: Verify identity-safe adoption, escalation, idempotent cleanup, and command results.
"""

import json
import os
from pathlib import Path
import signal
import sys

import pytest

from franka_motion_runtime.validation.runtime_config import CommandSpec
from franka_motion_runtime.validation.runtime_output import validate_command_result
from franka_motion_runtime.validation.runtime_process import (
    CommandResult,
    CommandTimeoutError,
    Deadline,
    DeadlineExpired,
    OwnedGroupCleaner,
    ProcessError,
    ProcessIdentity,
    ProcessRegistry,
    ProcessSupervisor,
)


class _FakeInspector:
    def __init__(self, groups: set[int]) -> None:
        self.groups = groups

    def owned_processes(self, run_token: str) -> tuple[ProcessIdentity, ...]:
        assert run_token == "run-token"
        return tuple(
            ProcessIdentity(
                pid=process_group_id,
                process_group_id=process_group_id,
                start_ticks=100 + process_group_id,
            )
            for process_group_id in sorted(self.groups)
        )


class _FakeClock:
    def __init__(self) -> None:
        self.now = 10.0

    def __call__(self) -> float:
        return self.now

    def wait(self, duration: float) -> None:
        self.now += duration


def test_deadline_clips_each_stage_to_one_absolute_expiry() -> None:
    """Stage budgets cannot extend the shared monotonic deadline."""
    clock = _FakeClock()
    deadline = Deadline.after(5.0, clock=clock)

    assert deadline.clip(10.0) == pytest.approx(5.0)
    clock.now = 13.0
    assert deadline.clip(10.0) == pytest.approx(2.0)
    clock.now = 15.0

    with pytest.raises(DeadlineExpired):
        deadline.clip(1.0)


def test_cleanup_joins_graceful_owned_groups_without_name_matching() -> None:
    """Token-owned groups should terminate without broad process matching."""
    inspector = _FakeInspector({101, 202})
    signaled = []

    def signal_group(process_group_id: int, signal_number: int) -> None:
        signaled.append((process_group_id, signal_number))
        inspector.groups.discard(process_group_id)

    cleaner = OwnedGroupCleaner(
        inspector,
        signal_group=signal_group,
        waiter=lambda duration: None,
    )

    report = cleaner.cleanup("run-token")

    assert report.clean
    assert report.terminated_groups == (202, 101)
    assert report.killed_groups == ()
    assert signaled == [
        (202, signal.SIGTERM),
        (101, signal.SIGTERM),
    ]


def test_cleanup_escalates_stubborn_group_and_waits_for_absence() -> None:
    """A surviving TERM group must receive KILL and disappear before pass."""
    inspector = _FakeInspector({303})
    clock = _FakeClock()
    signaled = []

    def signal_group(process_group_id: int, signal_number: int) -> None:
        signaled.append((process_group_id, signal_number))
        if signal_number == signal.SIGKILL:
            inspector.groups.clear()

    cleaner = OwnedGroupCleaner(
        inspector,
        signal_group=signal_group,
        waiter=clock.wait,
        clock=clock,
    )

    report = cleaner.cleanup(
        "run-token",
        terminate_grace_sec=0.1,
        kill_grace_sec=0.1,
    )

    assert report.clean
    assert report.terminated_groups == (303,)
    assert report.killed_groups == (303,)
    assert signaled == [
        (303, signal.SIGTERM),
        (303, signal.SIGKILL),
    ]


def test_cleanup_is_idempotent_after_owned_groups_disappear() -> None:
    """A repeated cleanup pass must remain a successful no-op."""
    inspector = _FakeInspector(set())
    cleaner = OwnedGroupCleaner(
        inspector,
        signal_group=lambda process_group_id, signal_number: None,
        waiter=lambda duration: None,
    )

    first_report = cleaner.cleanup("run-token")
    second_report = cleaner.cleanup("run-token")

    assert first_report.clean
    assert second_report.clean
    assert first_report.terminated_groups == ()
    assert second_report.terminated_groups == ()


def test_registry_persists_identity_without_argv_or_environment(
    tmp_path: Path,
) -> None:
    """State must be sufficient for ownership without leaking configuration."""
    state_path = tmp_path / "runtime-state.json"
    registry = ProcessRegistry(state_path)

    registry.write(
        "run-token",
        (ProcessIdentity(pid=7, process_group_id=7, start_ticks=99),),
    )
    value = json.loads(state_path.read_text(encoding="utf-8"))

    assert value == {
        "schema_version": 1,
        "run_token": "run-token",
        "roots": [
            {
                "pid": 7,
                "process_group_id": 7,
                "start_ticks": 99,
            }
        ],
    }
    assert "robot" not in state_path.read_text(encoding="utf-8").lower()


def test_registry_requires_explicit_same_token_adoption(tmp_path: Path) -> None:
    """A new run must not overwrite ownership state from an interrupted run."""
    registry = ProcessRegistry(tmp_path / "runtime-state.json")
    registry.claim("run-token")

    with pytest.raises(ProcessError, match="cleanup is required"):
        registry.claim("other-token")
    with pytest.raises(ProcessError, match="cleanup is required"):
        registry.claim("run-token")

    registry.claim("run-token", adopt_existing=True)
    assert registry.read_token() == "run-token"


def test_completed_command_requires_expected_exit_and_output(
    tmp_path: Path,
) -> None:
    """A zero exit alone must not satisfy a configured output contract."""
    log_path = tmp_path / "command.log"
    log_path.write_text("controller active\n", encoding="utf-8")
    command = CommandSpec(
        name="controller_probe",
        argv=("probe",),
        timeout_sec=1.0,
        stdout_regex=(r"^controller active$",),
    )
    result = CommandResult(
        name=command.name,
        argv=command.argv,
        return_code=0,
        elapsed_sec=0.1,
        log_path=log_path,
    )

    validate_command_result(command, result)

    log_path.write_text("controller inactive\n", encoding="utf-8")
    with pytest.raises(ProcessError, match="output did not match"):
        validate_command_result(command, result)


def test_expected_return_code_is_explicit_and_enforced(tmp_path: Path) -> None:
    """Admission rejection may pass only when return code 1 is declared."""
    log_path = tmp_path / "admission.log"
    log_path.write_text("motion blocked by health\n", encoding="utf-8")
    result = CommandResult(
        name="admission",
        argv=("client",),
        return_code=1,
        elapsed_sec=0.1,
        log_path=log_path,
    )
    default_command = CommandSpec(
        name="default_admission",
        argv=("client",),
        timeout_sec=1.0,
    )
    expected_failure = CommandSpec(
        name="expected_admission_failure",
        argv=("client",),
        timeout_sec=1.0,
        expected_return_codes=(1,),
    )

    with pytest.raises(ProcessError, match="expected \\(0,\\)"):
        validate_command_result(default_command, result)
    validate_command_result(expected_failure, result)


def test_expired_deadline_fails_before_process_spawn(tmp_path: Path) -> None:
    """An exhausted profile cannot create a child before fault recovery."""
    spawned = []

    def forbidden_spawn(*args, **kwargs):
        spawned.append((args, kwargs))
        raise AssertionError("process must not be spawned")

    clock = _FakeClock()
    supervisor = ProcessSupervisor(
        run_token="expired-before-spawn",
        state_path=tmp_path / "state.json",
        artifact_directory=tmp_path / "logs",
        environment=os.environ,
        popen_factory=forbidden_spawn,
        clock=clock,
    )
    command = CommandSpec(
        name="expired_command",
        argv=("never-start",),
        timeout_sec=1.0,
    )
    deadline = Deadline(expires_at=clock.now, clock=clock)

    with pytest.raises(DeadlineExpired):
        supervisor.run_command(
            command,
            command.argv,
            deadline=deadline,
            attempt=1,
        )

    assert not spawned
    assert supervisor.cleanup().clean


def test_timeout_joins_child_with_independent_cleanup_budget(
    tmp_path: Path,
) -> None:
    """A command timeout must leave no live or registered direct child."""
    supervisor = ProcessSupervisor(
        run_token=f"timeout-child-{os.getpid()}",
        state_path=tmp_path / "state.json",
        artifact_directory=tmp_path / "logs",
        environment=os.environ,
    )
    command = CommandSpec(
        name="sleeping_child",
        argv=(sys.executable, "-c", "import time; time.sleep(30)"),
        timeout_sec=0.05,
    )

    with pytest.raises(CommandTimeoutError) as captured:
        supervisor.run_command(
            command,
            command.argv,
            deadline=Deadline.after(0.05),
            attempt=1,
        )

    assert captured.value.result.return_code is not None
    state = json.loads((tmp_path / "state.json").read_text(encoding="utf-8"))
    assert state["roots"] == []
    assert supervisor.cleanup().clean
