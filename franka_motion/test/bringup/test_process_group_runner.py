"""
Test the bounded process-group shutdown policy deterministically.

Responsibility:
- Verify one-time escalation and termination at the configured kill deadline.

Does not own:
- Spawning real child processes or exercising ROS launch shutdown.
"""

import signal

import pytest

import franka_motion_runtime.processes.process_group_runner as process_group_runner
from franka_motion_runtime.processes.process_group_runner import (
    ProcessGroupRunner,
    ProcessGroupRunnerConfig,
)


class _FakeChild:
    """Minimal running-child adapter used without creating a process."""

    pid = 12345

    def poll(self):
        """Report a child that has not exited yet."""
        return None


def test_runner_escalates_once_and_stops_after_kill_deadline(
    monkeypatch,
) -> None:
    """SIGTERM must escalate to SIGKILL under monotonic deadlines."""
    config = ProcessGroupRunnerConfig(
        command=["fake_child"],
        child_name="bounded_shutdown_test",
        shutdown_signal=signal.SIGTERM,
        shutdown_timeout_sec=0.0,
        kill_timeout_sec=0.1,
    )
    runner = ProcessGroupRunner(config)
    runner._child = _FakeChild()
    runner._child_pgid = _FakeChild.pid
    runner._shutdown_requested = True
    runner._shutdown_started_at = 10.0

    now = [10.0]
    sent_signals = []
    monkeypatch.setattr(
        process_group_runner.time,
        "monotonic",
        lambda: now[0],
    )
    monkeypatch.setattr(runner, "_process_group_exists", lambda: True)
    monkeypatch.setattr(
        runner,
        "_send_process_group_signal",
        lambda signum: sent_signals.append(signum) or True,
    )

    assert not runner._advance_shutdown()
    assert sent_signals == [signal.SIGTERM, signal.SIGKILL]

    now[0] = 10.11
    assert runner._advance_shutdown()


@pytest.mark.parametrize("scans", [[False], [True, False]])
def test_exit_observation_keeps_leader_unreaped_until_final_signal(monkeypatch, scans):
    """The child PID pins the group throughout descendant cleanup."""
    from types import SimpleNamespace
    import os

    runner = ProcessGroupRunner(ProcessGroupRunnerConfig(
        command=['child'], child_name='pin', shutdown_signal=signal.SIGTERM,
        shutdown_timeout_sec=10, kill_timeout_sec=0,
    ))
    events = []
    child = SimpleNamespace(pid=12345, returncode=None)

    def wait(timeout):
        assert runner._signaling_closed
        assert runner._child_pgid is None
        events.append('reap')
        child.returncode = 7
        return 7

    child.wait = wait
    runner._child = child
    runner._child_pgid = child.pid
    monkeypatch.setattr(runner, '_start_child', lambda: None)
    monkeypatch.setattr(process_group_runner.signal, 'signal', lambda *args: None)

    def waitid(kind, pid, options):
        assert kind == os.P_PID and pid == child.pid
        assert options & os.WNOWAIT
        events.append('observe')
        return SimpleNamespace(si_code=os.CLD_EXITED, si_status=7)

    monkeypatch.setattr(process_group_runner.os, 'waitid', waitid)
    monkeypatch.setattr(process_group_runner.os, 'killpg',
                        lambda pgid, signum: events.append(signum))
    scan_results = iter(scans)
    monkeypatch.setattr(runner, '_process_group_exists', lambda: next(scan_results))
    assert runner.run() == 7
    assert events == ['observe', signal.SIGTERM, signal.SIGKILL, 'reap']
    assert not runner._send_process_group_signal(signal.SIGKILL)
    assert events[-1] == 'reap'


def test_already_reaped_child_retires_group_before_any_signal(monkeypatch):
    """An externally reaped leader cannot establish ownership of a numeric PGID."""
    from types import SimpleNamespace

    runner = ProcessGroupRunner(ProcessGroupRunnerConfig(
        command=['child'], child_name='pin', shutdown_signal=signal.SIGTERM,
        shutdown_timeout_sec=0, kill_timeout_sec=0,
    ))
    runner._child = SimpleNamespace(pid=12345, returncode=0)
    runner._child_pgid = 12345
    assert runner._child_return_code() == 0
    assert not runner._send_process_group_signal(signal.SIGTERM)


def test_normal_exit_without_descendant_cleanup_sends_no_signal(monkeypatch):
    """cleanup_on_child_exit=False must preserve independently running helpers."""
    from types import SimpleNamespace
    import os

    runner = ProcessGroupRunner(ProcessGroupRunnerConfig(
        command=['child'], child_name='no-cleanup', shutdown_signal=signal.SIGTERM,
        shutdown_timeout_sec=0, kill_timeout_sec=0, cleanup_on_child_exit=False,
    ))
    runner._child = SimpleNamespace(pid=12345, returncode=None, wait=lambda timeout: 0)
    runner._child_pgid = 12345
    signals = []
    monkeypatch.setattr(runner, '_start_child', lambda: None)
    monkeypatch.setattr(process_group_runner.signal, 'signal', lambda *args: None)
    monkeypatch.setattr(process_group_runner.os, 'waitid', lambda *args:
                        SimpleNamespace(si_code=os.CLD_EXITED, si_status=0))
    monkeypatch.setattr(process_group_runner.os, 'killpg', lambda *args: signals.append(args))
    assert runner.run() == 0
    assert signals == []
    assert runner._signaling_closed
