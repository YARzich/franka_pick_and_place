"""
Test persistent worker-client shutdown and stream-failure behavior.

Responsibility: Verify active-request interruption, process-group ownership,
graceful shutdown, and broken read/write recovery.
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import pytest

import manipulation_perception.pose.pose_worker_client as worker_client_module
from manipulation_perception.pose.pose_worker_client import JsonWorkerClient, WorkerClientStoppedError
from manipulation_perception.pose.subprocess_groups import (
    start_process_group,
    terminate_process_group,
)
from perception_process_test_support import (  # noqa: I100,I201
    _env,
    _GRANDCHILD_CODE,
    _kill_leftover_pid,
    _wait_for_pid_file,
    _wait_for_process_exit,
    _wait_until,
    _write_exiting_grandchild_worker,
    _write_graceful_grandchild_worker,
    _write_grandchild_spawning_worker,
    _write_idle_grandchild_worker,
)


@pytest.mark.parametrize('persistent', [False, True])
def test_stop_does_not_wait_for_environment_preparation_or_allow_late_spawn(
    tmp_path, monkeypatch, persistent,
):
    script = tmp_path / 'worker.py'
    script.write_text('# no process should be spawned\n')
    preparing = threading.Event()
    release_preparation = threading.Event()
    stop_finished = threading.Event()
    spawned = []
    request_errors = []

    def prepare_environment():
        preparing.set()
        assert release_preparation.wait(timeout=2.0)
        return _env()

    def unexpected_spawn(*_args, **_kwargs):
        spawned.append(True)
        raise AssertionError('a stopped client must not spawn')

    monkeypatch.setattr(worker_client_module, 'start_process_group', unexpected_spawn)
    client = JsonWorkerClient(
        worker_name='test', python_path=sys.executable, worker_script=script,
        env_factory=prepare_environment, timeout_sec=0.5, persistent=persistent,
    )

    def request():
        try:
            client.request({'id': 1})
        except Exception as error:
            request_errors.append(error)

    def stop():
        client.stop()
        stop_finished.set()

    requester = threading.Thread(target=request)
    stopper = threading.Thread(target=stop)
    requester.start()
    try:
        assert preparing.wait(timeout=0.5)
        stopper.start()
        stopped_while_preparation_blocked = stop_finished.wait(timeout=0.2)
    finally:
        release_preparation.set()
        requester.join(timeout=1.0)
        if stopper.ident is not None:
            stopper.join(timeout=1.0)
        client.stop()
    assert not requester.is_alive()
    assert not stopper.is_alive()
    assert stopped_while_preparation_blocked
    assert spawned == []
    assert len(request_errors) == 1
    assert isinstance(request_errors[0], WorkerClientStoppedError)


@pytest.mark.parametrize('cleanup_workers', [1, 2])
def test_process_group_retains_leader_identity_until_last_signal(
    tmp_path: Path,
    monkeypatch,
    cleanup_workers: int,
):
    child_pid_file = tmp_path / 'resistant_child.pid'
    command = (
        "import os, subprocess, sys, time\n"
        f"subprocess.Popen([sys.executable, '-c', {_GRANDCHILD_CODE!r}, "
        f"{str(child_pid_file)!r}])\n"
        f"while not os.path.exists({str(child_pid_file)!r}):\n"
        "    time.sleep(0.01)\n"
    )
    process = start_process_group(
        [sys.executable, '-c', command],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    child_pid = 0
    signals = []
    signals_after_reap = []
    original_killpg = os.killpg
    errors = []
    threads = []

    def inspect_signal(pgid, signum):
        if pgid == process.pid and signum != 0:
            signals.append(signum)
            try:
                os.waitid(os.P_PID, process.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)
            except ChildProcessError:
                signals_after_reap.append(signum)
        return original_killpg(pgid, signum)

    start_cleanup = threading.Barrier(cleanup_workers)

    def cleanup():
        try:
            start_cleanup.wait(timeout=1.0)
            terminate_process_group(process, terminate_timeout_sec=0.05, kill_timeout_sec=0.2)
        except Exception as exc:
            errors.append(exc)

    try:
        child_pid = _wait_for_pid_file(child_pid_file)
        assert _wait_until(
            lambda: os.waitid(
                os.P_PID, process.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT
            ) is not None
        )
        monkeypatch.setattr(os, 'killpg', inspect_signal)
        threads = [threading.Thread(target=cleanup, daemon=True) for _ in range(cleanup_workers)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=2.0)

        assert all(not thread.is_alive() for thread in threads)
        assert errors == []
        assert signal.SIGKILL in signals
        assert signals_after_reap == []
        assert _wait_for_process_exit(child_pid)
        with pytest.raises(ChildProcessError):
            os.waitid(os.P_PID, process.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)

        # A reused numeric PGID must not reactivate this retired owner's right
        # to send signals, even when the OS reports a new group at that number.
        late_signals = []
        monkeypatch.setattr(os, 'killpg', lambda pgid, signum: late_signals.append((pgid, signum)))
        terminate_process_group(process, terminate_timeout_sec=0.0, kill_timeout_sec=0.0)
        assert late_signals == []
    finally:
        monkeypatch.setattr(os, 'killpg', original_killpg)
        for thread in threads:
            thread.join(timeout=2.0)
        if child_pid:
            _kill_leftover_pid(child_pid)
        try:
            os.waitid(os.P_PID, process.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)
        except ChildProcessError:
            pass
        else:
            terminate_process_group(process, terminate_timeout_sec=0.0, kill_timeout_sec=0.2)


def test_process_group_repeated_cleanup_cannot_signal_reused_pgid(monkeypatch):
    process = start_process_group([sys.executable, '-c', 'import time; time.sleep(30)'])
    terminate_process_group(process, terminate_timeout_sec=0.0, kill_timeout_sec=0.5)
    with pytest.raises(ChildProcessError):
        os.waitid(os.P_PID, process.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)
    signals = []
    monkeypatch.setattr(os, 'killpg', lambda pgid, signum: signals.append((pgid, signum)))

    terminate_process_group(process, terminate_timeout_sec=0.0, kill_timeout_sec=0.0)
    terminate_process_group(process, terminate_timeout_sec=0.0, kill_timeout_sec=0.0)

    assert signals == []


@pytest.mark.parametrize('observation', ['poll', 'wait'])
def test_process_group_exit_observation_keeps_leader_waitable(observation: str):
    process = start_process_group([sys.executable, '-c', 'import sys; sys.exit(7)'])
    try:
        if observation == 'poll':
            assert _wait_until(lambda: process.poll_exit() is not None)
        else:
            assert process.wait_for_exit(timeout=1.0) == 7
        assert process.returncode == 7
        status = os.waitid(os.P_PID, process.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)
        assert status is not None
        assert status.si_status == 7
    finally:
        terminate_process_group(process, terminate_timeout_sec=0.0, kill_timeout_sec=0.5)


def test_process_group_lost_child_ownership_never_authorizes_group_signal(monkeypatch):
    process = start_process_group([sys.executable, '-c', 'pass'])
    # An external reaper violates exclusive ownership. Fail closed instead of
    # using the now-unverified numeric PGID to signal unrelated processes.
    os.waitpid(process.pid, 0)
    signals = []
    monkeypatch.setattr(os, 'killpg', lambda pgid, signum: signals.append((pgid, signum)))

    assert not terminate_process_group(process, terminate_timeout_sec=0.0, kill_timeout_sec=0.0)
    assert not terminate_process_group(process, terminate_timeout_sec=0.0, kill_timeout_sec=0.0)
    with pytest.raises(ChildProcessError):
        process.poll_exit()
    assert signals == []


def test_json_worker_client_stop_interrupts_active_persistent_request(tmp_path: Path):
    worker = tmp_path / 'slow_persistent_worker.py'
    worker.write_text(
        "import json, sys, time\n"
        "for line in sys.stdin:\n"
        "    request = json.loads(line)\n"
        "    if request.get('command') == 'shutdown':\n"
        "        break\n"
        "    print('started', flush=True)\n"
        "    time.sleep(30)\n"
        "    print(json.dumps({'id': request['id'], 'ok': True}), flush=True)\n"
    )
    client = JsonWorkerClient(
        worker_name='SlowPersistentWorker',
        python_path=sys.executable,
        worker_script=worker,
        env_factory=_env,
        timeout_sec=30.0,
        persistent=True,
    )
    errors: list[Exception] = []

    def request_worker() -> None:
        try:
            client.request({'id': 9})
        except Exception as exc:
            errors.append(exc)

    def persistent_is_active() -> bool:
        with client._process_lock:
            return client._process is not None

    thread = threading.Thread(target=request_worker, daemon=True)
    try:
        thread.start()
        assert _wait_until(persistent_is_active)
        started_at = time.monotonic()
        client.stop()
        thread.join(timeout=1.0)
        assert not thread.is_alive()
        assert time.monotonic() - started_at < 1.0
        assert errors
    finally:
        client.stop()


def test_json_worker_client_stop_kills_persistent_worker_process_group(tmp_path: Path):
    worker = tmp_path / 'grandchild_persistent_worker.py'
    grandchild_pid_file = tmp_path / 'grandchild.pid'
    _write_grandchild_spawning_worker(worker, grandchild_pid_file, persistent=True)
    client = JsonWorkerClient(
        worker_name='GrandchildPersistentWorker',
        python_path=sys.executable,
        worker_script=worker,
        env_factory=_env,
        timeout_sec=30.0,
        persistent=True,
    )
    errors: list[Exception] = []

    def request_worker() -> None:
        try:
            client.request({'id': 11})
        except Exception as exc:
            errors.append(exc)

    thread = threading.Thread(target=request_worker, daemon=True)
    grandchild_pid = 0
    try:
        thread.start()
        grandchild_pid = _wait_for_pid_file(grandchild_pid_file)
        client.stop()
        thread.join(timeout=1.0)
        assert not thread.is_alive()
        assert _wait_for_process_exit(grandchild_pid)
        assert errors
    finally:
        client.stop()
        if grandchild_pid:
            _kill_leftover_pid(grandchild_pid)


def test_json_worker_client_stop_kills_graceful_shutdown_process_group(tmp_path: Path):
    worker = tmp_path / 'graceful_grandchild_worker.py'
    grandchild_pid_file = tmp_path / 'grandchild.pid'
    _write_graceful_grandchild_worker(worker, grandchild_pid_file)
    client = JsonWorkerClient(
        worker_name='GracefulGrandchildWorker',
        python_path=sys.executable,
        worker_script=worker,
        env_factory=_env,
        timeout_sec=2.0,
        persistent=True,
        shutdown_command={'command': 'shutdown'},
    )
    grandchild_pid = 0

    try:
        assert client.request({'id': 13}) == {'id': 13, 'ok': True}
        grandchild_pid = _wait_for_pid_file(grandchild_pid_file)
        client.stop()
        assert _wait_for_process_exit(grandchild_pid)
    finally:
        client.stop()
        if grandchild_pid:
            _kill_leftover_pid(grandchild_pid)


def test_json_worker_client_read_failure_discards_persistent_worker_process_group(tmp_path: Path):
    worker = tmp_path / 'read_failure_grandchild_worker.py'
    grandchild_pid_file = tmp_path / 'grandchild.pid'
    _write_exiting_grandchild_worker(worker, grandchild_pid_file)
    client = JsonWorkerClient(
        worker_name='ReadFailureWorker',
        python_path=sys.executable,
        worker_script=worker,
        env_factory=_env,
        timeout_sec=2.0,
        persistent=True,
    )
    grandchild_pid = 0

    try:
        with pytest.raises(RuntimeError, match='ReadFailureWorker'):
            client.request({'id': 18})
        grandchild_pid = _wait_for_pid_file(grandchild_pid_file)
        assert _wait_for_process_exit(grandchild_pid)
        with client._process_lock:
            assert client._process is None
    finally:
        client.stop()
        if grandchild_pid:
            _kill_leftover_pid(grandchild_pid)


def test_json_worker_client_write_failure_discards_persistent_worker_process_group(tmp_path: Path):
    worker = tmp_path / 'write_failure_grandchild_worker.py'
    grandchild_pid_file = tmp_path / 'grandchild.pid'
    _write_idle_grandchild_worker(worker, grandchild_pid_file)
    client = JsonWorkerClient(
        worker_name='WriteFailureWorker',
        python_path=sys.executable,
        worker_script=worker,
        env_factory=_env,
        timeout_sec=2.0,
        persistent=True,
    )
    grandchild_pid = 0

    try:
        assert client.request({'id': 19}) == {'id': 19, 'ok': True}
        grandchild_pid = _wait_for_pid_file(grandchild_pid_file)
        with client._process_lock:
            assert client._process is not None
            assert client._process.stdin is not None
            client._process.stdin.close()
        with pytest.raises(ValueError):
            client.request({'id': 20})
        assert _wait_for_process_exit(grandchild_pid)
        with client._process_lock:
            assert client._process is None
    finally:
        client.stop()
        if grandchild_pid:
            _kill_leftover_pid(grandchild_pid)


def test_json_worker_client_restart_cleans_group_after_death_between_requests(tmp_path: Path):
    worker = tmp_path / 'idle_grandchild_worker.py'
    grandchild_pid_file = tmp_path / 'grandchild.pid'
    _write_idle_grandchild_worker(worker, grandchild_pid_file)
    client = JsonWorkerClient(
        worker_name='RestartWorker',
        python_path=sys.executable,
        worker_script=worker,
        env_factory=_env,
        timeout_sec=2.0,
        persistent=True,
    )
    grandchild_pid = 0
    try:
        assert client.request({'id': 21}) == {'id': 21, 'ok': True}
        grandchild_pid = _wait_for_pid_file(grandchild_pid_file)
        old_process = client._process
        assert old_process is not None
        os.kill(old_process.pid, signal.SIGKILL)
        old_process.wait_for_exit(timeout=1.0)

        assert client.request({'id': 22}) == {'id': 22, 'ok': True}

        assert _wait_for_process_exit(grandchild_pid)
        assert old_process.stdin.closed
        assert old_process.stdout.closed
    finally:
        client.stop()
        if grandchild_pid:
            _kill_leftover_pid(grandchild_pid)


def test_json_worker_client_stop_during_restart_cleanup_prevents_replacement(
    tmp_path: Path,
    monkeypatch,
):
    worker = tmp_path / 'idle_grandchild_worker.py'
    grandchild_pid_file = tmp_path / 'grandchild.pid'
    _write_idle_grandchild_worker(worker, grandchild_pid_file)
    client = JsonWorkerClient(
        worker_name='StoppedRestartWorker',
        python_path=sys.executable,
        worker_script=worker,
        env_factory=_env,
        timeout_sec=2.0,
        persistent=True,
    )
    cleanup_started = threading.Event()
    release_cleanup = threading.Event()
    stop_finished = threading.Event()
    errors = []
    grandchild_pid = 0
    request_thread = None
    stop_thread = None
    original_terminate = worker_client_module.terminate_process_group

    def request_worker():
        try:
            client.request({'id': 24})
        except Exception as exc:
            errors.append(exc)

    def stop_worker():
        client.stop()
        stop_finished.set()

    try:
        assert client.request({'id': 23}) == {'id': 23, 'ok': True}
        grandchild_pid = _wait_for_pid_file(grandchild_pid_file)
        old_process = client._process
        assert old_process is not None
        os.kill(old_process.pid, signal.SIGKILL)
        old_process.wait_for_exit(timeout=1.0)

        def delayed_terminate(process, **kwargs):
            if process is old_process:
                cleanup_started.set()
                assert release_cleanup.wait(timeout=3.0)
            return original_terminate(process, **kwargs)

        monkeypatch.setattr(worker_client_module, 'terminate_process_group', delayed_terminate)
        request_thread = threading.Thread(target=request_worker, daemon=True)
        request_thread.start()
        assert cleanup_started.wait(timeout=1.0)
        stop_thread = threading.Thread(target=stop_worker, daemon=True)
        stop_thread.start()
        assert stop_finished.wait(timeout=1.0)
        release_cleanup.set()
        request_thread.join(timeout=3.0)

        assert not request_thread.is_alive()
        assert len(errors) == 1
        assert isinstance(errors[0], WorkerClientStoppedError)
        assert client._process is None
        assert _wait_for_process_exit(grandchild_pid)
    finally:
        release_cleanup.set()
        if request_thread is not None:
            request_thread.join(timeout=3.0)
        if stop_thread is not None:
            stop_thread.join(timeout=3.0)
        client.stop()
        if grandchild_pid:
            _kill_leftover_pid(grandchild_pid)
