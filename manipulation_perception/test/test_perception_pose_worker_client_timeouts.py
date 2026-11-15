"""
Test worker-client timeout and forced process-tree shutdown behavior.

Responsibility: Verify timeout recovery, worker restart, shutdown-command
interruption, and one-shot/persistent process-group cleanup.
"""

from pathlib import Path
import subprocess
import sys
import threading
import time

import pytest

from manipulation_perception.pose.pose_worker_client import JsonWorkerClient
from perception_process_test_support import (  # noqa: I100,I201
    _env,
    _GRANDCHILD_CODE,
    _kill_leftover_pid,
    _process_is_running,
    _wait_for_pid_file,
    _wait_for_process_exit,
    _wait_until,
    _write_grandchild_spawning_worker,
)


def test_json_worker_client_one_shot_timeout_captures_output_and_kills_children(tmp_path: Path):
    worker = tmp_path / 'noisy_timeout_worker.py'
    child_pid_file = tmp_path / 'noisy_timeout_child.pid'
    worker.write_text(
        "import subprocess, sys, time\n"
        "sys.stdin.read()\n"
        "print('stdout' * 50000, flush=True)\n"
        "print('stderr' * 50000, file=sys.stderr, flush=True)\n"
        f"subprocess.Popen([sys.executable, '-c', {_GRANDCHILD_CODE!r}, "
        f"{str(child_pid_file)!r}])\n"
        "time.sleep(30)\n"
    )
    client = JsonWorkerClient(
        worker_name='NoisyTimeoutWorker',
        python_path=sys.executable,
        worker_script=worker,
        env_factory=_env,
        timeout_sec=0.5,
        persistent=False,
    )
    child_pid = 0
    try:
        started_at = time.monotonic()
        with pytest.raises(subprocess.TimeoutExpired) as raised:
            client.request({'id': 18})
        assert time.monotonic() - started_at < 3.0
        assert raised.value.output == b'stdout' * 50000 + b'\n'
        assert raised.value.stderr == b'stderr' * 50000 + b'\n'
        child_pid = _wait_for_pid_file(child_pid_file)
        assert _wait_for_process_exit(child_pid)
    finally:
        client.stop()
        if child_pid:
            _kill_leftover_pid(child_pid)


def test_json_worker_client_timeout_stops_persistent_worker_and_restarts(tmp_path: Path):
    worker = tmp_path / 'timeout_persistent_worker.py'
    worker_pid_file = tmp_path / 'worker.pid'
    worker.write_text(
        "import json, os, sys, time\n"
        f"with open({str(worker_pid_file)!r}, 'w', encoding='utf-8') as pid_file:\n"
        "    pid_file.write(str(os.getpid()))\n"
        "    pid_file.flush()\n"
        "for line in sys.stdin:\n"
        "    request = json.loads(line)\n"
        "    if request.get('sleep'):\n"
        "        time.sleep(30)\n"
        "    print(json.dumps({'id': request['id'], 'ok': True, "
        "'pid': os.getpid()}), flush=True)\n"
    )
    restarts = []
    client = JsonWorkerClient(
        worker_name='TimeoutPersistentWorker',
        python_path=sys.executable,
        worker_script=worker,
        env_factory=_env,
        timeout_sec=0.5,
        persistent=True,
        on_worker_restart=lambda: restarts.append('restart'),
    )
    timed_out_pid = 0
    errors: list[Exception] = []

    def request_worker() -> None:
        try:
            client.request({'id': 14, 'sleep': True})
        except Exception as exc:
            errors.append(exc)

    try:
        first_response = client.request({'id': 13})
        timed_out_pid = first_response['pid']
        assert _process_is_running(timed_out_pid)

        thread = threading.Thread(target=request_worker, daemon=True)
        thread.start()
        thread.join(timeout=2.0)
        assert not thread.is_alive()
        assert len(errors) == 1
        assert isinstance(errors[0], subprocess.TimeoutExpired)
        assert _wait_for_process_exit(timed_out_pid)
        with client._process_lock:
            assert client._process is None

        response = client.request({'id': 15})
        assert response['id'] == 15
        assert response['ok'] is True
        assert response['pid'] != timed_out_pid
        assert restarts == ['restart']
    finally:
        client.stop()
        if timed_out_pid:
            _kill_leftover_pid(timed_out_pid)


def test_json_worker_client_timeout_kills_persistent_worker_process_group(tmp_path: Path):
    worker = tmp_path / 'timeout_grandchild_worker.py'
    grandchild_pid_file = tmp_path / 'grandchild.pid'
    _write_grandchild_spawning_worker(worker, grandchild_pid_file, persistent=True)
    client = JsonWorkerClient(
        worker_name='TimeoutGrandchildWorker',
        python_path=sys.executable,
        worker_script=worker,
        env_factory=_env,
        timeout_sec=0.5,
        persistent=True,
    )
    errors: list[Exception] = []

    def request_worker() -> None:
        try:
            client.request({'id': 16})
        except Exception as exc:
            errors.append(exc)

    thread = threading.Thread(target=request_worker, daemon=True)
    grandchild_pid = 0
    try:
        thread.start()
        grandchild_pid = _wait_for_pid_file(grandchild_pid_file)
        thread.join(timeout=2.0)
        assert not thread.is_alive()
        assert len(errors) == 1
        assert isinstance(errors[0], subprocess.TimeoutExpired)
        assert _wait_for_process_exit(grandchild_pid)
        with client._process_lock:
            assert client._process is None
    finally:
        client.stop()
        if grandchild_pid:
            _kill_leftover_pid(grandchild_pid)


def test_json_worker_client_stop_interrupts_active_persistent_request_with_shutdown_command(
    tmp_path: Path,
):
    worker = tmp_path / 'slow_shutdown_command_worker.py'
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
        worker_name='SlowShutdownCommandWorker',
        python_path=sys.executable,
        worker_script=worker,
        env_factory=_env,
        timeout_sec=30.0,
        persistent=True,
        shutdown_command={'command': 'shutdown'},
    )
    errors: list[Exception] = []

    def request_worker() -> None:
        try:
            client.request({'id': 17})
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
        thread.join(timeout=3.0)
        assert not thread.is_alive()
        assert time.monotonic() - started_at < 3.0
        assert errors
    finally:
        client.stop()


def test_json_worker_client_stop_interrupts_active_one_shot_request(tmp_path: Path):
    worker = tmp_path / 'slow_worker.py'
    worker.write_text(
        "import json, sys, time\n"
        "request = json.loads(sys.stdin.readline())\n"
        "time.sleep(30)\n"
        "print(json.dumps({'id': request['id'], 'ok': True}), flush=True)\n"
    )
    client = JsonWorkerClient(
        worker_name='SlowWorker',
        python_path=sys.executable,
        worker_script=worker,
        env_factory=_env,
        timeout_sec=30.0,
        persistent=False,
    )
    errors: list[Exception] = []

    def request_worker() -> None:
        try:
            client.request({'id': 10})
        except Exception as exc:
            errors.append(exc)

    def one_shot_is_active() -> bool:
        with client._process_lock:
            return client._one_shot_process is not None

    thread = threading.Thread(target=request_worker, daemon=True)
    try:
        thread.start()
        assert _wait_until(one_shot_is_active)
        started_at = time.monotonic()
        client.stop()
        thread.join(timeout=1.0)
        assert not thread.is_alive()
        assert time.monotonic() - started_at < 1.0
        assert errors
    finally:
        client.stop()


def test_json_worker_client_stop_kills_one_shot_worker_process_group(tmp_path: Path):
    worker = tmp_path / 'grandchild_one_shot_worker.py'
    grandchild_pid_file = tmp_path / 'grandchild.pid'
    _write_grandchild_spawning_worker(worker, grandchild_pid_file, persistent=False)
    client = JsonWorkerClient(
        worker_name='GrandchildOneShotWorker',
        python_path=sys.executable,
        worker_script=worker,
        env_factory=_env,
        timeout_sec=30.0,
        persistent=False,
    )
    errors: list[Exception] = []

    def request_worker() -> None:
        try:
            client.request({'id': 12})
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
