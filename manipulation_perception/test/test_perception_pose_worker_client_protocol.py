"""
Test the JSON worker client's basic request protocol.

Responsibility: Verify one-shot/persistent requests, response correlation,
missing scripts, and terminal idempotent stop behavior.
"""

from pathlib import Path
import sys

import pytest

from manipulation_perception.pose.pose_worker_client import (
    JsonWorkerClient,
    WorkerClientStoppedError,
)
from perception_process_test_support import _env  # noqa: I100,I201


def test_json_worker_client_runs_one_shot_worker_and_skips_stdout_noise(tmp_path: Path):
    worker = tmp_path / 'worker.py'
    worker.write_text(
        "import json, sys\n"
        "request = json.loads(sys.stdin.readline())\n"
        "print('boot log')\n"
        "print(json.dumps({'id': request['id'], 'ok': True, 'echo': request.get('value')}))\n"
    )
    client = JsonWorkerClient(
        worker_name='TestWorker',
        python_path=sys.executable,
        worker_script=worker,
        env_factory=_env,
        timeout_sec=2.0,
        persistent=False,
    )

    assert client.request({'id': 3, 'value': 'done'}) == {'id': 3, 'ok': True, 'echo': 'done'}


def test_json_worker_client_runs_persistent_worker_and_sends_shutdown(tmp_path: Path):
    worker = tmp_path / 'persistent_worker.py'
    worker.write_text(
        "import json, sys\n"
        "for line in sys.stdin:\n"
        "    request = json.loads(line)\n"
        "    if request.get('command') == 'shutdown':\n"
        "        break\n"
        "    print('noise', flush=True)\n"
        "    print(json.dumps({'id': request['id'], 'ok': True, "
        "'count': request['count']}), flush=True)\n"
    )
    logs: list[str] = []
    client = JsonWorkerClient(
        worker_name='PersistentWorker',
        python_path=sys.executable,
        worker_script=worker,
        env_factory=_env,
        timeout_sec=2.0,
        persistent=True,
        verbose=True,
        log_info=logs.append,
        shutdown_command={'command': 'shutdown'},
    )

    try:
        assert client.request({'id': 4, 'count': 1}) == {'id': 4, 'ok': True, 'count': 1}
        assert any('noise' in line for line in logs)
    finally:
        client.stop()


def test_json_worker_client_one_shot_handles_large_input_stdout_and_stderr(tmp_path: Path):
    worker = tmp_path / 'noisy_worker.py'
    worker.write_text(
        "import json, sys\n"
        "sys.stdout.write('log' * 100000 + '\\n')\n"
        "sys.stdout.flush()\n"
        "sys.stderr.write('warning' * 100000 + '\\n')\n"
        "sys.stderr.flush()\n"
        "request = json.load(sys.stdin)\n"
        "print(json.dumps({'id': request['id'], 'ok': True, 'echo': request['value']}))\n"
    )
    client = JsonWorkerClient(
        worker_name='NoisyWorker',
        python_path=sys.executable,
        worker_script=worker,
        env_factory=_env,
        timeout_sec=2.0,
        persistent=False,
    )
    payload = {'id': 5, 'value': 'large request' * 30000}

    try:
        assert client.request(payload) == {'id': 5, 'ok': True, 'echo': payload['value']}
    finally:
        client.stop()


def test_json_worker_client_reports_missing_worker_script(tmp_path: Path):
    client = JsonWorkerClient(
        worker_name='MissingWorker',
        python_path=sys.executable,
        worker_script=tmp_path / 'missing.py',
        env_factory=_env,
        timeout_sec=1.0,
        persistent=False,
    )

    with pytest.raises(FileNotFoundError, match='MissingWorker'):
        client.request({'id': 1})


@pytest.mark.parametrize('persistent', [False, True])
def test_json_worker_client_stop_is_idempotent_and_terminal(
    tmp_path: Path,
    persistent: bool,
):
    worker = tmp_path / 'must_not_start.py'
    started_file = tmp_path / 'started'
    worker.write_text(
        "from pathlib import Path\n"
        f"Path({str(started_file)!r}).write_text('started')\n"
    )
    client = JsonWorkerClient(
        worker_name='TerminalWorker',
        python_path=sys.executable,
        worker_script=worker,
        env_factory=_env,
        timeout_sec=1.0,
        persistent=persistent,
    )

    client.stop()
    client.stop()

    with pytest.raises(WorkerClientStoppedError, match='уже остановлен'):
        client.request({'id': 1})
    assert not started_file.exists()
