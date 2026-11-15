"""
Test the JSONL entrypoint of the GigaPose coarse worker.

Responsibility: Verify malformed requests, normal responses, and explicit shutdown handling.
"""

import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys


WORKER_PATH = (
    Path(__file__).resolve().parents[1]
    / 'scripts'
    / 'pose'
    / 'workers'
    / 'gigapose_worker.py'
)


def _load_worker_module():
    spec = importlib.util.spec_from_file_location(
        'gigapose_worker_entrypoint_for_test',
        WORKER_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class FakeWorker:
    def __init__(self) -> None:
        self.requests = []

    def run_request(self, request: dict) -> dict:
        self.requests.append(request)
        return {'id': request.get('id'), 'ok': True, 'echo': request.get('payload')}


def test_worker_loop_handles_bad_json_requests_and_shutdown(monkeypatch):
    module = _load_worker_module()
    worker = FakeWorker()
    stdin = io.StringIO(
        'bad-json\n'
        '{"id": 3, "payload": "ok"}\n'
        '{"command": "shutdown"}\n'
        '{"id": 4}\n'
    )
    stdout = io.StringIO()
    monkeypatch.setattr(sys, 'stdin', stdin)

    with contextlib.redirect_stdout(stdout):
        assert module.run_worker_loop(worker) == 0

    responses = [json.loads(line) for line in stdout.getvalue().splitlines()]
    assert responses[0]['id'] is None
    assert responses[0]['ok'] is False
    assert responses[1] == {'id': 3, 'ok': True, 'echo': 'ok'}
    assert worker.requests == [{'id': 3, 'payload': 'ok'}]
