"""
Test the correlated JSONL protocol used by external inference workers.

Responsibility: Ignore noise, match request IDs, surface errors, and close worker loops cleanly.
"""

import io
import json
from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.pose.worker_protocol import (  # noqa: E402,I100,I101,I201
    parse_worker_json_line,
    parse_worker_stdout,
    run_jsonl_worker_loop,
)


class _FakeWorker:
    def __init__(self) -> None:
        self.requests: list[dict] = []

    def run_request(self, request: dict) -> dict:
        self.requests.append(request)
        return {'id': request.get('id'), 'ok': True, 'echo': request.get('payload')}


class _FailingClosableWorker:
    def __init__(self) -> None:
        self.closed = False

    def run_request(self, _request: dict) -> dict:
        raise RuntimeError('backend failed')

    def close(self) -> None:
        self.closed = True


def test_parse_worker_json_line_ignores_noise_wrong_ids_and_non_object_json():
    assert parse_worker_json_line('not-json', 7, 'MegaPose') is None
    assert parse_worker_json_line('[1, 2, 3]', 7, 'MegaPose') is None
    assert parse_worker_json_line('{"id": 8, "ok": true}', 7, 'MegaPose') is None


def test_parse_worker_json_line_returns_matching_success_payload():
    payload = parse_worker_json_line('{"id": 7, "ok": true, "pose": [1, 2, 3]}', 7, 'MegaPose')

    assert payload == {'id': 7, 'ok': True, 'pose': [1, 2, 3]}


def test_parse_worker_json_line_raises_worker_error():
    with pytest.raises(RuntimeError, match='bad request'):
        parse_worker_json_line('{"id": 7, "ok": false, "error": "bad request"}', 7, 'MegaPose')


def test_parse_worker_stdout_skips_logs_until_matching_response():
    stdout = '\n'.join(
        [
            'worker boot log',
            '{"id": 1, "ok": true}',
            '[]',
            '{"id": 2, "ok": true, "result": "done"}',
        ]
    )

    assert parse_worker_stdout(stdout, 2, 'GigaPose') == {'id': 2, 'ok': True, 'result': 'done'}


def test_parse_worker_stdout_reports_missing_response():
    with pytest.raises(RuntimeError, match='GigaPose worker не вернул JSON response'):
        parse_worker_stdout('worker log\n{"id": 1, "ok": true}', 2, 'GigaPose')


def test_run_jsonl_worker_loop_handles_bad_json_requests_and_shutdown():
    worker = _FakeWorker()
    input_stream = io.StringIO(
        '\ninvalid-json\n[]\nnull\n{"id": 3, "payload": "ok"}\n'
        '{"command": "shutdown"}\n{"id": 4}\n'
    )
    output_stream = io.StringIO()

    assert run_jsonl_worker_loop(
        worker,
        input_stream=input_stream,
        output_stream=output_stream,
    ) == 0

    responses = [json.loads(line) for line in output_stream.getvalue().splitlines()]
    assert responses[0]['id'] is None
    assert responses[0]['ok'] is False
    assert responses[1]['error'] == 'request must be a JSON object'
    assert responses[2]['error'] == 'request must be a JSON object'
    assert responses[3] == {'id': 3, 'ok': True, 'echo': 'ok'}
    assert worker.requests == [{'id': 3, 'payload': 'ok'}]


def test_run_jsonl_worker_loop_optionally_closes_worker_after_failure():
    worker = _FailingClosableWorker()

    with pytest.raises(RuntimeError, match='backend failed'):
        run_jsonl_worker_loop(
            worker,
            input_stream=io.StringIO('{"id": 1}\n'),
            output_stream=io.StringIO(),
            close_worker=True,
        )

    assert worker.closed is True
