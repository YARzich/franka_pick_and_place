"""
JSON-over-stdio worker protocol helpers.

Responsibility:
- Run the shared JSON-lines stdin/stdout loop used by pose workers.
- Parse worker stdout lines into matching request responses.
- Convert worker error payloads into Python exceptions.

Does not own:
- Subprocess creation or client-side timeout handling.
- Backend-specific request construction.
- ROS node threading/lifecycle.
"""

from __future__ import annotations

import json
import sys
from typing import Any, Protocol, TextIO


class JsonRequestWorker(Protocol):
    """Backend adapter invoked serially by the JSON-lines server loop."""

    def run_request(self, request: dict[str, Any]) -> dict[str, Any]:
        """Return one JSON-compatible response carrying the request ID."""
        ...


def run_jsonl_worker_loop(
    worker: JsonRequestWorker,
    *,
    input_stream: TextIO | None = None,
    output_stream: TextIO | None = None,
    close_worker: bool = False,
) -> int:
    """
    Serve JSON-lines requests until EOF or a shutdown command.

    The loop is deliberately single-threaded so backend GPU ownership remains
    serialized. Exactly one flushed JSON object is emitted for every non-shutdown
    input line, including malformed JSON. ``close_worker`` guarantees best-effort
    owner cleanup through the worker's optional ``close`` method.
    """
    source = sys.stdin if input_stream is None else input_stream
    sink = sys.stdout if output_stream is None else output_stream

    try:
        for line in source:
            line = line.strip()
            if not line:
                continue
            try:
                request = json.loads(line)
            except json.JSONDecodeError as exc:
                response = {"id": None, "ok": False, "error": f"bad json: {exc}"}
            else:
                if not isinstance(request, dict):
                    response = {
                        "id": None,
                        "ok": False,
                        "error": "request must be a JSON object",
                    }
                elif request.get("command") == "shutdown":
                    break
                else:
                    response = worker.run_request(request)

            print(json.dumps(response), file=sink, flush=True)
    finally:
        if close_worker:
            close = getattr(worker, "close", None)
            if callable(close):
                close()

    return 0


def parse_worker_json_line(line: str, request_id: int, worker_name: str) -> dict[str, Any] | None:
    """Return the matching successful response, ignoring logs and other IDs."""
    try:
        payload = json.loads(line.strip())
    except json.JSONDecodeError:
        return None

    if not isinstance(payload, dict):
        return None
    if payload.get("id") != request_id:
        return None
    if not payload.get("ok", False):
        raise RuntimeError(str(payload.get("error", "unknown worker error")))
    return payload


def parse_worker_stdout(stdout: str, request_id: int, worker_name: str) -> dict[str, Any]:
    """Find one matching response in mixed worker output or raise on absence."""
    for line in stdout.splitlines():
        payload = parse_worker_json_line(line, request_id, worker_name)
        if payload is not None:
            return payload
    raise RuntimeError(f"{worker_name} worker не вернул JSON response")
