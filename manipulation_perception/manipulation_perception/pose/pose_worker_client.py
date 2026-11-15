"""
JSON-over-stdio subprocess client for pose-estimation workers.

Responsibility:
- Start optional persistent worker subprocesses.
- Send one JSON request and read the matching JSON response with timeout handling.
- Stop worker processes, close their streams and reject every later request.
- Interrupt an active persistent or one-shot worker during node shutdown.
- Notify an optional observer only when a persistent worker is replaced.

Does not own:
- Building backend-specific JSON request payloads.
- Backend environment construction.
- ROS logging, publishers or callbacks.
- Diagnostic policy or counter storage.
"""

from __future__ import annotations

from collections.abc import Callable
import json
import os
from pathlib import Path
import select
import subprocess
import tempfile
import threading
import time

from manipulation_perception.pose.subprocess_groups import (
    OwnedProcessGroup,
    start_process_group,
    terminate_process_group,
)
from manipulation_perception.pose.worker_protocol import (
    parse_worker_json_line,
    parse_worker_stdout,
)


LogCallback = Callable[[str], None]
EnvFactory = Callable[[], dict[str, str]]
TIMEOUT_TERMINATE_TIMEOUT_SEC = 0.2
TIMEOUT_KILL_TIMEOUT_SEC = 1.0


class _PersistentWorkerProcessError(RuntimeError):
    pass


class WorkerClientStoppedError(RuntimeError):
    """Raised when a request is attempted after permanent client shutdown."""

    pass


class JsonWorkerClient:
    """
    Own a persistent or one-shot JSON worker process group.

    ``request`` is serialized by a lock and may be called concurrently with
    ``stop``. Deadlines use ``time.monotonic`` and therefore ignore ROS clock
    jumps. ``stop`` permanently rejects new work, interrupts an active process
    group and closes streams; it is safe to call repeatedly.
    """

    def __init__(
        self,
        *,
        worker_name: str,
        python_path: str,
        worker_script: Path,
        env_factory: EnvFactory,
        timeout_sec: float,
        persistent: bool,
        verbose: bool = False,
        log_info: LogCallback | None = None,
        shutdown_command: dict | None = None,
        on_worker_restart: Callable[[], None] | None = None,
    ) -> None:
        self.worker_name = worker_name
        self.python_path = python_path
        self.worker_script = worker_script
        self.env_factory = env_factory
        self.timeout_sec = timeout_sec
        self.persistent = persistent
        self.verbose = verbose
        self.log_info = log_info
        self.shutdown_command = shutdown_command
        self._on_worker_restart = on_worker_restart or (lambda: None)
        self._request_lock = threading.Lock()
        self._process_lock = threading.Lock()
        self._stop_lock = threading.Lock()
        self._cleanup_failure = threading.Event()
        self._process: OwnedProcessGroup | None = None
        self._one_shot_process: OwnedProcessGroup | None = None
        self._stopped = False
        self._has_started_persistent_process = False

    def request(self, payload: dict) -> dict:
        """
        Send one ID-bearing payload and wait for its matching response.

        Timeout or protocol failure terminates a persistent worker so the next
        request starts cleanly. A stopped client raises
        ``WorkerClientStoppedError`` instead of restarting a process.
        """
        with self._request_lock:
            with self._process_lock:
                self._raise_if_stopped_locked()
            if self.persistent:
                process = self._ensure_process()
                try:
                    assert process.stdin is not None
                    process.stdin.write(json.dumps(payload) + "\n")
                    process.stdin.flush()
                    return self._read_response(process, payload["id"])
                except subprocess.TimeoutExpired:
                    self._discard_persistent_process(process)
                    raise
                except (BrokenPipeError, OSError, ValueError, _PersistentWorkerProcessError):
                    self._discard_persistent_process(process)
                    raise
            return self._run_once(payload)

    def stop(self) -> None:
        """Permanently close admission and terminate every owned process group."""
        with self._stop_lock:
            with self._process_lock:
                self._stopped = True
                process = self._process
                self._process = None
                one_shot_process = self._one_shot_process
                self._one_shot_process = None

            if process is not None:
                try:
                    self._stop_persistent_process(process)
                finally:
                    self._close_process_streams(process)

            if one_shot_process is not None:
                try:
                    self._terminate_process_group(one_shot_process)
                finally:
                    self._close_process_streams(one_shot_process)

    @property
    def has_cleanup_failure(self) -> bool:
        """Report a prior uncertain reap even after its handle was detached."""
        return self._cleanup_failure.is_set()

    def _terminate_process_group(self, process: OwnedProcessGroup, **timeouts: float) -> None:
        try:
            if not terminate_process_group(process, **timeouts):
                self._cleanup_failure.set()
        except BaseException:
            self._cleanup_failure.set()
            raise

    def _stop_persistent_process(self, process: OwnedProcessGroup) -> None:
        try:
            if process.poll_exit() is None and self.shutdown_command is not None:
                try:
                    if process.stdin is not None:
                        process.stdin.write(json.dumps(self.shutdown_command) + "\n")
                        process.stdin.flush()
                    process.wait_for_exit(timeout=2.0)
                except (BrokenPipeError, OSError, ValueError, subprocess.TimeoutExpired):
                    self._terminate_process_group(process)
                else:
                    self._terminate_process_group(process, terminate_timeout_sec=0.0)
                return

            self._terminate_process_group(process)
        except BaseException:
            self._cleanup_failure.set()
            raise

    def _ensure_process(self) -> OwnedProcessGroup:
        if not self.worker_script.is_file():
            raise FileNotFoundError(
                f"{self.worker_name} worker script не найден: "
                f"{self.worker_script}"
            )

        with self._process_lock:
            self._raise_if_stopped_locked()
            if self._process is not None and self._process.poll_exit() is None:
                return self._process
            exited_process = self._process
            self._process = None

        # Observing leader exit retains its identity until descendant cleanup.
        # The serialized request owns cleanup after detaching the old handle;
        # stop() can close admission while cleanup waits outside the state lock.
        if exited_process is not None:
            self._discard_persistent_process(exited_process)

        # Cache/environment preparation may perform slow filesystem I/O. The
        # request lock serializes it; stop must remain free to close admission.
        env = self.env_factory()
        env["PYTHONUNBUFFERED"] = "1"
        with self._process_lock:
            self._raise_if_stopped_locked()
            command = [self.python_path, str(self.worker_script)]
            self._process = start_process_group(
                command,
                env=env,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            if self._has_started_persistent_process:
                self._on_worker_restart()
            self._has_started_persistent_process = True
            if self.log_info is not None:
                self.log_info(
                    f"{self.worker_name} worker запущен: {' '.join(command)}"
                )
            return self._process

    def _run_once(self, payload: dict) -> dict:
        if not self.worker_script.is_file():
            raise FileNotFoundError(
                f"{self.worker_name} worker script не найден: "
                f"{self.worker_script}"
            )

        command = [self.python_path, str(self.worker_script)]
        # File capture avoids pipe backpressure without communicate(), whose
        # implicit reap would release the PGID while stop() can still signal it.
        with (
            tempfile.TemporaryFile(mode='w+') as input_file,
            tempfile.TemporaryFile(mode='w+') as output_file,
            tempfile.TemporaryFile(mode='w+') as error_file,
        ):
            input_file.write(json.dumps(payload) + "\n")
            input_file.seek(0)
            env = self.env_factory()
            with self._process_lock:
                self._raise_if_stopped_locked()
                process = start_process_group(
                    command,
                    stdin=input_file,
                    stdout=output_file,
                    stderr=error_file,
                    env=env,
                    text=True,
                )
                self._one_shot_process = process
            try:
                try:
                    process.wait_for_exit(timeout=self.timeout_sec)
                finally:
                    self._terminate_process_group(
                        process,
                        terminate_timeout_sec=TIMEOUT_TERMINATE_TIMEOUT_SEC,
                        kill_timeout_sec=TIMEOUT_KILL_TIMEOUT_SEC,
                    )
            except subprocess.TimeoutExpired as error:
                output_file.seek(0)
                error_file.seek(0)
                error.output = output_file.buffer.read()
                error.stderr = error_file.buffer.read()
                raise
            finally:
                with self._process_lock:
                    if self._one_shot_process is process:
                        self._one_shot_process = None

            output_file.seek(0)
            error_file.seek(0)
            stdout, stderr = output_file.read(), error_file.read()

        completed = subprocess.CompletedProcess(
            [self.python_path, str(self.worker_script)],
            process.returncode,
            stdout,
            stderr,
        )
        if completed.returncode != 0:
            raise RuntimeError(
                f"{self.worker_name} subprocess завершился с ошибкой "
                f"({completed.returncode}): {completed.stderr.strip() or completed.stdout.strip()}"
            )
        return parse_worker_stdout(completed.stdout, payload["id"], self.worker_name)

    def _read_response(self, process: OwnedProcessGroup, request_id: int) -> dict:
        assert process.stdout is not None
        stdout_fd = process.stdout.fileno()
        deadline = time.monotonic() + self.timeout_sec
        recent_lines: list[str] = []
        pending_text = ""

        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise subprocess.TimeoutExpired(self.worker_name.lower(), self.timeout_sec)
            if process.poll_exit() is not None:
                details = "\n".join(recent_lines[-10:])
                raise _PersistentWorkerProcessError(
                    f"{self.worker_name} worker завершился с кодом {process.returncode}: {details}"
                )

            ready, _, _ = select.select([stdout_fd], [], [], min(0.25, remaining))
            if not ready:
                continue

            chunk = os.read(stdout_fd, 4096)
            if not chunk:
                continue
            pending_text += chunk.decode(errors="replace")
            while "\n" in pending_text:
                line, pending_text = pending_text.split("\n", maxsplit=1)
                line = line.strip()
                if not line:
                    continue
                recent_lines.append(line)

                payload = parse_worker_json_line(line, request_id, self.worker_name)
                if payload is None:
                    if self.verbose and self.log_info is not None:
                        self.log_info(f"{self.worker_name} worker: {line}")
                    continue
                return payload

            if process.poll_exit() is not None and pending_text.strip():
                line = pending_text.strip()
                recent_lines.append(line)
                payload = parse_worker_json_line(line, request_id, self.worker_name)
                if payload is None:
                    if self.verbose and self.log_info is not None:
                        self.log_info(f"{self.worker_name} worker: {line}")
                    pending_text = ""
                    continue
                return payload

    def _discard_persistent_process(self, process: OwnedProcessGroup) -> None:
        with self._process_lock:
            if self._process is process:
                self._process = None
        try:
            self._terminate_process_group(
                process,
                terminate_timeout_sec=TIMEOUT_TERMINATE_TIMEOUT_SEC,
                kill_timeout_sec=TIMEOUT_KILL_TIMEOUT_SEC,
            )
        finally:
            self._close_process_streams(process)

    def _raise_if_stopped_locked(self) -> None:
        if self._stopped:
            raise WorkerClientStoppedError(
                f"{self.worker_name} worker client уже остановлен"
            )

    @staticmethod
    def _close_process_streams(process: OwnedProcessGroup) -> None:
        for stream in (process.stdin, process.stdout, process.stderr):
            if stream is not None:
                try:
                    stream.close()
                except OSError:
                    pass
