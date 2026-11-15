"""
MegaPose backend runner.

Responsibility:
- Run MegaPose through the persistent/one-shot worker when requested.
- Preserve the current CLI fallback behavior.
- Select the object pose from worker or CLI output.
- Terminate an active CLI fallback subprocess during shutdown.

Does not own:
- ROS image/depth subscriptions or publishers.
- Worker subprocess mechanics; those belong to `pose_worker_client`.
- Runtime example file writing; that belongs to `pose_runtime_files`.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path
import subprocess
import tempfile
import threading

import numpy as np

from manipulation_perception.pose.pose_backend_common import (
    EnvFactory,
    LogCallback,
    RequestIdCounter,
    SubprocessRun,
)
from manipulation_perception.pose.pose_estimation_io import (
    load_pose_from_output_files,
    select_pose_from_object_data,
)
from manipulation_perception.pose.pose_runtime_files import MegaPoseRuntimeExample
from manipulation_perception.pose.pose_worker_client import JsonWorkerClient
from manipulation_perception.pose.pose_worker_requests import (
    build_megapose_worker_request,
)
from manipulation_perception.pose.subprocess_groups import (
    OwnedProcessGroup,
    start_process_group,
    terminate_process_group,
)


CLI_TERMINATE_TIMEOUT_SEC = 0.2
CLI_KILL_TIMEOUT_SEC = 1.0


@dataclass(frozen=True)
class MegaPoseWorkerRequestConfig:
    """Immutable example/model batching contract sent to the JSON worker."""

    example_name: str
    example_dir: Path
    object_label: str
    bsz_images: int
    bsz_objects: int
    renderer_workers: int


@dataclass(frozen=True)
class MegaPoseCliConfig:
    """One-shot CLI fallback command, environment and steady timeout policy."""

    python_path: str
    example_name: str
    output_pose_paths: list[Path]
    object_label: str
    process_timeout_sec: float
    env_factory: EnvFactory


class MegaPoseBackendRunner:
    """
    Own worker and CLI fallback processes for one inference stream.

    Inference calls are serialized by the owning node. ``stop`` may run from a
    concurrent shutdown path: it prevents new work, stops the JSON client and
    terminates the active CLI process group. A worker failure may fall back to
    CLI only when no coarse estimates require the worker-only path.
    """

    def __init__(
        self,
        *,
        runtime_example: MegaPoseRuntimeExample,
        worker_client: JsonWorkerClient,
        worker_config: MegaPoseWorkerRequestConfig,
        cli_config: MegaPoseCliConfig,
        use_worker: bool,
        verbose: bool = False,
        log_info: LogCallback | None = None,
        log_warn: LogCallback | None = None,
        log_error: LogCallback | None = None,
        subprocess_run: SubprocessRun | None = None,
        on_worker_timeout: Callable[[], None] | None = None,
    ) -> None:
        self.runtime_example = runtime_example
        self.worker_client = worker_client
        self.worker_config = worker_config
        self.cli_config = cli_config
        self.use_worker = use_worker
        self.verbose = verbose
        self.log_info = log_info
        self.log_warn = log_warn
        self.log_error = log_error
        self.subprocess_run = subprocess_run
        self._on_worker_timeout = on_worker_timeout or (lambda: None)
        self._worker_request_ids = RequestIdCounter()
        self._stopped = threading.Event()
        self._cli_process_lock = threading.Lock()
        self._stop_lock = threading.Lock()
        self._cleanup_failure = threading.Event()
        self._cli_process: OwnedProcessGroup | None = None

    def stop(self) -> None:
        """Prevent new work and terminate both possible subprocess backends."""
        self._stopped.set()
        with self._stop_lock:
            self.worker_client.stop()
            self._stop_cli_process()

    def run_with_runtime_example(
        self,
        rgb: np.ndarray,
        depth: np.ndarray | None,
        use_depth: bool,
        model_name: str,
        camera_k: Sequence[float],
        bbox: tuple[int, int, int, int],
    ) -> dict | None:
        """Write runtime files, then infer a pose selected for the target label."""
        if self._stopped.is_set():
            return None
        self.runtime_example.prepare(rgb, depth, use_depth, camera_k, bbox)
        if self._stopped.is_set():
            return None
        return self.run(model_name, coarse_estimates=None)

    def run(self, model_name: str, coarse_estimates: dict | None) -> dict | None:
        """Run worker inference with the allowed CLI recovery policy."""
        if self._stopped.is_set():
            return None
        if self.use_worker or coarse_estimates is not None:
            try:
                response = self._request_worker_inference(model_name, coarse_estimates)
                pose_data = select_pose_from_object_data(
                    response.get("object_data", []),
                    self.worker_config.object_label,
                )
                if pose_data is not None:
                    if self.verbose and response.get("timing") and self.log_info is not None:
                        self.log_info(f"MegaPose worker timing: {response['timing']}")
                    return pose_data
                if self.log_warn is not None:
                    self.log_warn("MegaPose worker не вернул pose, читаю output file.")
            except Exception as exc:  # pragma: no cover - runtime fallback
                if self._stopped.is_set():
                    return None
                if isinstance(exc, subprocess.TimeoutExpired):
                    self._on_worker_timeout()
                if self.log_error is not None:
                    self.log_error(f"MegaPose worker недоступен, fallback на CLI: {exc}")
                self.worker_client.stop()

        if coarse_estimates is not None:
            return None
        if self._stopped.is_set():
            return None
        return self._run_cli(model_name)

    def _request_worker_inference(self, model_name: str, coarse_estimates: dict | None) -> dict:
        request = build_megapose_worker_request(
            request_id=self._worker_request_ids.next_id(),
            example_name=self.worker_config.example_name,
            example_dir=self.worker_config.example_dir,
            model_name=model_name,
            object_label=self.worker_config.object_label,
            bsz_images=self.worker_config.bsz_images,
            bsz_objects=self.worker_config.bsz_objects,
            renderer_workers=self.worker_config.renderer_workers,
            coarse_estimates=coarse_estimates,
        )
        return self.worker_client.request(request)

    def _run_cli(self, model_name: str) -> dict | None:
        if self._stopped.is_set():
            return None
        command = [
            self.cli_config.python_path,
            "-m",
            "megapose.scripts.run_inference_on_example",
            self.cli_config.example_name,
            "--run-inference",
            "--model",
            model_name,
        ]
        if self.subprocess_run is None:
            completed = self._run_cli_process(command)
        else:
            if self._stopped.is_set():
                return None
            completed = self.subprocess_run(
                command,
                check=False,
                env=self.cli_config.env_factory(),
                text=True,
                capture_output=True,
                timeout=self.cli_config.process_timeout_sec,
            )
        if completed is None:
            return None
        if completed.returncode != 0:
            if self.log_error is not None:
                self.log_error(
                    "MegaPose subprocess завершился с ошибкой "
                    f"({completed.returncode}): "
                    f"{completed.stderr.strip() or completed.stdout.strip()}"
                )
            return None
        if self.verbose and completed.stdout and self.log_info is not None:
            self.log_info(completed.stdout.strip())
        return load_pose_from_output_files(
            self.cli_config.output_pose_paths,
            self.cli_config.object_label,
        )

    def _run_cli_process(
        self, command: list[str]
    ) -> subprocess.CompletedProcess[str] | None:
        # Capture without communicate(): its implicit reap would let concurrent
        # stop() signal a reused numeric PGID after the leader has exited.
        with (
            tempfile.TemporaryFile(mode='w+') as output_file,
            tempfile.TemporaryFile(mode='w+') as error_file,
        ):
            with self._cli_process_lock:
                if self._stopped.is_set():
                    return None
                process = start_process_group(
                    command,
                    env=self.cli_config.env_factory(),
                    text=True,
                    stdout=output_file,
                    stderr=error_file,
                )
                self._cli_process = process
            try:
                try:
                    process.wait_for_exit(timeout=self.cli_config.process_timeout_sec)
                finally:
                    self._terminate_cli_process(process)
            except subprocess.TimeoutExpired as error:
                output_file.seek(0)
                error_file.seek(0)
                error.output = output_file.buffer.read()
                error.stderr = error_file.buffer.read()
                raise
            finally:
                with self._cli_process_lock:
                    if self._cli_process is process:
                        self._cli_process = None

            output_file.seek(0)
            error_file.seek(0)
            stdout, stderr = output_file.read(), error_file.read()

        return subprocess.CompletedProcess(command, process.returncode, stdout, stderr)

    def _stop_cli_process(self) -> None:
        with self._cli_process_lock:
            process = self._cli_process
            self._cli_process = None
        if process is not None:
            self._terminate_cli_process(process)

    @property
    def has_cleanup_failure(self) -> bool:
        """Report an uncertain CLI reap that forbids runtime-file cleanup."""
        return self._cleanup_failure.is_set()

    def _terminate_cli_process(self, process: OwnedProcessGroup) -> None:
        try:
            if not terminate_process_group(
                process,
                terminate_timeout_sec=CLI_TERMINATE_TIMEOUT_SEC,
                kill_timeout_sec=CLI_KILL_TIMEOUT_SEC,
            ):
                self._cleanup_failure.set()
        except BaseException:
            self._cleanup_failure.set()
            raise
