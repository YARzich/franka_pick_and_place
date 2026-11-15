"""
Lifecycle coordinator for persistent GigaPose backend preloading.

Responsibility:
- Start coarse and refiner model loading in a strict sequence.
- Hold the first inference behind one readiness barrier.
- Release blocked inference and join the preload thread during shutdown.

Does not own:
- Worker subprocesses or their termination.
- Backend request construction.
- Frame inference and result interpretation.
"""

from __future__ import annotations

from collections.abc import Callable
import threading

from manipulation_perception.pose.pose_backend_common import LogCallback
from manipulation_perception.pose.pose_inference_worker import InferenceWorkerLifecycle
from manipulation_perception.pose.pose_worker_client import JsonWorkerClient


RequestFactory = Callable[[], dict]


class GigaPosePrewarmCoordinator:
    """Serialize persistent model loading and expose one inference barrier."""

    def __init__(
        self,
        *,
        coarse_client: JsonWorkerClient,
        refiner_client: JsonWorkerClient,
        log_info: LogCallback | None = None,
        log_warn: LogCallback | None = None,
    ) -> None:
        self._coarse_client = coarse_client
        self._refiner_client = refiner_client
        self._log_info = log_info
        self._log_warn = log_warn
        self._lock = threading.Lock()
        self._started = False
        self._shutdown = threading.Event()
        self._complete = threading.Event()
        self._complete.set()
        self._worker = InferenceWorkerLifecycle()

    def start(
        self,
        coarse_request_factory: RequestFactory,
        refiner_request_factory: RequestFactory,
    ) -> bool:
        """Start one model-only preload sequence for persistent clients."""
        with self._lock:
            if self._started or self._shutdown.is_set():
                return False
            if not bool(getattr(self._coarse_client, "persistent", False)):
                return False
            if not bool(getattr(self._refiner_client, "persistent", False)):
                return False
            coarse_request = coarse_request_factory()
            refiner_request = refiner_request_factory()
            self._started = True
            self._complete.clear()

        try:
            started = self._worker.start(
                target=self._run,
                args=(coarse_request, refiner_request),
                name="gigapose_backend_prewarm",
            )
        except RuntimeError:
            self._complete.set()
            raise
        if not started:
            self._complete.set()
        elif self._log_info is not None:
            self._log_info("GigaPose coarse и MegaPose refiner начали startup prewarm")
        return started

    def wait_until_ready(self) -> bool:
        """Block until both preload requests finish or shutdown starts."""
        self._complete.wait()
        return not self._shutdown.is_set()

    def request_shutdown(self) -> None:
        """Reject starts and immediately release inference waiting at the barrier."""
        self._shutdown.set()
        self._complete.set()
        self._worker.request_shutdown()

    def join(self, timeout_sec: float) -> bool:
        """Wait a bounded wall-clock interval for the preload thread."""
        return self._worker.join(timeout_sec=timeout_sec)

    def _run(self, coarse_request: dict, refiner_request: dict) -> None:
        try:
            if not self._request("GigaPose coarse", self._coarse_client, coarse_request):
                return
            if self._shutdown.is_set():
                return
            self._request("MegaPose refiner", self._refiner_client, refiner_request)
        finally:
            self._complete.set()

    def _request(
        self,
        backend_name: str,
        client: JsonWorkerClient,
        request: dict,
    ) -> bool:
        try:
            response = client.request(request)
            if response.get("prewarmed") is not True:
                raise RuntimeError("worker did not confirm prewarm")
        except Exception as error:  # noqa: BLE001 - external worker boundary
            if not self._shutdown.is_set() and self._log_warn is not None:
                self._log_warn(f"{backend_name} startup prewarm не выполнен: {error}")
            return False
        if not self._shutdown.is_set() and self._log_info is not None:
            self._log_info(f"{backend_name} startup prewarm завершён")
        return True
