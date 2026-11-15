"""
Thread lifecycle helper for asynchronous pose inference.

Responsibility:
- Own the active inference worker thread reference and shutdown flag.
- Reject new worker starts after shutdown begins.
- Provide bounded joining for ROS node teardown.
- Use a daemon worker so a stuck backend cannot keep Python alive after bounded teardown.

Does not own:
- Backend inference execution.
- ROS publishers, subscriptions or logging.
- Frame throttling policy; that belongs to `pose_inference_gate`.
"""

from __future__ import annotations

from collections.abc import Callable
import threading
import time
from typing import Any


class InferenceWorkerLifecycle:
    """
    Own at most one inference thread and coordinate bounded node teardown.

    ``request_shutdown`` prevents future starts but does not interrupt an active
    backend call. ``join`` uses a wall/steady timeout supplied by the caller. A
    daemon worker intentionally permits process exit after that bound, so target
    code must not assume it will finish during shutdown.
    """

    def __init__(self, *, daemon: bool = True) -> None:
        self._daemon = daemon
        self._shutdown_requested = threading.Event()
        self._condition = threading.Condition()
        self._worker: threading.Thread | None = None
        self._starting = False

    def is_shutdown_requested(self) -> bool:
        """Return whether teardown has permanently closed worker admission."""
        return self._shutdown_requested.is_set()

    def request_shutdown(self) -> None:
        """Permanently reject future starts without blocking for active work."""
        self._shutdown_requested.set()

    def start(
        self,
        *,
        target: Callable[..., None],
        args: tuple[Any, ...],
        name: str,
    ) -> bool:
        """Start one owned worker, returning false if busy or shutting down."""
        def run_and_clear() -> None:
            try:
                target(*args)
            finally:
                self._clear_worker(threading.current_thread())

        with self._condition:
            if self._shutdown_requested.is_set():
                return False
            if self._worker is not None or self._starting:
                return False
            self._worker = threading.Thread(target=run_and_clear, name=name, daemon=self._daemon)
            worker = self._worker
            self._starting = True

        try:
            worker.start()
        except RuntimeError:
            with self._condition:
                if self._worker is worker:
                    self._worker = None
                self._starting = False
                self._condition.notify_all()
            raise
        with self._condition:
            self._starting = False
            self._condition.notify_all()
        return True

    def join(self, timeout_sec: float) -> bool:
        """Wait at most ``timeout_sec`` wall seconds for active work to finish."""
        timeout = max(0.0, float(timeout_sec))
        deadline = time.monotonic() + timeout
        with self._condition:
            while self._starting:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self._condition.wait(timeout=remaining)
            worker = self._worker
            if worker is not None and not worker.is_alive():
                self._worker = None
                worker = None
        if worker is None or worker is threading.current_thread():
            return True

        worker.join(timeout=max(0.0, deadline - time.monotonic()))
        return not worker.is_alive()

    def _clear_worker(self, worker: threading.Thread) -> None:
        with self._condition:
            if self._worker is worker:
                self._worker = None
            self._condition.notify_all()
