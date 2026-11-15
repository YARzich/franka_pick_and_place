"""
Frame throttling and single-flight guard for pose inference.

Responsibility:
- Apply frame skipping and minimum inference interval policy.
- Prevent overlapping inference workers.

Does not own:
- ROS subscriptions or message decoding.
- Backend inference execution.
- Publishing detections.
"""

from __future__ import annotations

import threading


class PoseInferenceGate:
    """
    Combine serialized frame decimation with a thread-safe single-flight gate.

    ``accepts_frame`` belongs to the ROS subscription thread and is not locked.
    ``try_start``, ``is_running`` and ``mark_done`` may cross the subscription
    and inference-worker threads. Interval checks use caller-supplied monotonic
    time so ROS simulation clock jumps cannot create overlapping inference.
    """

    def __init__(self, *, run_every_n_frames: int, min_interval_sec: float) -> None:
        self._run_every_n_frames = max(1, int(run_every_n_frames))
        self._min_interval_sec = max(0.0, float(min_interval_sec))
        self._frame_counter = 0
        self._last_inference_start = 0.0
        self._running = False
        self._lock = threading.Lock()

    def accepts_frame(self) -> bool:
        """Advance the callback-local frame counter and apply decimation."""
        self._frame_counter += 1
        return self._frame_counter % self._run_every_n_frames == 0

    def try_start(self, now_monotonic: float) -> bool:
        """Atomically reserve the sole inference slot when interval permits."""
        with self._lock:
            if now_monotonic - self._last_inference_start < self._min_interval_sec:
                return False
            if self._running:
                return False
            self._running = True
            self._last_inference_start = now_monotonic
            return True

    def is_running(self) -> bool:
        """Return whether an inference slot is currently reserved."""
        with self._lock:
            return self._running

    def mark_done(self) -> None:
        """Release the inference slot after every worker completion path."""
        with self._lock:
            self._running = False
