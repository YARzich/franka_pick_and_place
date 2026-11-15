"""
Shared primitives for pose backend runners.

Responsibility:
- Keep tiny backend-neutral callback aliases and monotonic request ids.

Does not own:
- Backend-specific request payloads.
- Worker subprocess lifecycle.
- ROS subscriptions, publishers or message conversion.
"""

from __future__ import annotations

from collections.abc import Callable
import threading
from typing import Any


LogCallback = Callable[[str], None]
EnvFactory = Callable[[], dict[str, str]]
SubprocessRun = Callable[..., Any]


class RequestIdCounter:
    """
    Generate thread-safe process-local positive request IDs for one client.

    Startup prewarm and inference may reserve IDs from separate threads before
    the worker client's request lock serializes their transport.
    """

    def __init__(self, start: int = 0) -> None:
        self._value = int(start)
        self._lock = threading.Lock()

    def next_id(self) -> int:
        """Advance and return the next monotonically increasing identifier."""
        with self._lock:
            self._value += 1
            return self._value
