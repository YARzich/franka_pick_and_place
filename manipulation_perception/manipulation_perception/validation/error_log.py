"""
Bounded error retention for runtime evidence collectors.

Responsibility:
    Deduplicate and truncate observation errors while retaining a deterministic
    indication that additional failures were omitted.

Does not own:
    Logging, pass/fail policy, ROS callbacks, or artifact persistence.
"""

from __future__ import annotations

from collections.abc import Iterable


DEFAULT_ERROR_CAPACITY = 32
DEFAULT_ERROR_LENGTH = 512


class BoundedErrorLog:
    """Retain a small ordered set of representative validation failures."""

    def __init__(
        self,
        *,
        capacity: int = DEFAULT_ERROR_CAPACITY,
        maximum_length: int = DEFAULT_ERROR_LENGTH,
    ) -> None:
        if capacity <= 0 or maximum_length <= 0:
            raise ValueError("error-log bounds must be positive")
        self._capacity = int(capacity)
        self._maximum_length = int(maximum_length)
        self._messages: list[str] = []
        self._seen: set[str] = set()
        self._omitted_count = 0

    def add(self, message: object) -> None:
        """Retain one normalized unique message or count it as omitted."""
        text = str(message).strip()
        if not text:
            return
        if len(text) > self._maximum_length:
            text = text[: self._maximum_length - 3] + "..."
        if text in self._seen:
            return
        if len(self._messages) < self._capacity:
            self._seen.add(text)
            self._messages.append(text)
        else:
            self._omitted_count += 1

    def extend(self, messages: Iterable[object]) -> None:
        """Retain each message under the same fixed memory bound."""
        for message in messages:
            self.add(message)

    def as_tuple(self) -> tuple[str, ...]:
        """Return retained examples plus one deterministic truncation summary."""
        result = list(self._messages)
        if self._omitted_count:
            result.append(f"{self._omitted_count} additional errors omitted")
        return tuple(result)


__all__ = [
    "DEFAULT_ERROR_CAPACITY",
    "DEFAULT_ERROR_LENGTH",
    "BoundedErrorLog",
]
