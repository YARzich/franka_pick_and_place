"""
Top-level runtime matrix value and configuration error boundary.

Responsibility:
- Expose exact profile lookup and the shared fail-closed configuration error.

Does not own:
- Profile invariants, JSON parsing, process execution or evidence output.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, TYPE_CHECKING

if TYPE_CHECKING:
    from franka_motion_runtime.validation.runtime_types import RuntimeProfile


class ConfigError(ValueError):
    """Report an invalid or unsupported runtime matrix contract."""


@dataclass(frozen=True)
class RuntimeMatrix:
    """Versioned collection of named runtime profiles."""

    matrix_id: str
    profiles: Mapping[str, RuntimeProfile]
    schema_version: int

    def profile(self, profile_id: str) -> RuntimeProfile:
        """Return one exact profile without fallback or fuzzy matching."""
        try:
            return self.profiles[profile_id]
        except KeyError as error:
            available = ", ".join(sorted(self.profiles))
            raise ConfigError(
                f"unknown runtime profile {profile_id!r}; expected: {available}"
            ) from error
