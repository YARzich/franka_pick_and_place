"""
Small fail-fast validators shared by ROS parameter adapters.

Responsibility: Report missing scenario-owned strings and invalid positive values
before publishers, subscriptions, models, or worker processes are created.
"""

from __future__ import annotations

from collections.abc import Mapping
import math
import re


_IDENTIFIER_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


def require_non_empty(values: Mapping[str, object], *, context: str) -> None:
    """Raise one actionable error listing every missing string parameter."""
    missing = sorted(name for name, value in values.items() if not str(value).strip())
    if missing:
        raise ValueError(f"{context}: required parameters are empty: {', '.join(missing)}")


def require_positive(value: float, name: str, *, context: str) -> None:
    """Require a finite value greater than zero for a metric configuration."""
    numeric_value = float(value)
    if not math.isfinite(numeric_value) or numeric_value <= 0.0:
        raise ValueError(f"{context}: {name} must be finite and greater than zero")


def require_non_negative(value: float, name: str, *, context: str) -> None:
    """Require a finite metric value that may explicitly be zero."""
    numeric_value = float(value)
    if not math.isfinite(numeric_value) or numeric_value < 0.0:
        raise ValueError(
            f"{context}: {name} must be finite and greater than or equal to zero"
        )


def require_identifier(value: str, name: str, *, context: str) -> None:
    """Reject path separators and unstable free-form backend identifiers."""
    if _IDENTIFIER_PATTERN.fullmatch(str(value)) is None:
        raise ValueError(
            f"{context}: {name} must contain only letters, digits, '.', '-', or '_'"
        )


def is_valid_frame_id(value: object) -> bool:
    """Return whether a value is one canonical relative TF frame identifier."""
    frame_id = str(value)
    return bool(
        frame_id
        and frame_id == frame_id.strip()
        and not frame_id.startswith("/")
        and not frame_id.endswith("/")
        and "//" not in frame_id
        and not any(character.isspace() or ord(character) < 32 for character in frame_id)
    )


def require_frame_id(value: str, name: str, *, context: str) -> None:
    """Require a non-empty REP-105-style frame without a leading slash."""
    if not is_valid_frame_id(value):
        raise ValueError(
            f"{context}: {name} must be non-empty, contain no whitespace, "
            "and be a canonical relative frame without empty path segments"
        )
