"""
Primitive strict JSON value readers used by runtime matrix parsing.

Responsibility:
- Validate object, array, string, number, integer and boolean shapes.
- Reject missing and unknown object fields with path-aware errors.

Does not own:
- Runtime schema versions, profile relationships or file I/O.
"""

from __future__ import annotations

import math
from typing import Mapping

from franka_motion_runtime.validation.runtime_types import ConfigError


def check_keys(
    value: Mapping[str, object],
    required: set[str],
    optional: set[str],
    path: str,
) -> None:
    """Reject missing and unknown keys in one strict JSON object."""
    missing = required - set(value)
    unknown = set(value) - required - optional
    if missing:
        raise ConfigError(f"{path}: missing keys: {', '.join(sorted(missing))}")
    if unknown:
        raise ConfigError(f"{path}: unknown keys: {', '.join(sorted(unknown))}")


def object_value(value: object, path: str) -> dict[str, object]:
    """Return an object whose keys are all strings."""
    if not isinstance(value, dict) or any(
        not isinstance(key, str) for key in value
    ):
        raise ConfigError(f"{path} must be an object with string keys")
    return value


def array_value(value: object, path: str) -> list[object]:
    """Return a JSON array without coercing other iterables."""
    if not isinstance(value, list):
        raise ConfigError(f"{path} must be an array")
    return value


def string_tuple(value: object, path: str) -> tuple[str, ...]:
    """Return non-empty strings while preserving array order."""
    values = array_value(value, path)
    return tuple(
        non_empty_string(element, f"{path}[{index}]")
        for index, element in enumerate(values)
    )


def non_empty_string(value: object, path: str) -> str:
    """Return one non-empty string without whitespace normalization."""
    if not isinstance(value, str) or not value:
        raise ConfigError(f"{path} must be a non-empty string")
    return value


def number(value: object, path: str) -> float:
    """Return one finite number while rejecting JSON booleans."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{path} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ConfigError(f"{path} must be finite")
    return result


def integer(value: object, path: str) -> int:
    """Return one integer while rejecting JSON booleans."""
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError(f"{path} must be an integer")
    return value


def boolean(value: object, path: str) -> bool:
    """Return one exact JSON boolean without truthiness conversion."""
    if not isinstance(value, bool):
        raise ConfigError(f"{path} must be a boolean")
    return value
