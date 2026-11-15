"""
Small ROS parameter coercion helpers for perception nodes.

Responsibility:
- Convert raw ROS parameter values into plain Python types consistently.
- Fail early on ambiguous user-provided values.

Does not own:
- Parameter declaration.
- Node lifecycle or logging.
"""

from __future__ import annotations

from typing import Any


_TRUE_VALUES = {"1", "true", "t", "yes", "y", "on"}
_FALSE_VALUES = {"0", "false", "f", "no", "n", "off", ""}


def bool_value(value: Any, name: str) -> bool:
    """
    Coerce common ROS/YAML boolean forms or reject ambiguous input.

    Numeric zero maps to false and every other numeric value to true. Strings
    must match the explicit accepted sets; ``name`` is used in the validation
    error so operators can locate the invalid parameter.
    """
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        return value != 0
    if isinstance(value, float):
        return value != 0.0

    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in _TRUE_VALUES:
            return True
        if normalized in _FALSE_VALUES:
            return False

    raise ValueError(
        f"Параметр {name} должен быть boolean-значением, получено: {value!r}"
    )
