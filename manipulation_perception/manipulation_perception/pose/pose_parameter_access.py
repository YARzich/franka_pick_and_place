"""
Small typed helpers for the pose node's public ROS parameter contract.

Responsibility:
    Read ROS parameter values into explicit bool, integer, float, string, and
    sequence representations used by grouped pose configuration loaders.

Does not own:
    Parameter declaration, validation, or dynamic updates.
"""

from __future__ import annotations

from collections.abc import Iterable
from pathlib import Path
from typing import Any, Protocol

from manipulation_perception.common.parameter_utils import bool_value


class ParameterNode(Protocol):
    """Subset of the rclpy Node parameter API used by pure config readers."""

    def declare_parameter(self, name: str, default_value: Any) -> Any:
        ...

    def get_parameter(self, name: str) -> Any:
        ...


ParameterSpec = tuple[str, Any]


def declare_parameters(
    node: ParameterNode,
    specifications: Iterable[ParameterSpec],
) -> None:
    """Declare a stable group of ROS parameters in specification order."""
    for name, default_value in specifications:
        node.declare_parameter(name, default_value)


class ParameterReader:
    """Normalize parameter values without retaining mutable node state."""

    def __init__(self, node: ParameterNode) -> None:
        self._node = node

    def value(self, name: str) -> Any:
        return self._node.get_parameter(name).value

    def text(self, name: str, *, fallback: str = "") -> str:
        return str(self.value(name)).strip() or fallback

    def boolean(self, name: str) -> bool:
        return bool_value(self.value(name), name)

    def path(self, name: str) -> Path:
        return Path(str(self.value(name))).expanduser()

    def optional_path(self, name: str) -> Path | None:
        value = str(self.value(name)).strip()
        return Path(value).expanduser() if value else None


__all__ = [
    "ParameterNode",
    "ParameterReader",
    "ParameterSpec",
    "declare_parameters",
]
