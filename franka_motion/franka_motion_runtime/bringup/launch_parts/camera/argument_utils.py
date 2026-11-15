"""
Shared helpers for grouped table-camera launch argument declarations.

Responsibility:
- Create matching ``LaunchConfiguration`` and ``DeclareLaunchArgument`` objects.
- Keep backend argument modules compact and consistent.

Does not own:
- Backend-specific defaults.
- Launch-file assembly order or mode conditions.
"""

from __future__ import annotations

from launch.actions import DeclareLaunchArgument
from launch.substitution import Substitution
from launch.substitutions import LaunchConfiguration

LaunchArgumentGroup = tuple[dict[str, LaunchConfiguration], list[DeclareLaunchArgument]]


def add_launch_argument(
    configs: dict[str, LaunchConfiguration],
    arguments: list[DeclareLaunchArgument],
    name: str,
    default_value: str | Substitution | None,
    description: str,
) -> None:
    """
    Add one declaration and matching LaunchConfiguration atomically.

    ``default_value=None`` declares a required argument: launch refuses to
    start and names the missing argument instead of silently using a path
    that only exists on one machine. An ``EnvironmentVariable`` substitution
    with no fallback of its own keeps that same fail-closed contract while
    letting one exported variable supply the value without retyping it on
    every command line.
    """
    configs[name] = LaunchConfiguration(name)
    arguments.append(
        DeclareLaunchArgument(
            name,
            default_value=default_value,
            description=description,
        )
    )
