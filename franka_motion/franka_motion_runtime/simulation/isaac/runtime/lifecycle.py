"""
Owned Isaac runtime resource cleanup.

Responsibility:
- Track partially constructed World and SimulationApp resources.
- Attempt all cleanup steps while preserving the primary setup/runtime error.

Does not own:
- Creating Isaac objects or deciding simulation-loop behavior.
- Process signals, ROS graphs or scene assembly.
"""

from __future__ import annotations

from dataclasses import dataclass
import sys
from typing import Any, Callable, TypeVar

_ResultT = TypeVar("_ResultT")


@dataclass
class IsaacRuntimeOwner:
    """Own partially initialized runtime objects until deterministic cleanup."""

    simulation_app: Any | None = None
    world: Any | None = None
    world_cls: Any | None = None

    def close(self) -> None:
        """Release all resources currently registered with this owner."""
        close_world_and_app(self.world, self.simulation_app, self.world_cls)


def run_with_runtime_owner(
    callback: Callable[[IsaacRuntimeOwner], _ResultT],
) -> _ResultT:
    """Run a callback with an owner that always cleans partial resources."""
    owner = IsaacRuntimeOwner()
    try:
        return callback(owner)
    finally:
        owner.close()


def close_world_and_app(
    world: Any | None,
    simulation_app: Any | None,
    world_cls: Any | None,
) -> None:
    """Attempt every cleanup step without masking an active primary error."""
    primary_error = sys.exc_info()[1]
    cleanup_errors: list[Exception] = []

    if world is not None:
        _attempt_cleanup(world.stop, cleanup_errors)
    if world_cls is not None:
        _attempt_cleanup(world_cls.clear_instance, cleanup_errors)
    if simulation_app is not None:
        _attempt_cleanup(simulation_app.close, cleanup_errors)

    if cleanup_errors and primary_error is not None:
        if hasattr(primary_error, "add_note"):
            primary_error.add_note(
                "Isaac cleanup also failed: "
                + "; ".join(str(error) for error in cleanup_errors)
            )
        return
    if cleanup_errors:
        raise RuntimeError("Isaac runtime cleanup failed") from cleanup_errors[0]


def _attempt_cleanup(callback, errors: list[Exception]) -> None:
    try:
        callback()
    except Exception as error:  # Cleanup must continue through app.close().
        errors.append(error)
