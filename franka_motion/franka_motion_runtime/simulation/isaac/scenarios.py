"""
Isaac MoveIt scenario data passed from public launch entrypoints.

Responsibility:
- Define immutable Isaac scene and robot options consumed by launch helpers.
- Carry table/cube/camera flags without coupling entrypoints to command building.

Does not own:
- Resolving package resource paths.
- Creating Isaac process, controller or MoveIt actions.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class IsaacMoveItScenario:
    """Immutable simulator inputs selected by a bringup scenario."""

    scene_usd_file: str | None = None
    include_isaac_experience_arg: bool = False
    no_default_ground: bool = False
    add_cube: bool = False
    cube_size: float = 0.05
    cube_position: tuple[float, float, float] = (0.4, 0.0, 0.03)


# Both table entrypoints must spawn the same task object described by the
# scenario-owned Isaac object catalog.
ISAAC_TABLE_SCENARIO = IsaacMoveItScenario(
    scene_usd_file="table_world_usd_clean.usd",
    include_isaac_experience_arg=True,
    no_default_ground=True,
    add_cube=True,
)
