"""
Gazebo scenario data passed by Franka launch entrypoints.

Responsibility:
- Define immutable inputs required to assemble a Franka Gazebo scenario.
- Carry world, temporary-file, and optional base-pose choices between helpers.

Does not own:
- Resource-path lookup or creation of simulator, bridge, or controller actions.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class GazeboScenario:
    """Immutable simulator inputs resolved by the Franka bringup layer."""

    world_file: str
    tmp_urdf_path: Path
    tmp_bridge_config_path: Path
    base_pose: dict[str, str] | None = None
