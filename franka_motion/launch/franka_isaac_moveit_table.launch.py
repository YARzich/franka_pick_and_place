"""
Isaac Sim table-scene MoveIt launch entrypoint for Franka Panda.

Responsibility:
- Select immutable table resources, collision geometry and Octomap profile.
- Delegate graph assembly to the standard Isaac MoveIt factory.

Does not own:
- Simulator, controller, planner or perception implementation.
"""

from __future__ import annotations

from launch import LaunchDescription

from franka_motion_runtime.bringup.isaac_moveit import (
    create_isaac_moveit_description,
)
from franka_motion_runtime.bringup.launch_parts.moveit import OctomapProfile
from franka_motion_runtime.bringup.launch_parts.planning_scene import (
    ISAAC_TABLE_COLLISION,
)
from franka_motion_runtime.simulation.isaac.scenarios import ISAAC_TABLE_SCENARIO


def generate_launch_description() -> LaunchDescription:
    """Create the canonical Isaac table MoveIt stack."""
    launch_description = create_isaac_moveit_description(
        ISAAC_TABLE_SCENARIO,
        octomap_profile=OctomapProfile.DISABLED,
        start_transfer_server_default=True,
        table_collision_profile=ISAAC_TABLE_COLLISION,
    )
    return launch_description
