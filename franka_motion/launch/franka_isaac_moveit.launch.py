"""
Isaac Sim empty-scene MoveIt launch entrypoint for Franka Panda.

Responsibility:
- Select immutable resources and the disabled Octomap profile for the empty scene.
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
from franka_motion_runtime.simulation.isaac.scenarios import IsaacMoveItScenario


def generate_launch_description() -> LaunchDescription:
    """Create the canonical Isaac empty-scene MoveIt stack."""
    scenario = IsaacMoveItScenario()
    launch_description = create_isaac_moveit_description(
        scenario,
        octomap_profile=OctomapProfile.DISABLED,
        start_transfer_server_default=False,
    )
    return launch_description
