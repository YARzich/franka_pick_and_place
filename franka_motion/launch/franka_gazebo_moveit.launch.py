"""
Gazebo empty-world MoveIt launch entrypoint for Franka Panda.

Responsibility:
- Select immutable resources and the disabled Octomap profile for the empty scene.
- Delegate graph assembly to the standard Gazebo MoveIt factory.

Does not own:
- Simulator, controller, planner or perception implementation.
"""

from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription

from franka_motion_runtime.bringup.gazebo_moveit import (
    create_gazebo_moveit_description,
)
from franka_motion_runtime.bringup.launch_parts.moveit import OctomapProfile
from franka_motion_runtime.simulation.gazebo.scenarios import GazeboScenario


def generate_launch_description() -> LaunchDescription:
    """Create the canonical Gazebo empty-world MoveIt stack."""
    scenario = GazeboScenario(
        world_file="panda_empty.world.sdf",
        tmp_urdf_path=Path("/tmp/franka_panda.urdf"),
        tmp_bridge_config_path=Path("/tmp/franka_gazebo_empty_bridge.yaml"),
    )
    launch_description = create_gazebo_moveit_description(
        scenario,
        octomap_profile=OctomapProfile.DISABLED,
        start_transfer_server_default=False,
    )
    return launch_description
