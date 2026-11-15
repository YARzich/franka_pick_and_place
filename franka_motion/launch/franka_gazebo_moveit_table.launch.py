"""
Gazebo table-world MoveIt launch entrypoint for Franka Panda.

Responsibility:
- Select immutable table resources, collision geometry and Octomap profile.
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
from franka_motion_runtime.bringup.launch_parts.planning_scene import (
    GAZEBO_TABLE_COLLISION,
)
from franka_motion_runtime.simulation.gazebo.scenarios import GazeboScenario


TABLE_BASE_POSE = {
    "x": "0.0",
    "y": "0.0",
    "z": "1.0",
    "roll": "0.0",
    "pitch": "0.0",
    "yaw": "0.0",
}


def generate_launch_description() -> LaunchDescription:
    """Create the canonical Gazebo table MoveIt stack."""
    scenario = GazeboScenario(
        world_file="table_world.sdf",
        tmp_urdf_path=Path("/tmp/franka_panda_table.urdf"),
        tmp_bridge_config_path=Path("/tmp/franka_gazebo_table_bridge.yaml"),
        base_pose=TABLE_BASE_POSE,
    )
    launch_description = create_gazebo_moveit_description(
        scenario,
        octomap_profile=OctomapProfile.DISABLED,
        start_transfer_server_default=True,
        table_collision_profile=GAZEBO_TABLE_COLLISION,
    )
    return launch_description
