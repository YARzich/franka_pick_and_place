"""
Static MoveIt planning-scene geometry for simulator table scenarios.

Responsibility:
- Define explicit tabletop collision profiles in the robot base frame.
- Create the one-shot node that applies a selected profile to MoveIt.

Does not own:
- Dynamic cube collision objects used by pick/place scenarios.
- Simulator asset parsing or generic MoveIt configuration.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from franka_motion_runtime.bringup.launch_parts.constants import (
    MANIPULATION_PACKAGE,
    SCENARIO_PACKAGE,
    STATIC_SCENE_PACKAGE,
)


@dataclass(frozen=True)
class TableCollisionProfile:
    """Axis-aligned tabletop approximation expressed in one planning frame."""

    frame_id: str
    dimensions: tuple[float, float, float]
    position: tuple[float, float, float]


# Gazebo owns the same local SDF box, so physics and MoveIt use one tabletop
# approximation. A 2 mm mounting clearance avoids contact with link0.
GAZEBO_TABLE_COLLISION = TableCollisionProfile(
    frame_id="panda_link0",
    dimensions=(1.5532, 2.3937, 0.05),
    position=(0.6669, 0.0, -0.027),
)

# table_world_usd_clean.usd uses centimetres: tabletop scale (200, 100, 10)
# with centre z=-5 cm. Franka is rooted at the tabletop origin; the same 2 mm
# mounting clearance is applied to the MoveIt approximation.
ISAAC_TABLE_COLLISION = TableCollisionProfile(
    frame_id="panda_link0",
    dimensions=(2.0, 1.0, 0.1),
    position=(0.0, 0.0, -0.052),
)


def create_table_planning_scene_node(
    *,
    use_sim_time: LaunchConfiguration | bool,
    profile: TableCollisionProfile,
) -> Node:
    """Create the one-shot coordinated table collision bootstrap node."""
    return Node(
        package=STATIC_SCENE_PACKAGE,
        executable="static_table_scene",
        name="static_table_scene",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "use_scene_coordinator": True,
                "object_id": "table",
                "frame_id": profile.frame_id,
                "dimensions": list(profile.dimensions),
                "position": list(profile.position),
            }
        ],
    )


def create_scene_coordinator_node(
    *,
    use_sim_time: LaunchConfiguration | bool,
) -> Node:
    """Create the single-writer PlanningScene ownership coordinator."""
    generic_share = Path(get_package_share_directory(MANIPULATION_PACKAGE))
    scenario_share = Path(get_package_share_directory(SCENARIO_PACKAGE))
    coordinator_node = Node(
        package=MANIPULATION_PACKAGE,
        executable="planning_scene_coordinator",
        name="planning_scene_coordinator",
        output="screen",
        parameters=[
            str(generic_share / "config" / "planning_scene_coordinator.yaml"),
            str(
                scenario_share / "config" / "manipulation" /
                "planning_scene_coordinator.yaml"
            ),
            str(
                scenario_share / "config" / "manipulation" /
                "robot_profile.yaml"
            ),
            {"use_sim_time": use_sim_time},
        ],
    )
    return coordinator_node
