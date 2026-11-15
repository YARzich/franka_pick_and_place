"""
Gazebo process and entity-spawn action factories.

Responsibility:
- Create headless or GUI Gazebo process actions for a selected world.
- Create the ``ros_gz_sim`` entity-spawn action for a generated robot URDF.

Does not own:
- Gazebo environment variables, bridge configuration, or startup ordering.
"""

from __future__ import annotations

from pathlib import Path

from launch.actions import ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_NAME = "panda"


def create_gazebo_processes(
    *,
    world_path: Path,
    headless: LaunchConfiguration,
) -> tuple[ExecuteProcess, ExecuteProcess]:
    """Create conditioned headless and GUI processes for one Gazebo world."""
    gz_sim_process_headless = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", str(world_path)],
        output="screen",
        condition=IfCondition(headless),
    )
    gz_sim_process_gui = ExecuteProcess(
        cmd=["gz", "sim", "-r", str(world_path)],
        output="screen",
        condition=UnlessCondition(headless),
    )
    return gz_sim_process_headless, gz_sim_process_gui


def create_spawn_robot_node(tmp_urdf_path: Path, *, robot_name: str = ROBOT_NAME) -> Node:
    """Create an entity-spawn action for the generated robot URDF."""
    return Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file",
            str(tmp_urdf_path),
            "-name",
            robot_name,
            "-allow_renaming",
            "true",
        ],
        output="screen",
    )
