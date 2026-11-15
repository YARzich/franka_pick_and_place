"""
Resolve package resource paths used by launch building blocks.

Responsibility:
- Collect package share/prefix paths into typed scenario path containers.
- Keep Gazebo and Isaac launch entrypoints from hardcoding resource locations.

Does not own:
- Creating temporary files or validating external dependency availability.
- Runtime parameter values loaded from the resolved resources.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from franka_motion_runtime.bringup.launch_parts.constants import (
    BRINGUP_PACKAGE,
    CONTROL_PACKAGE,
    DESCRIPTION_PACKAGE,
    GAZEBO_SIM_PACKAGE,
    ISAAC_SIM_PACKAGE,
    MOVEIT_CONFIG_PACKAGE,
)
from franka_motion_runtime.simulation.gazebo.scenarios import GazeboScenario


@dataclass(frozen=True)
class GazeboLaunchPaths:
    """Canonical resources needed by one Gazebo integration leaf."""

    package_share: Path
    world: Path
    bridge_wrapper: Path
    managed_process_wrapper: Path
    robot_xacro: Path
    semantic_config: Path
    initial_positions: Path
    ros2_control_config: Path
    controller_config: Path
    gripper_config: Path
    kinematics_config: Path
    tmp_urdf: Path
    tmp_bridge_config: Path
    simulator_share: Path | None = None


@dataclass(frozen=True)
class IsaacLaunchPaths:
    """Canonical resources needed by one Isaac Sim integration leaf."""

    package_share: Path
    managed_process_wrapper: Path
    isaac_worlds: Path
    initial_positions: Path
    semantic_config: Path
    kinematics_config: Path
    ros2_control_config: Path
    gripper_config: Path
    moveit_trajectory_execution_config: Path
    panda_xacro: Path
    panda_usd: Path
    isaac_script: Path
    simulator_share: Path | None = None


def create_gazebo_launch_paths(scenario: GazeboScenario) -> GazeboLaunchPaths:
    """Resolve canonical owners for one Gazebo leaf scenario."""
    package_share = Path(get_package_share_directory(BRINGUP_PACKAGE))
    package_prefix = Path(get_package_prefix(BRINGUP_PACKAGE))
    description_share = Path(get_package_share_directory(DESCRIPTION_PACKAGE))
    simulator_share = Path(get_package_share_directory(GAZEBO_SIM_PACKAGE))
    control_config_dir = (
        Path(get_package_share_directory(CONTROL_PACKAGE)) / "config" / "control"
    )
    moveit_config_dir = (
        Path(get_package_share_directory(MOVEIT_CONFIG_PACKAGE)) / "config" / "moveit"
    )

    return GazeboLaunchPaths(
        package_share=package_share,
        world=simulator_share / "worlds" / "gazebo" / scenario.world_file,
        bridge_wrapper=(
            package_prefix / "lib" / BRINGUP_PACKAGE / "ros_gz_bridge_wrapper.py"
        ),
        managed_process_wrapper=(
            package_prefix / "lib" / BRINGUP_PACKAGE / "managed_process_wrapper.py"
        ),
        robot_xacro=(
            description_share / "description" / "urdf" / "panda_gazebo.urdf.xacro"
        ),
        semantic_config=moveit_config_dir / "panda.srdf",
        initial_positions=control_config_dir / "initial_positions.yaml",
        ros2_control_config=control_config_dir / "panda_ros2_control_gazebo.yaml",
        controller_config=control_config_dir / "panda_ros2_control_gazebo.yaml",
        gripper_config=control_config_dir / "panda_gripper_driver.yaml",
        kinematics_config=moveit_config_dir / "kinematics.yaml",
        tmp_urdf=scenario.tmp_urdf_path,
        tmp_bridge_config=scenario.tmp_bridge_config_path,
        simulator_share=simulator_share,
    )


def create_isaac_launch_paths() -> IsaacLaunchPaths:
    """Resolve canonical owners for an Isaac Sim leaf scenario."""
    package_share = Path(get_package_share_directory(BRINGUP_PACKAGE))
    package_prefix = Path(get_package_prefix(BRINGUP_PACKAGE))
    description_share = Path(get_package_share_directory(DESCRIPTION_PACKAGE))
    simulator_share = Path(get_package_share_directory(ISAAC_SIM_PACKAGE))
    control_config_dir = (
        Path(get_package_share_directory(CONTROL_PACKAGE)) / "config" / "control"
    )
    moveit_config_dir = (
        Path(get_package_share_directory(MOVEIT_CONFIG_PACKAGE)) / "config" / "moveit"
    )

    return IsaacLaunchPaths(
        package_share=package_share,
        managed_process_wrapper=(
            package_prefix / "lib" / BRINGUP_PACKAGE / "managed_process_wrapper.py"
        ),
        isaac_worlds=simulator_share / "worlds" / "isaac",
        initial_positions=control_config_dir / "initial_positions.yaml",
        semantic_config=moveit_config_dir / "panda.srdf",
        kinematics_config=moveit_config_dir / "kinematics.yaml",
        ros2_control_config=control_config_dir / "panda_ros2_control_isaac.yaml",
        gripper_config=control_config_dir / "panda_gripper_driver.yaml",
        moveit_trajectory_execution_config=(
            moveit_config_dir / "isaac_trajectory_execution.yaml"
        ),
        panda_xacro=(
            description_share / "description" / "urdf" / "panda_isaac.urdf.xacro"
        ),
        panda_usd=(
            simulator_share / "worlds" / "isaac" / "FrankaPanda" / "franka.usd"
        ),
        isaac_script=simulator_share / "scripts" / "run_franka_isaac_sim.py",
        simulator_share=simulator_share,
    )
