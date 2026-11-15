"""
Assemble public launch arguments for table-camera perception scenarios.

Responsibility: Keep the Gazebo/Isaac table-camera launch argument API stable and
merge backend-specific argument groups in the order expected by launch files.

Does not own:
- Individual backend defaults; those live in the matching ``*_arguments.py`` modules.
- Runtime validation or perception node construction.
"""

from __future__ import annotations

from collections.abc import Callable

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from franka_motion_runtime.bringup.launch_parts.camera.argument_utils import LaunchArgumentGroup
from franka_motion_runtime.bringup.launch_parts.camera.core_arguments import create_core_arguments
from franka_motion_runtime.bringup.launch_parts.camera.detection_arguments import create_detection_arguments
from franka_motion_runtime.bringup.launch_parts.camera.ess_arguments import create_ess_arguments
from franka_motion_runtime.bringup.launch_parts.camera.foundationpose_arguments import create_foundationpose_arguments
from franka_motion_runtime.bringup.launch_parts.camera.gigapose_arguments import create_gigapose_arguments
from franka_motion_runtime.bringup.launch_parts.camera.megapose_arguments import create_megapose_arguments
from franka_motion_runtime.bringup.launch_parts.camera.paths import TableCameraPaths
from franka_motion_runtime.bringup.launch_parts.camera.stereo_arguments import create_stereo_arguments
from franka_motion_runtime.bringup.launch_parts.camera.yolo_arguments import create_yolo_arguments

LaunchArgumentFactory = Callable[[TableCameraPaths], LaunchArgumentGroup]


_BACKEND_ARGUMENT_GROUPS: tuple[LaunchArgumentFactory, ...] = (
    create_stereo_arguments,
    create_detection_arguments,
    create_yolo_arguments,
    create_megapose_arguments,
    create_gigapose_arguments,
    create_ess_arguments,
    create_foundationpose_arguments,
)


def _merge_argument_group(
    configs: dict[str, LaunchConfiguration],
    arguments: list[DeclareLaunchArgument],
    group: LaunchArgumentGroup,
) -> None:
    group_configs, group_arguments = group
    configs.update(group_configs)
    arguments.extend(group_arguments)


def _create_table_camera_launch_arguments(
    paths: TableCameraPaths,
    *,
    headless_description: str,
    include_fastdds_argument: bool,
) -> tuple[dict[str, LaunchConfiguration], list[DeclareLaunchArgument]]:
    configs, arguments = create_core_arguments(
        paths,
        headless_description=headless_description,
        include_fastdds_argument=include_fastdds_argument,
    )

    for create_group in _BACKEND_ARGUMENT_GROUPS:
        _merge_argument_group(configs, arguments, create_group(paths))

    return configs, arguments


def create_gazebo_table_camera_launch_arguments(
    paths: TableCameraPaths,
) -> tuple[dict[str, LaunchConfiguration], list[DeclareLaunchArgument]]:
    """Declare the complete Gazebo table-camera public argument surface."""
    return _create_table_camera_launch_arguments(
        paths,
        headless_description="Запуск Gazebo без GUI (gz sim -s)",
        include_fastdds_argument=True,
    )


def create_isaac_table_camera_launch_arguments(
    paths: TableCameraPaths,
) -> tuple[dict[str, LaunchConfiguration], list[DeclareLaunchArgument]]:
    """Declare the complete Isaac table-camera public argument surface."""
    return _create_table_camera_launch_arguments(
        paths,
        headless_description="Запуск Isaac Sim без GUI",
        include_fastdds_argument=False,
    )
