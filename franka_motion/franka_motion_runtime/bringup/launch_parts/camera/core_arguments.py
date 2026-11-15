"""
Core table-camera launch arguments shared by Gazebo and Isaac variants.

Responsibility:
- Declare simulator-agnostic controls, RViz/config paths and pipeline mode arguments.
- Keep common table-camera argument names shared by Gazebo and Isaac entrypoints.

Does not own:
- Perception backend tuning knobs.
- Simulator-specific process or camera spawn arguments.
"""

from __future__ import annotations

from franka_motion_runtime.bringup.launch_parts.camera.argument_utils import (
    LaunchArgumentGroup,
    add_launch_argument,
)
from franka_motion_runtime.bringup.launch_parts.camera.paths import TableCameraPaths


def create_core_arguments(
    paths: TableCameraPaths,
    *,
    headless_description: str,
    include_fastdds_argument: bool,
) -> LaunchArgumentGroup:
    """Declare simulator-independent camera and visualization controls."""
    configs = {}
    arguments = []

    add_launch_argument(
        configs,
        arguments,
        "use_sim_time",
        "true",
        "Использовать симуляционное время",
    )
    add_launch_argument(configs, arguments, "headless", "false", headless_description)
    add_launch_argument(
        configs,
        arguments,
        "start_rviz",
        "true",
        "Запускать RViz2 с облаком точек стереокамеры",
    )
    add_launch_argument(
        configs,
        arguments,
        "table_camera_params_file",
        str(paths.perception_config),
        "YAML с параметрами нод table-camera perception pipeline.",
    )
    add_launch_argument(configs, arguments, "rviz_config", str(paths.rviz_config), "Путь к RViz2 конфигу")
    if include_fastdds_argument:
        add_launch_argument(
            configs,
            arguments,
            "fastdds_builtin_transports",
            "DEFAULT",
            (
                "Значение FASTDDS_BUILTIN_TRANSPORTS для launched ROS2-процессов. "
                "DEFAULT включает UDPv4+SHM; UDPv4 можно использовать как "
                "fallback при проблемах с SHM."
            ),
        )
    add_launch_argument(
        configs,
        arguments,
        "mode",
        "gigaPose",
        (
            "Режим пайплайна: megaPose | gigaPose | foundationPose | auto | none | ecc | ess | yolo. "
            "auto является alias megaPose; mode:=yolo включает окно настройки."
        ),
    )
    return configs, arguments
