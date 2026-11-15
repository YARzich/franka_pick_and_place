"""
Common launch argument declarations shared by scenario families.

Responsibility:
- Declare simulator-wide arguments reused by Gazebo and Isaac launch entrypoints.
- Keep shared launch argument names and descriptions consistent.

Does not own:
- Table-camera perception backend arguments.
- Scenario-specific launch defaults or node wiring.
"""

from __future__ import annotations

import os
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable


def isaac_python_default() -> EnvironmentVariable:
    """Use an explicit interpreter, retaining the binary-install root override."""
    binary_root = os.environ.get("ISAAC_SIM_ROOT", "").strip()
    fallback = str(Path(binary_root).expanduser() / "python.sh") if binary_root else "python3"
    return EnvironmentVariable("FRANKA_ISAAC_PYTHON", default_value=fallback)


def declare_use_sim_time_argument(description: str) -> DeclareLaunchArgument:
    """Declare the shared simulation-clock policy."""
    return DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description=description,
    )


def declare_headless_argument(description: str) -> DeclareLaunchArgument:
    """Declare the shared simulator GUI switch."""
    return DeclareLaunchArgument(
        "headless",
        default_value="false",
        description=description,
    )


def declare_start_servo_argument() -> DeclareLaunchArgument:
    """Declare the opt-in MoveIt Servo backend switch."""
    return DeclareLaunchArgument(
        "start_servo",
        default_value="false",
        choices=["true", "false"],
        description=(
            "Запускать MoveIt Servo и реактивное действие ServoToPose."
        ),
    )


def declare_common_isaac_arguments(
    *,
    include_isaac_experience: bool = False,
) -> list[DeclareLaunchArgument]:
    """Declare simulator-wide plus shared Isaac runtime arguments."""
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description=(
            "Использовать симуляционное время (ожидается топик /clock)."
        ),
    )
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Запуск Isaac Sim без GUI.",
    )
    runtime_arguments = declare_isaac_runtime_arguments(
        include_isaac_experience=include_isaac_experience,
    )
    return [
        use_sim_time_arg,
        headless_arg,
        *runtime_arguments,
    ]


def declare_isaac_experience_argument() -> DeclareLaunchArgument:
    """Declare the optional Isaac Kit experience override."""
    return DeclareLaunchArgument(
        "isaac_experience",
        default_value="",
        description=(
            "Путь к .kit experience файлу "
            "(пусто = использовать дефолт из run_franka_isaac_sim.py)."
        ),
    )


def declare_isaac_runtime_arguments(
    *,
    include_isaac_experience: bool,
    include_gxf_log_level: bool = False,
) -> list[DeclareLaunchArgument]:
    """Declare control/runtime arguments shared by all Isaac scenarios."""
    fix_base_argument = DeclareLaunchArgument(
        "fix_base",
        default_value="true",
        description=(
            "Зафиксировать основание манипулятора при импорте URDF."
        ),
    )
    isaac_python_argument = DeclareLaunchArgument(
        "isaac_python",
        default_value=isaac_python_default(),
        description=(
            "Python с Isaac Sim; default: FRANKA_ISAAC_PYTHON или python3."
        ),
    )
    isaac_robot_prim_argument = DeclareLaunchArgument(
        "isaac_robot_prim",
        default_value="/World/Franka",
        description="Путь прима, куда импортируется робот Franka.",
    )
    time_steps_argument = DeclareLaunchArgument(
        "time_steps",
        default_value="0",
        description="Число шагов симуляции (0 = без ограничений).",
    )
    wait_for_clock_argument = DeclareLaunchArgument(
        "wait_for_clock",
        default_value="true",
        description="Ожидать /clock перед запуском контроллеров.",
    )
    ros_domain_id_argument = DeclareLaunchArgument(
        "ros_domain_id",
        default_value="0",
        description="ROS_DOMAIN_ID для всех узлов запуска.",
    )
    rmw_implementation_argument = DeclareLaunchArgument(
        "rmw_implementation",
        default_value="rmw_fastrtps_cpp",
        description="Используемая реализация RMW.",
    )
    runtime_arguments = [
        fix_base_argument,
        isaac_python_argument,
    ]
    if include_isaac_experience:
        runtime_arguments.append(declare_isaac_experience_argument())
    runtime_arguments.extend(
        [
            isaac_robot_prim_argument,
            time_steps_argument,
            wait_for_clock_argument,
            ros_domain_id_argument,
            rmw_implementation_argument,
        ]
    )
    if include_gxf_log_level:
        gxf_log_level_argument = DeclareLaunchArgument(
            "gxf_log_level",
            default_value="ERROR",
            description="Уровень логов GXF/NITROS.",
        )
        runtime_arguments.append(gxf_log_level_argument)
    return runtime_arguments
