"""
ESS stereo-depth launch arguments for Isaac ROS table-camera modes.

Responsibility:
- Declare TensorRT ESS engine path and filtering threshold.
- Keep ESS-specific launch defaults separate from generic stereo settings.

Does not own:
- Generic point-cloud settings.
- Detector or pose-estimator parameters.
"""

from __future__ import annotations

from launch.substitutions import EnvironmentVariable

from franka_motion_runtime.bringup.launch_parts.camera.argument_utils import (
    LaunchArgumentGroup,
    add_launch_argument,
)
from franka_motion_runtime.bringup.launch_parts.camera.paths import TableCameraPaths


def create_ess_arguments(paths: TableCameraPaths) -> LaunchArgumentGroup:
    """Declare Isaac ROS ESS engine and threshold controls."""
    del paths
    configs = {}
    arguments = []

    add_launch_argument(
        configs,
        arguments,
        "ess_engine_file_path",
        EnvironmentVariable("FRANKA_GPU_ESS_ENGINE"),
        (
            "Абсолютный путь к TensorRT ESS engine (используется при "
            "mode:=ess, mode:=foundationPose, mode:=megaPose и mode:=gigaPose). "
            "Берётся из переменной окружения FRANKA_GPU_ESS_ENGINE, если "
            "явно не передан; без неё запуск завершится ошибкой. Получение "
            "описано в franka_motion/models/README.md, раздел ESS."
        ),
    )
    add_launch_argument(
        configs,
        arguments,
        "ess_threshold",
        "0.3",
        "Порог confidence фильтра ESS (0.0..1.0).",
    )
    return configs, arguments
