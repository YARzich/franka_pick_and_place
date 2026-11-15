"""
FoundationPose launch arguments for table-camera pose estimation.

Responsibility:
- Declare FoundationPose ONNX/TensorRT model paths.
- Declare object mesh and texture resources used by the FoundationPose backend.

Does not own:
- Stereo-depth settings.
- Detector or MegaPose/GigaPose parameters.
"""

from __future__ import annotations

from launch.substitutions import EnvironmentVariable

from franka_motion_runtime.bringup.launch_parts.camera.argument_utils import (
    LaunchArgumentGroup,
    add_launch_argument,
)
from franka_motion_runtime.bringup.launch_parts.camera.paths import TableCameraPaths


def create_foundationpose_arguments(paths: TableCameraPaths) -> LaunchArgumentGroup:
    """Declare FoundationPose model engine and mesh controls."""
    configs = {}
    arguments = []

    add_launch_argument(
        configs,
        arguments,
        "foundationpose_refine_model_file_path",
        EnvironmentVariable("FRANKA_GPU_FOUNDATIONPOSE_REFINE_MODEL"),
        (
            "Абсолютный путь к ONNX refine model для mode:=foundationPose. "
            "Берётся из переменной окружения "
            "FRANKA_GPU_FOUNDATIONPOSE_REFINE_MODEL, если явно не передан; "
            "без неё запуск завершится ошибкой. Получение описано в "
            "franka_motion/models/README.md, раздел FoundationPose."
        ),
    )
    add_launch_argument(
        configs,
        arguments,
        "foundationpose_refine_engine_file_path",
        EnvironmentVariable("FRANKA_GPU_FOUNDATIONPOSE_REFINE_ENGINE"),
        (
            "Абсолютный путь к TensorRT refine engine для "
            "mode:=foundationPose. Берётся из переменной окружения "
            "FRANKA_GPU_FOUNDATIONPOSE_REFINE_ENGINE, если явно не передан; "
            "без неё запуск завершится ошибкой. Получение описано в "
            "franka_motion/models/README.md, раздел FoundationPose."
        ),
    )
    add_launch_argument(
        configs,
        arguments,
        "foundationpose_score_model_file_path",
        EnvironmentVariable("FRANKA_GPU_FOUNDATIONPOSE_SCORE_MODEL"),
        (
            "Абсолютный путь к ONNX score model для mode:=foundationPose. "
            "Берётся из переменной окружения "
            "FRANKA_GPU_FOUNDATIONPOSE_SCORE_MODEL, если явно не передан; "
            "без неё запуск завершится ошибкой. Получение описано в "
            "franka_motion/models/README.md, раздел FoundationPose."
        ),
    )
    add_launch_argument(
        configs,
        arguments,
        "foundationpose_score_engine_file_path",
        EnvironmentVariable("FRANKA_GPU_FOUNDATIONPOSE_SCORE_ENGINE"),
        (
            "Абсолютный путь к TensorRT score engine для "
            "mode:=foundationPose. Берётся из переменной окружения "
            "FRANKA_GPU_FOUNDATIONPOSE_SCORE_ENGINE, если явно не передан; "
            "без неё запуск завершится ошибкой. Получение описано в "
            "franka_motion/models/README.md, раздел FoundationPose."
        ),
    )
    add_launch_argument(
        configs,
        arguments,
        "foundationpose_mesh_file_path",
        str(paths.foundationpose_mesh),
        "Абсолютный путь к OBJ-модели куба для mode:=foundationPose.",
    )
    add_launch_argument(
        configs,
        arguments,
        "foundationpose_texture_path",
        str(paths.foundationpose_texture),
        "Абсолютный путь к текстуре куба для mode:=foundationPose.",
    )
    return configs, arguments
