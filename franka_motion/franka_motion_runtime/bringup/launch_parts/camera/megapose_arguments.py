"""
MegaPose launch arguments for table-camera pose estimation.

Responsibility:
- Declare MegaPose runtime, mesh and inference scheduling parameters.
- Keep MegaPose defaults independent from other pose-estimator argument groups.

Does not own:
- GigaPose retrieval parameters.
- FoundationPose TensorRT settings.
"""

from __future__ import annotations

from launch.substitutions import EnvironmentVariable

from franka_motion_runtime.bringup.launch_parts.camera.argument_utils import (
    LaunchArgumentGroup,
    add_launch_argument,
)
from franka_motion_runtime.bringup.launch_parts.camera.paths import TableCameraPaths


def create_megapose_arguments(paths: TableCameraPaths) -> LaunchArgumentGroup:
    """Declare MegaPose/GigaPose worker and model controls."""
    configs = {}
    arguments = []

    add_launch_argument(
        configs,
        arguments,
        "megapose_python",
        EnvironmentVariable("FRANKA_GPU_MEGAPOSE_PYTHON"),
        (
            "Python окружения, где установлен megapose6d. Берётся из "
            "переменной окружения FRANKA_GPU_MEGAPOSE_PYTHON, если явно не "
            "передан; без неё запуск завершится ошибкой. Установка описана "
            "в franka_motion/models/README.md, раздел MegaPose."
        ),
    )
    add_launch_argument(
        configs,
        arguments,
        "megapose_data_dir",
        EnvironmentVariable("FRANKA_GPU_MEGAPOSE_DATA_DIR"),
        (
            "MEGAPOSE_DATA_DIR с загруженными моделями; runtime example "
            "создается внутри examples/. Берётся из переменной окружения "
            "FRANKA_GPU_MEGAPOSE_DATA_DIR, если явно не передан; без неё "
            "запуск завершится ошибкой. Установка описана в "
            "franka_motion/models/README.md, раздел MegaPose."
        ),
    )
    add_launch_argument(
        configs,
        arguments,
        "megapose_model",
        "auto",
        "Имя модели MegaPose. auto выбирает RGBD при наличии stereo depth, иначе RGB.",
    )
    add_launch_argument(
        configs,
        arguments,
        "megapose_mesh_file_path",
        str(paths.foundationpose_mesh),
        "Абсолютный путь к mesh объекта для mode:=megaPose.",
    )
    add_launch_argument(
        configs,
        arguments,
        "megapose_mesh_units",
        "m",
        "Единицы исходного mesh для MegaPose: m или mm. MegaPose runtime получает mm.",
    )
    add_launch_argument(
        configs,
        arguments,
        "megapose_object_label",
        "cube_5cm",
        "Label объекта в MegaPose input/output JSON.",
    )
    add_launch_argument(
        configs,
        arguments,
        "megapose_depth_mode",
        "on",
        "Использование stereo depth в MegaPose: auto | on/rgbd | icp | off.",
    )
    add_launch_argument(
        configs,
        arguments,
        "megapose_bbox_timeout_sec",
        "3.0",
        (
            "Максимальный возраст bbox для MegaPose; больше общего ROI timeout, "
            "потому что inference тяжелый."
        ),
    )
    add_launch_argument(
        configs,
        arguments,
        "megapose_run_every_n_frames",
        "30",
        "Запускать MegaPose каждый N-й RGB кадр.",
    )
    add_launch_argument(
        configs,
        arguments,
        "megapose_min_inference_interval_sec",
        "1.0",
        "Минимальный интервал между MegaPose inference запусками.",
    )
    add_launch_argument(
        configs,
        arguments,
        "megapose_process_timeout_sec",
        "120.0",
        "Timeout MegaPose worker/CLI inference.",
    )
    add_launch_argument(
        configs,
        arguments,
        "megapose_persistent_worker",
        "false",
        "Держать MegaPose модель загруженной в отдельном worker-процессе.",
    )
    add_launch_argument(
        configs,
        arguments,
        "megapose_bsz_images",
        "64",
        (
            "Batch size изображений внутри MegaPose worker; "
            "меньшее значение экономит VRAM."
        ),
    )
    add_launch_argument(
        configs,
        arguments,
        "megapose_bsz_objects",
        "8",
        "Batch size объектов внутри MegaPose worker.",
    )
    return configs, arguments
