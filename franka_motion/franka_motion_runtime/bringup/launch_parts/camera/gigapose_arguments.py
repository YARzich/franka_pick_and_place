"""
GigaPose launch arguments for table-camera coarse pose retrieval.

Responsibility: Declare GigaPose repository/data paths, retrieval parameters and
MegaPose refiner knobs used by ``mode:=gigaPose``.

Does not own:
- Generic MegaPose-only runtime settings.
- Detector or stereo-depth parameters.
"""

from __future__ import annotations

from launch.substitutions import EnvironmentVariable

from franka_motion_runtime.bringup.launch_parts.camera.argument_utils import (
    LaunchArgumentGroup,
    add_launch_argument,
)
from franka_motion_runtime.bringup.launch_parts.camera.paths import TableCameraPaths


def create_gigapose_arguments(paths: TableCameraPaths) -> LaunchArgumentGroup:
    """Declare GigaPose data, checkpoint and refiner controls."""
    del paths
    configs = {}
    arguments = []

    add_launch_argument(
        configs,
        arguments,
        "gigapose_python",
        EnvironmentVariable("FRANKA_GPU_GIGAPOSE_PYTHON"),
        (
            "Python окружения, где установлен GigaPose. Берётся из переменной "
            "окружения FRANKA_GPU_GIGAPOSE_PYTHON, если явно не передан; без "
            "неё запуск завершится ошибкой. Установка описана в "
            "franka_motion/models/README.md, раздел GigaPose."
        ),
    )
    add_launch_argument(
        configs,
        arguments,
        "gigapose_repo_dir",
        EnvironmentVariable("FRANKA_GPU_GIGAPOSE_REPO_DIR"),
        (
            "Путь к checkout GigaPose. Берётся из переменной окружения "
            "FRANKA_GPU_GIGAPOSE_REPO_DIR, если явно не передан; без неё "
            "запуск завершится ошибкой. Установка описана в "
            "franka_motion/models/README.md, раздел GigaPose."
        ),
    )
    add_launch_argument(
        configs,
        arguments,
        "gigapose_dinov2_repo_dir",
        EnvironmentVariable("FRANKA_GPU_DINOV2_REPO_DIR"),
        (
            "Локальный checkout DINOv2 для GigaPose; явное пустое значение "
            "допустимо, если DINOv2 не нужен. Берётся из переменной "
            "окружения FRANKA_GPU_DINOV2_REPO_DIR (можно экспортировать "
            "пустой строкой), если явно не передан; без неё запуск "
            "завершится ошибкой. Установка описана в "
            "franka_motion/models/README.md, раздел GigaPose."
        ),
    )
    add_launch_argument(
        configs,
        arguments,
        "gigapose_data_dir",
        EnvironmentVariable("FRANKA_GPU_GIGAPOSE_DATA_DIR"),
        (
            "Корень данных GigaPose с templates/checkpoints. Берётся из "
            "переменной окружения FRANKA_GPU_GIGAPOSE_DATA_DIR, если явно не "
            "передан; без неё запуск завершится ошибкой. Установка описана "
            "в franka_motion/models/README.md, раздел GigaPose."
        ),
    )
    add_launch_argument(
        configs,
        arguments,
        "gigapose_checkpoint_path",
        "",
        "Путь к gigaPose_v1.ckpt; пусто означает <gigapose_data_dir>/pretrained/gigaPose_v1.ckpt.",
    )
    add_launch_argument(
        configs,
        arguments,
        "gigapose_dataset_name",
        "franka_cube",
        "Имя BOP-подобного dataset для GigaPose templates.",
    )
    add_launch_argument(
        configs,
        arguments,
        "gigapose_object_id",
        "1",
        "BOP object id для coarse-stage GigaPose.",
    )
    add_launch_argument(
        configs,
        arguments,
        "gigapose_template_count",
        "162",
        "Количество template viewpoints для GigaPose.",
    )
    add_launch_argument(
        configs,
        arguments,
        "gigapose_max_num_dets_per_forward",
        "4",
        "Сколько ROI одновременно подавать в GigaPose retrieval forward.",
    )
    add_launch_argument(
        configs,
        arguments,
        "gigapose_translation_scale",
        "0.001",
        "Масштаб перевода GigaPose в метры; для BOP-моделей в мм обычно 0.001.",
    )
    add_launch_argument(
        configs,
        arguments,
        "gigapose_num_hypotheses",
        "5",
        "Сколько coarse-гипотез передавать из GigaPose в MegaPose refiner.",
    )
    add_launch_argument(
        configs,
        arguments,
        "gigapose_min_matching_score",
        "0.10",
        (
            "Минимальный лучший coarse matching score; "
            "кадр ниже порога не передаётся в refiner."
        ),
    )
    add_launch_argument(
        configs,
        arguments,
        "gigapose_persistent_worker",
        "true",
        (
            "Держать GigaPose retrieval-модель загруженной "
            "в отдельном worker-процессе."
        ),
    )
    add_launch_argument(
        configs,
        arguments,
        "gigapose_refiner_bsz_images",
        "512",
        "Batch size official MegaPose scorer/refiner внутри mode:=gigaPose.",
    )
    add_launch_argument(
        configs,
        arguments,
        "gigapose_refiner_bsz_objects",
        "8",
        "Сколько GigaPose coarse-гипотез refinement обрабатывает параллельно.",
    )
    add_launch_argument(
        configs,
        arguments,
        "gigapose_refiner_iterations",
        "5",
        "Количество итераций official MegaPose refiner для mode:=gigaPose.",
    )
    return configs, arguments
