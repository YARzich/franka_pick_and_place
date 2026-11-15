"""
YOLO launch arguments for table-camera detector backends.

Responsibility: Declare YOLO runtime, model, SAHI and temporal filtering
parameters.

Does not own:
- Stereo-depth parameters.
- Pose-estimator backend parameters.
"""

from __future__ import annotations

from launch.substitutions import EnvironmentVariable

from franka_motion_runtime.bringup.launch_parts.camera.argument_utils import (
    LaunchArgumentGroup,
    add_launch_argument,
)
from franka_motion_runtime.bringup.launch_parts.camera.paths import TableCameraPaths


def create_yolo_arguments(paths: TableCameraPaths) -> LaunchArgumentGroup:
    """Declare the stable YOLO runtime and post-processing controls."""
    configs = {}
    arguments = []

    add_launch_argument(
        configs,
        arguments,
        "yolo_python",
        EnvironmentVariable("FRANKA_GPU_YOLO_PYTHON"),
        (
            "Python интерпретатор окружения, где установлен ultralytics. "
            "Берётся из переменной окружения FRANKA_GPU_YOLO_PYTHON, если "
            "явно не передан; без неё запуск завершится ошибкой. Установка "
            "описана в franka_motion/models/README.md, раздел YOLO runtime."
        ),
    )
    add_launch_argument(
        configs,
        arguments,
        "yolo_model_path",
        str(paths.yolo_model),
        "Путь к runtime YOLO segment модели кубика.",
    )
    add_launch_argument(
        configs,
        arguments,
        "yolo_mask_model_path",
        str(paths.yolo_model),
        "Путь к YOLO segment модели кубика для переключателя bbox/mask в mode:=yolo.",
    )
    add_launch_argument(
        configs,
        arguments,
        "yolo_confidence",
        "0.35",
        "Порог confidence для YOLO (0.0..1.0).",
    )
    add_launch_argument(configs, arguments, "yolo_iou", "0.01", "NMS IoU threshold для YOLO.")
    add_launch_argument(
        configs,
        arguments,
        "yolo_max_det",
        "30",
        "Максимальное число детекций YOLO на кадр.",
    )
    add_launch_argument(configs, arguments, "yolo_device", "0", "Устройство YOLO: 0 (CUDA) или cpu.")
    add_launch_argument(
        configs,
        arguments,
        "yolo_use_world",
        "false",
        "Использовать open-vocabulary модель YOLO-World.",
    )
    add_launch_argument(
        configs,
        arguments,
        "yolo_open_vocab_classes",
        "",
        (
            "Список классов для YOLO-World через запятую "
            "(например cube,block,dice,box)."
        ),
    )
    add_launch_argument(configs, arguments, "yolo_imgsz", "640", "Размер входа YOLO.")
    add_launch_argument(
        configs,
        arguments,
        "yolo_run_every_n_frames",
        "1",
        "Обрабатывать каждый N-й кадр в режиме YOLO.",
    )
    add_launch_argument(configs, arguments, "yolo_agnostic_nms", "false", "Class-agnostic NMS для YOLO.")
    add_launch_argument(
        configs,
        arguments,
        "yolo_augment",
        "false",
        "Test-time augmentation для YOLO (медленнее).",
    )
    add_launch_argument(
        configs,
        arguments,
        "yolo_class_whitelist",
        "cube",
        "Список разрешённых классов YOLO через запятую (имена или id).",
    )
    add_launch_argument(
        configs, arguments, "yolo_sahi_slice_width", "224",
        "Ширина тайла SAHI (пиксели)."
    )
    add_launch_argument(
        configs, arguments, "yolo_sahi_slice_height", "256",
        "Высота тайла SAHI (пиксели)."
    )
    add_launch_argument(
        configs, arguments, "yolo_sahi_overlap_ratio", "0.20",
        "Перекрытие тайлов SAHI (0.0..0.8)."
    )
    add_launch_argument(
        configs,
        arguments,
        "yolo_min_box_area_ratio",
        "0.0001",
        "Минимальная доля площади bbox от кадра (0.0..1.0).",
    )
    add_launch_argument(
        configs,
        arguments,
        "yolo_max_box_area_ratio",
        "0.35",
        "Максимальная доля площади bbox от кадра (0.0..1.0).",
    )
    add_launch_argument(
        configs,
        arguments,
        "yolo_temporal_confirm_frames",
        "1",
        "Число подряд кадров для подтверждения детекции.",
    )
    add_launch_argument(
        configs,
        arguments,
        "yolo_temporal_iou_threshold",
        "0.50",
        "IoU порог сопоставления bbox между кадрами (0.05..0.95).",
    )
    return configs, arguments
