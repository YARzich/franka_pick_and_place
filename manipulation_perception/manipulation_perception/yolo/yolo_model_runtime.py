"""
YOLO model loading and runtime selection helpers.

Responsibility:
- Validate explicitly supplied YOLO model files or model identifiers.
- Lazy-load Ultralytics/SAHI dependencies only when the runtime node starts.
- Build the active base/mask/SAHI model set without depending on ROS.
- Resolve class filters and SAHI device strings from plain values.

Does not own:
- ROS parameters, publishers, subscriptions or callbacks.
- OpenCV debug windows or runtime control widgets.
- Per-frame post-processing, temporal filtering or output message publication.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable


@dataclass(frozen=True)
class LoadedYoloRuntime:
    """
    Owned model handles and optional SAHI capabilities for one node.

    The bundle is immutable, but contained model objects may own mutable GPU
    state and must be invoked serially by the detector callback path.
    """

    model: Any
    model_source: str
    base_model_is_segment: bool
    mask_model: Any | None
    mask_model_available: bool
    sahi_model: Any | None
    get_sliced_prediction: Any | None
    sahi_available: bool


def load_ultralytics() -> tuple[Any, Any]:
    """Import Ultralytics model constructors with an operator-facing error."""
    try:
        from ultralytics import YOLO
        from ultralytics import YOLOWorld
    except ImportError as exc:  # pragma: no cover - dependency/runtime check
        raise ImportError(
            "Пакет ultralytics не найден. Установите его в используемое Python-окружение."
        ) from exc
    return YOLO, YOLOWorld


def load_sahi() -> tuple[Any | None, Any | None]:
    """Import optional SAHI APIs, returning unavailable capabilities as ``None``."""
    try:
        from sahi import AutoDetectionModel
        from sahi.predict import get_sliced_prediction
    except ImportError:
        return None, None
    return AutoDetectionModel, get_sliced_prediction


def resolve_model_source(model_path: str) -> str:
    """Require an explicitly provisioned local model to avoid hidden downloads."""
    if not str(model_path).strip():
        raise ValueError("YOLO model_path must be supplied by the scenario")
    model_file = Path(model_path)
    if model_file.is_file():
        return str(model_file)
    raise FileNotFoundError(
        f"YOLO модель не найдена: {model_file}. "
        "Передайте путь к заранее подготовленному локальному checkpoint."
    )


def resolve_sahi_device(device: str) -> str | None:
    """Normalize ROS device text into SAHI's CPU/MPS/CUDA convention."""
    device_clean = str(device).strip().lower()
    if not device_clean:
        return None
    if device_clean in {"cpu", "mps"}:
        return device_clean
    if device_clean.isdigit():
        return f"cuda:{device_clean}"
    if device_clean.startswith("cuda"):
        return device_clean
    return device_clean


def resolve_class_filter(model_names: Any, class_tokens: list[str]) -> list[int] | None:
    """Resolve numeric/name tokens against model classes, dropping unknown names."""
    if not class_tokens:
        return None

    name_to_id: dict[str, int] = {}
    if isinstance(model_names, dict):
        name_to_id = {str(value): int(key) for key, value in model_names.items()}
    elif isinstance(model_names, (list, tuple)):
        name_to_id = {str(value): index for index, value in enumerate(model_names)}

    class_ids: list[int] = []
    for token in class_tokens:
        if token.isdigit():
            class_ids.append(int(token))
            continue
        if token in name_to_id:
            class_ids.append(name_to_id[token])

    class_ids = sorted(set(class_ids))
    return class_ids or None


def active_yolo_model(model: Any, mask_model: Any | None, use_mask_model: bool) -> Any:
    """Select the mask model only when requested and successfully loaded."""
    if use_mask_model and mask_model is not None:
        return mask_model
    return model


def active_yolo_model_path(
    model_path: str,
    mask_model_path: str,
    mask_model: Any | None,
    use_mask_model: bool,
) -> str:
    """Return the source path corresponding to the actually selected model."""
    if use_mask_model and mask_model is not None:
        return mask_model_path
    return model_path


def create_sahi_detection_model(
    auto_detection_model: Any,
    *,
    model_source: str,
    confidence: float,
    device: str,
    image_size: int,
) -> Any:
    """Construct SAHI's Ultralytics adapter across supported API versions."""
    sahi_device = resolve_sahi_device(device)
    try:
        return auto_detection_model.from_pretrained(
            model_type="ultralytics",
            model_path=model_source,
            confidence_threshold=confidence,
            device=sahi_device,
            image_size=image_size,
        )
    except TypeError:
        return auto_detection_model.from_pretrained(
            model_type="ultralytics",
            model_path=model_source,
            confidence_threshold=confidence,
            device=sahi_device,
        )


def _warn(logger: Any | None, message: str) -> None:
    if logger is not None:
        logger.warn(message)


def load_yolo_runtime(
    *,
    model_path: str,
    mask_model_path: str,
    use_yolo_world: bool,
    open_vocab_classes: list[str],
    enable_runtime_controls: bool,
    confidence: float,
    device: str,
    imgsz: int,
    logger: Any | None = None,
    ultralytics_loader: Callable[[], tuple[Any, Any]] = load_ultralytics,
    sahi_loader: Callable[[], tuple[Any | None, Any | None]] = load_sahi,
) -> LoadedYoloRuntime:
    """
    Construct selected Ultralytics/SAHI backends without ROS ownership.

    Model objects are returned as one immutable runtime bundle. Optional
    dependencies are loaded through injected factories so CPU tests never need
    network, GPU or model downloads. Configuration errors propagate before a
    subscription or worker is created by the owning ROS node.
    """
    model_source = resolve_model_source(model_path)
    YOLO, YOLOWorld = ultralytics_loader()

    if use_yolo_world:
        model = YOLOWorld(model_source)
        if open_vocab_classes:
            model.set_classes(open_vocab_classes)
    else:
        model = YOLO(model_source)
        if open_vocab_classes:
            _warn(
                logger,
                "open_vocab_classes задан, но use_yolo_world=false; "
                "список классов будет проигнорирован.",
            )

    base_model_is_segment = getattr(model, "task", "") == "segment"
    mask_model = None
    mask_model_available = False
    if not use_yolo_world and enable_runtime_controls and mask_model_path:
        mask_model_file = Path(mask_model_path)
        if mask_model_file.is_file():
            mask_model = YOLO(str(mask_model_file))
            mask_model_available = getattr(mask_model, "task", "") == "segment"
            if not mask_model_available:
                _warn(
                    logger,
                    "mask_model_path задан, но модель не является "
                    f"segment-моделью: {mask_model_file}",
                )
        else:
            _warn(
                logger,
                "mask_model_path не найден, переключение bbox/mask отключено: "
                f"{mask_model_file}",
            )

    auto_detection_model, get_sliced_prediction = sahi_loader()
    sahi_available = auto_detection_model is not None and get_sliced_prediction is not None
    sahi_model = None
    if not use_yolo_world and not base_model_is_segment and sahi_available:
        sahi_model = create_sahi_detection_model(
            auto_detection_model,
            model_source=model_source,
            confidence=confidence,
            device=device,
            image_size=imgsz,
        )
    elif not use_yolo_world and not base_model_is_segment and not sahi_available:
        _warn(
            logger,
            "Пакет sahi не найден, использую legacy tiled inference. "
            "Рекомендуется: pip install sahi",
        )

    return LoadedYoloRuntime(
        model=model,
        model_source=model_source,
        base_model_is_segment=base_model_is_segment,
        mask_model=mask_model,
        mask_model_available=mask_model_available,
        sahi_model=sahi_model,
        get_sliced_prediction=get_sliced_prediction,
        sahi_available=sahi_available,
    )
