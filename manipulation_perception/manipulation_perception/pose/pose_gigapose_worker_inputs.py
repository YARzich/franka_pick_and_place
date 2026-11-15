"""
Input helpers for the GigaPose coarse JSON worker.

Responsibility:
- Normalize prewarm and inference JSON requests into typed objects.
- Validate GigaPose coarse runtime file layout.
- Load RGB/mask inputs and build the masked retrieval batch.

Does not own:
- Loading the GigaPose model or Hydra config.
- Reading prediction files or serializing worker responses.
- stdin/stdout worker protocol.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import cv2
import numpy as np


@dataclass(frozen=True)
class GigaPoseBackendConfig:
    """Model and dataset identity shared by prewarm and inference requests."""

    repo_dir: Path
    root_dir: Path
    dataset_name: str
    object_id: int
    checkpoint_path: Path
    dinov2_repo_dir: Path | None
    template_count: int
    translation_scale: float
    max_num_dets_per_forward: int

    @property
    def log_dir(self) -> Path:
        """Return the configured dataset's process-local prediction directory."""
        return self.root_dir / "ros_runtime" / self.dataset_name


@dataclass(frozen=True)
class GigaPoseWorkerRequest(GigaPoseBackendConfig):
    """
    Validated input and model identity for one coarse retrieval request.

    ``bbox`` uses half-open image-pixel bounds. ``camera_matrix`` maps metric
    camera coordinates to pixels, while ``translation_scale`` converts the
    prediction artifact into metres before it leaves the worker.
    """

    request_id: int
    image_path: Path
    mask_path: Path | None
    bbox: tuple[int, int, int, int]
    camera_matrix: np.ndarray
    num_hypotheses: int


def parse_gigapose_backend_config(request: Mapping[str, Any]) -> GigaPoseBackendConfig:
    """Parse the model identity accepted by prewarm and inference requests."""
    dinov2_repo_dir = str(request.get("dinov2_repo_dir", "")).strip()
    return GigaPoseBackendConfig(
        repo_dir=Path(str(request["repo_dir"])).expanduser(),
        root_dir=Path(str(request["root_dir"])).expanduser(),
        dataset_name=str(request["dataset_name"]),
        object_id=int(request["object_id"]),
        checkpoint_path=Path(str(request["checkpoint_path"])).expanduser(),
        dinov2_repo_dir=Path(dinov2_repo_dir).expanduser() if dinov2_repo_dir else None,
        template_count=max(1, int(request.get("template_count", 162))),
        translation_scale=float(request.get("translation_scale", 0.001)),
        max_num_dets_per_forward=max(1, int(request.get("max_num_dets_per_forward", 4))),
    )


def parse_gigapose_worker_request(request: Mapping[str, Any]) -> GigaPoseWorkerRequest:
    """Parse a JSON-compatible inference request and its model identity."""
    config = parse_gigapose_backend_config(request)
    mask_path_raw = str(request.get("mask_path", "")).strip()
    return GigaPoseWorkerRequest(
        repo_dir=config.repo_dir,
        root_dir=config.root_dir,
        dataset_name=config.dataset_name,
        object_id=config.object_id,
        checkpoint_path=config.checkpoint_path,
        dinov2_repo_dir=config.dinov2_repo_dir,
        template_count=config.template_count,
        translation_scale=config.translation_scale,
        max_num_dets_per_forward=config.max_num_dets_per_forward,
        request_id=int(request["id"]),
        image_path=Path(str(request["image_path"])).expanduser(),
        mask_path=Path(mask_path_raw).expanduser() if mask_path_raw else None,
        bbox=tuple(int(value) for value in request["bbox"]),
        camera_matrix=np.asarray(request["K"], dtype=np.float32).reshape(3, 3),
        num_hypotheses=max(1, int(request.get("num_hypotheses", 5))),
    )


def gigapose_backend_cache_key(request: GigaPoseBackendConfig) -> tuple[str, ...]:
    """Return fields whose change requires a separately loaded CUDA model."""
    return (
        str(request.repo_dir),
        str(request.root_dir),
        request.dataset_name,
        str(request.object_id),
        str(request.checkpoint_path),
        str(request.dinov2_repo_dir or ""),
        str(request.template_count),
        str(request.max_num_dets_per_forward),
        str(request.translation_scale),
    )


def validate_gigapose_runtime_files(request: GigaPoseBackendConfig) -> None:
    """Fail before model loading when repositories, templates or weights are absent."""
    if not (request.repo_dir / "configs" / "test.yaml").is_file():
        raise FileNotFoundError(f"GigaPose repo не найден или неполный: {request.repo_dir}")
    if not request.checkpoint_path.is_file():
        raise FileNotFoundError(f"GigaPose checkpoint не найден: {request.checkpoint_path}")
    if (
        request.dinov2_repo_dir is not None
        and not (request.dinov2_repo_dir / "hubconf.py").is_file()
    ):
        raise FileNotFoundError(f"DINOv2 repo не найден или неполный: {request.dinov2_repo_dir}")

    template_root = request.root_dir / "datasets" / "templates" / request.dataset_name
    template_dir = template_root / f"{request.object_id:06d}"
    pose_path = template_root / "object_poses" / f"{request.object_id:06d}.npy"
    models_info_path = (
        request.root_dir
        / "datasets"
        / request.dataset_name
        / "models"
        / "models_info.json"
    )
    if not template_dir.is_dir() or not pose_path.is_file():
        raise FileNotFoundError(
            "GigaPose templates не найдены: "
            f"ожидались {template_dir} и {pose_path}"
        )
    if not models_info_path.is_file():
        raise FileNotFoundError(f"GigaPose models_info.json не найден: {models_info_path}")


def load_gigapose_rgb(image_path: Path) -> np.ndarray:
    """Load one RGB uint8 runtime image or raise for a missing file."""
    bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise FileNotFoundError(f"Не удалось прочитать RGB image: {image_path}")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def load_gigapose_mask(
    mask_path: Path | None,
    target_shape: tuple[int, int],
) -> np.ndarray | None:
    """Load a binary mask at image resolution; missing masks remain optional."""
    if mask_path is None:
        return None
    mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
    if mask is None:
        return None
    if mask.shape[:2] != target_shape:
        mask = cv2.resize(
            mask,
            (target_shape[1], target_shape[0]),
            interpolation=cv2.INTER_NEAREST,
        )
    return (mask > 0).astype(np.float32)


def clamp_gigapose_bbox(
    bbox: tuple[int, int, int, int],
    width: int,
    height: int,
) -> tuple[int, int, int, int]:
    """Clamp integer half-open crop bounds while preserving image boundaries."""
    if width <= 0 or height <= 0:
        raise ValueError("GigaPose image dimensions must be positive")
    x1, y1, x2, y2 = bbox
    x1 = max(0, min(width - 1, x1))
    y1 = max(0, min(height - 1, y1))
    x2 = max(x1 + 1, min(width, x2))
    y2 = max(y1 + 1, min(height, y2))
    return x1, y1, x2, y2


def build_gigapose_batch(
    *,
    rgb: np.ndarray,
    mask: np.ndarray | None,
    bbox: tuple[int, int, int, int],
    camera_matrix: np.ndarray,
    crop_transform: Any,
    normalize: Any,
    torch_module: Any,
    pandas_module: Any,
    tensor_collection_module: Any,
    object_id: int,
) -> Any:
    """
    Build the masked, cropped CUDA retrieval batch for one object.

    An absent or empty mask falls back to the half-open detector bbox. The
    returned collection contains normalized RGB, mask, camera intrinsics and
    crop transform metadata expected by GigaPose.
    """
    height, width = rgb.shape[:2]
    x1, y1, x2, y2 = clamp_gigapose_bbox(bbox, width, height)
    image = torch_module.from_numpy(rgb.astype(np.float32) / 255.0).permute(2, 0, 1)

    if mask is None:
        mask_array = np.zeros((height, width), dtype=np.float32)
        mask_array[y1:y2, x1:x2] = 1.0
    else:
        mask_array = np.asarray(mask, dtype=np.float32)
        if mask_array.shape[:2] != (height, width):
            mask_array = cv2.resize(mask_array, (width, height), interpolation=cv2.INTER_NEAREST)
        if not np.any(mask_array > 0.5):
            mask_array[y1:y2, x1:x2] = 1.0

    mask_tensor = torch_module.from_numpy((mask_array > 0.5).astype(np.float32))
    masked_rgb = image * mask_tensor.unsqueeze(0)
    rgba = torch_module.cat([masked_rgb, mask_tensor.unsqueeze(0)], dim=0).unsqueeze(0)

    boxes = torch_module.tensor([[x1, y1, x2, y2]], dtype=torch_module.long)
    cropped = crop_transform(boxes, images=rgba)
    target_img = normalize(cropped["images"][:, :3])
    target_mask = cropped["images"][:, -1]
    target_k = torch_module.from_numpy(camera_matrix).float().unsqueeze(0)

    infos = pandas_module.DataFrame(
        {
            "label": [str(object_id)],
            "scene_id": [1],
            "view_id": [1],
        }
    )
    test_list = tensor_collection_module.PandasTensorCollection(
        infos=pandas_module.DataFrame(
            {
                "im_id": [1],
                "scene_id": [1],
                "obj_id": [object_id],
                "inst_count": [1],
                "detection_time": [0.0],
            }
        )
    )
    batch = tensor_collection_module.PandasTensorCollection(
        tar_img=target_img,
        tar_mask=target_mask,
        tar_K=target_k,
        tar_M=cropped["M"].float(),
        infos=infos,
        test_list=test_list,
    )
    return batch.cuda()
