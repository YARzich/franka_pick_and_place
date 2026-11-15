"""
Input helpers for the GigaPose refiner JSON worker.

Responsibility:
- Normalize refiner prewarm and inference requests into typed objects.
- Convert coarse estimates into the tensor collection expected by MegaPose.

Does not own:
- Loading refiner/scoring models.
- Mesh preparation for the runtime BOP object dataset.
- Final object_data serialization.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import cv2
import numpy as np


@dataclass(frozen=True)
class GigaPoseRefinerBackendConfig:
    """Model and object identity shared by prewarm and inference requests."""

    repo_dir: Path
    root_dir: Path
    dataset_name: str
    object_id: int
    megapose_models_root: Path
    bsz_images: int
    bsz_objects: int
    n_iterations: int
    num_workers: int

    @property
    def refiner_models_dir(self) -> Path:
        """Return the request-specific BOP model directory prepared at runtime."""
        return self.root_dir / "ros_runtime" / "refiner_models" / self.dataset_name


@dataclass(frozen=True)
class GigaPoseRefinerWorkerRequest(GigaPoseRefinerBackendConfig):
    """
    Validated file, camera and coarse-pose input for one refiner request.

    ``camera_matrix`` maps metric camera coordinates to image pixels. Coarse
    transforms are 4x4 object-to-camera hypotheses supplied by GigaPose.
    """

    output_label: str
    request_id: int
    image_path: Path
    camera_matrix: np.ndarray
    coarse_estimates: Mapping[str, Any]


def parse_gigapose_refiner_backend_config(
    request: Mapping[str, Any],
) -> GigaPoseRefinerBackendConfig:
    """Parse the refiner identity accepted by prewarm and inference requests."""
    return GigaPoseRefinerBackendConfig(
        repo_dir=Path(str(request["repo_dir"])).expanduser(),
        root_dir=Path(str(request["root_dir"])).expanduser(),
        dataset_name=str(request["dataset_name"]),
        object_id=int(request["object_id"]),
        megapose_models_root=Path(str(request["megapose_models_root"])).expanduser(),
        bsz_images=max(1, int(request.get("bsz_images", 512))),
        bsz_objects=max(1, int(request.get("bsz_objects", 8))),
        n_iterations=max(1, int(request.get("n_iterations", 5))),
        num_workers=max(1, int(request.get("num_workers", 1))),
    )


def parse_gigapose_refiner_worker_request(
    request: Mapping[str, Any],
) -> GigaPoseRefinerWorkerRequest:
    """Parse one inference request and its shared refiner identity."""
    config = parse_gigapose_refiner_backend_config(request)
    return GigaPoseRefinerWorkerRequest(
        repo_dir=config.repo_dir,
        root_dir=config.root_dir,
        dataset_name=config.dataset_name,
        object_id=config.object_id,
        megapose_models_root=config.megapose_models_root,
        bsz_images=config.bsz_images,
        bsz_objects=config.bsz_objects,
        n_iterations=config.n_iterations,
        num_workers=config.num_workers,
        output_label=str(request.get("output_label") or f"obj_{config.object_id:06d}"),
        request_id=int(request["id"]),
        image_path=Path(str(request["image_path"])).expanduser(),
        camera_matrix=np.asarray(request["K"], dtype=np.float32).reshape(3, 3),
        coarse_estimates=request.get("coarse_estimates") or {},
    )


def gigapose_refiner_backend_cache_key(
    request: GigaPoseRefinerBackendConfig,
) -> tuple[str, ...]:
    """Return fields whose change requires a separately loaded GPU backend."""
    return (
        str(request.repo_dir),
        str(request.root_dir),
        request.dataset_name,
        str(request.object_id),
        str(request.megapose_models_root),
        str(request.bsz_images),
        str(request.bsz_objects),
        str(request.n_iterations),
        str(request.num_workers),
    )


def load_refiner_rgb(image_path: Path) -> np.ndarray:
    """Load an RGB uint8 image or raise when the runtime file is unavailable."""
    bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise FileNotFoundError(f"Не удалось прочитать RGB image: {image_path}")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def make_gigapose_refiner_pose_estimates(
    coarse_estimates: Mapping[str, Any],
    *,
    object_id: int,
    torch_module: Any,
    pandas_module: Any,
    tensor_collection_type: Any,
) -> Any:
    """
    Convert coarse 4x4 camera poses into MegaPose's CUDA collection.

    Missing or mismatched scores default to ``1.0``. The returned tensor
    collection owns CUDA pose tensors and labels every hypothesis with the BOP
    object identifier expected by the refiner dataset.
    """
    poses = torch_module.as_tensor(coarse_estimates.get("poses", []), dtype=torch_module.float32)
    if poses.ndim != 3 or poses.shape[1:] != (4, 4):
        raise ValueError("coarse_estimates.poses должен иметь размер [N,4,4]")

    num_poses = int(poses.shape[0])
    scores = list(coarse_estimates.get("scores", []))
    if len(scores) != num_poses:
        scores = [1.0] * num_poses

    infos = pandas_module.DataFrame(
        {
            "scene_id": [1] * num_poses,
            "im_id": [1] * num_poses,
            "matching_score": [float(score) for score in scores],
            "batch_im_id": [0] * num_poses,
            "instance_id": [0] * num_poses,
            "hypothesis_id": list(range(num_poses)),
            "label": [f"obj_{object_id:06d}"] * num_poses,
            "time": [float(coarse_estimates.get("elapsed_sec", 0.0))] * num_poses,
        }
    )
    return tensor_collection_type(infos=infos, poses=poses.cuda())
