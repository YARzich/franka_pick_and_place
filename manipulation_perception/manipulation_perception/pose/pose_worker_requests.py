"""
Request builders for MegaPose and GigaPose worker processes.

Responsibility:
- Serialize camera, bbox, path and model options into worker JSON requests.
- Build model-only prewarm requests without frame-specific inference data.
- Preserve existing model/depth-mode selection rules.

Does not own:
- Worker subprocess startup or shutdown.
- Filesystem preparation of runtime examples.
- ROS message callbacks or publishers.
"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence


RGB_MODEL = "megapose-1.0-RGB-multi-hypothesis"
RGBD_MODEL = "megapose-1.0-RGBD"
ICP_MODEL = "megapose-1.0-RGB-multi-hypothesis-icp"


def camera_k_matrix(k_values: Sequence[float]) -> list[list[float]]:
    """Convert the first nine CameraInfo values into a row-major 3x3 matrix."""
    if len(k_values) < 9:
        raise ValueError("CameraInfo.k должен содержать 9 значений")
    k = [float(value) for value in k_values[:9]]
    return [
        [k[0], k[1], k[2]],
        [k[3], k[4], k[5]],
        [k[6], k[7], k[8]],
    ]


def model_uses_depth(model_name: str) -> bool:
    """Return whether the external model name declares a depth input contract."""
    normalized = model_name.lower()
    return any(token in normalized for token in ("rgbd", "rgb-d", "icp", "depth"))


def select_megapose_model(
    coarse_estimator: str,
    megapose_model: str,
    depth_mode: str,
    has_depth: bool,
) -> str:
    """Resolve the MegaPose model without silently weakening requested depth mode."""
    if coarse_estimator == "gigapose":
        return RGB_MODEL
    if megapose_model and megapose_model.lower() != "auto":
        return megapose_model
    if depth_mode == "icp":
        return ICP_MODEL
    if depth_mode in {"on", "true", "1", "rgbd"}:
        return RGBD_MODEL
    if depth_mode == "auto" and has_depth:
        return RGBD_MODEL
    return RGB_MODEL


def build_megapose_worker_request(
    request_id: int,
    example_name: str,
    example_dir: Path,
    model_name: str,
    object_label: str,
    bsz_images: int,
    bsz_objects: int,
    renderer_workers: int,
    coarse_estimates: dict | None,
) -> dict:
    """Build one JSON-safe MegaPose worker request with optional coarse poses."""
    request = {
        "id": request_id,
        "example_name": example_name,
        "example_dir": str(example_dir),
        "model": model_name,
        "object_label": object_label,
        "bsz_images": bsz_images,
        "bsz_objects": bsz_objects,
        "renderer_workers": renderer_workers,
    }
    if coarse_estimates is not None:
        request["coarse_estimates"] = coarse_estimates
    return request


def build_gigapose_worker_request(
    request_id: int,
    image_path: Path,
    mask_path: Path | None,
    camera_k: Sequence[float],
    bbox: tuple[int, int, int, int],
    repo_dir: Path,
    dinov2_repo_dir: Path | None,
    root_dir: Path,
    dataset_name: str,
    object_id: int,
    checkpoint_path: Path,
    template_count: int,
    max_num_dets_per_forward: int,
    translation_scale: float,
    num_hypotheses: int,
) -> dict:
    """Build a JSON-safe coarse request with pixel bbox and camera intrinsics."""
    return {
        "id": request_id,
        "image_path": str(image_path),
        "mask_path": str(mask_path) if mask_path is not None else "",
        "bbox": [int(value) for value in bbox],
        "K": camera_k_matrix(camera_k),
        "repo_dir": str(repo_dir),
        "dinov2_repo_dir": str(dinov2_repo_dir) if dinov2_repo_dir is not None else "",
        "root_dir": str(root_dir),
        "dataset_name": dataset_name,
        "object_id": object_id,
        "checkpoint_path": str(checkpoint_path),
        "template_count": template_count,
        "max_num_dets_per_forward": max_num_dets_per_forward,
        "translation_scale": translation_scale,
        "num_hypotheses": num_hypotheses,
    }


def build_gigapose_prewarm_request(
    request_id: int,
    repo_dir: Path,
    dinov2_repo_dir: Path | None,
    root_dir: Path,
    dataset_name: str,
    object_id: int,
    checkpoint_path: Path,
    template_count: int,
    max_num_dets_per_forward: int,
    translation_scale: float,
) -> dict:
    """Build a model-only request that fills the coarse worker cache."""
    return {
        "id": request_id,
        "command": "prewarm",
        "repo_dir": str(repo_dir),
        "dinov2_repo_dir": str(dinov2_repo_dir) if dinov2_repo_dir is not None else "",
        "root_dir": str(root_dir),
        "dataset_name": dataset_name,
        "object_id": object_id,
        "checkpoint_path": str(checkpoint_path),
        "template_count": template_count,
        "max_num_dets_per_forward": max_num_dets_per_forward,
        "translation_scale": translation_scale,
    }


def build_gigapose_refiner_worker_request(
    request_id: int,
    image_path: Path,
    camera_k: Sequence[float],
    repo_dir: Path,
    root_dir: Path,
    dataset_name: str,
    object_id: int,
    output_label: str,
    megapose_models_root: Path,
    bsz_images: int,
    bsz_objects: int,
    n_iterations: int,
    coarse_estimates: dict,
) -> dict:
    """Build a JSON-safe refinement request carrying coarse camera poses."""
    return {
        "id": request_id,
        "image_path": str(image_path),
        "K": camera_k_matrix(camera_k),
        "repo_dir": str(repo_dir),
        "root_dir": str(root_dir),
        "dataset_name": dataset_name,
        "object_id": object_id,
        "output_label": output_label,
        "megapose_models_root": str(megapose_models_root),
        "bsz_images": bsz_images,
        "bsz_objects": bsz_objects,
        "n_iterations": n_iterations,
        "coarse_estimates": coarse_estimates,
    }


def build_gigapose_refiner_prewarm_request(
    request_id: int,
    repo_dir: Path,
    root_dir: Path,
    dataset_name: str,
    object_id: int,
    megapose_models_root: Path,
    bsz_images: int,
    bsz_objects: int,
    n_iterations: int,
) -> dict:
    """Build a model-only request that fills the refiner worker cache."""
    return {
        "id": request_id,
        "command": "prewarm",
        "repo_dir": str(repo_dir),
        "root_dir": str(root_dir),
        "dataset_name": dataset_name,
        "object_id": object_id,
        "megapose_models_root": str(megapose_models_root),
        "bsz_images": bsz_images,
        "bsz_objects": bsz_objects,
        "n_iterations": n_iterations,
    }
