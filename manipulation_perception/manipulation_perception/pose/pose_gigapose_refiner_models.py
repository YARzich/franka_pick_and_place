"""
Runtime model-dataset preparation for the GigaPose refiner worker.

Responsibility:
- Validate local GigaPose/MegaPose refiner files.
- Prepare a BOP-style runtime models directory for the MegaPose refiner.
- Keep mesh scaling/densifying rules in one place.

Does not own:
- Loading neural network checkpoints into PyTorch modules.
- Running refiner/scoring inference.
- Worker JSON protocol.
"""

from __future__ import annotations

import json
from pathlib import Path
import shutil

import numpy as np

from manipulation_perception.pose.mesh_utils import (
    copy_material_assets,
    read_obj_vertices,
    write_dense_scaled_obj,
    write_scaled_obj,
)
from manipulation_perception.pose.pose_gigapose_refiner_worker_inputs import (
    GigaPoseRefinerBackendConfig,
)


def validate_gigapose_refiner_runtime_files(request: GigaPoseRefinerBackendConfig) -> None:
    """Fail before CUDA setup when model, mesh or checkpoint inputs are absent."""
    if not (request.repo_dir / "configs" / "model" / "refiner" / "rgb.yaml").is_file():
        raise FileNotFoundError(f"GigaPose refiner config не найден: {request.repo_dir}")

    models_dir = request.root_dir / "datasets" / request.dataset_name / "models"
    models_info_path = models_dir / "models_info.json"
    mesh_path = models_dir / f"obj_{request.object_id:06d}.obj"
    if not models_info_path.is_file():
        raise FileNotFoundError(f"GigaPose models_info.json не найден: {models_info_path}")
    if not mesh_path.is_file():
        raise FileNotFoundError(f"GigaPose object mesh не найден: {mesh_path}")

    for run_dir in ("coarse-rgb-906902141", "refiner-rgb-653307694"):
        if not (request.megapose_models_root / run_dir / "checkpoint.pth.tar").is_file():
            raise FileNotFoundError(
                "MegaPose model checkpoint для GigaPose refiner не найден: "
                f"{request.megapose_models_root / run_dir}"
            )


def prepare_gigapose_refiner_models_dir(request: GigaPoseRefinerBackendConfig) -> Path:
    """Replace the runtime BOP model metadata and scaled/densified object mesh."""
    source_dir = request.root_dir / "datasets" / request.dataset_name / "models"
    request.refiner_models_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(
        source_dir / "models_info.json",
        request.refiner_models_dir / "models_info.json",
    )

    source_mesh = source_dir / f"obj_{request.object_id:06d}.obj"
    target_mesh = request.refiner_models_dir / source_mesh.name
    scale = mesh_to_bop_mm_scale(source_mesh, source_dir / "models_info.json", request.object_id)
    if count_obj_vertices(source_mesh) < 2000:
        write_dense_scaled_obj(source_mesh, target_mesh, scale, min_vertices=2500, fallback="copy")
    else:
        write_scaled_obj(source_mesh, target_mesh, scale)
    copy_material_assets(source_mesh, request.refiner_models_dir)
    return request.refiner_models_dir


def count_obj_vertices(input_path: Path) -> int:
    """Count geometric vertex records without parsing full OBJ topology."""
    return sum(
        1
        for line in input_path.read_text(errors="ignore").splitlines()
        if line.startswith("v ")
    )


def mesh_to_bop_mm_scale(mesh_path: Path, models_info_path: Path, object_id: int) -> float:
    """
    Infer whether OBJ coordinates need metre-to-millimetre BOP scaling.

    The models-info diameter is compared with the raw mesh diagonal. A ratio
    above 100 denotes metre coordinates relative to BOP's millimetre metadata.
    """
    vertices = read_obj_vertices(mesh_path)
    extents = vertices.max(axis=0) - vertices.min(axis=0)
    source_diameter = float(np.linalg.norm(extents))
    if source_diameter <= 0.0:
        return 1.0

    models_info = json.loads(models_info_path.read_text())
    object_info = models_info.get(str(object_id), {})
    bop_diameter = float(object_info.get("diameter", source_diameter))
    ratio = bop_diameter / source_diameter
    return 1000.0 if ratio > 100.0 else 1.0
