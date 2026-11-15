"""
GigaPose coarse-estimator and MegaPose-refiner parameters.

Responsibility:
    Declare and validate GigaPose retrieval, dataset, subprocess, and optional
    MegaPose refinement settings as one typed configuration group.

Does not own:
    Worker construction or inference execution.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import shutil

from manipulation_perception.common.config_validation import (
    require_identifier,
    require_non_empty,
)
from manipulation_perception.pose.pose_parameter_access import (
    declare_parameters,
    ParameterNode,
    ParameterReader,
    ParameterSpec,
)


_DEFAULT_CHECKPOINT_RELATIVE_PATH = Path('pretrained') / 'gigaPose_v1.ckpt'


_GIGAPOSE_PARAMETER_SPECS: tuple[ParameterSpec, ...] = (
    ("detection_mask_topic", ""),
    ("gigapose_python", ""),
    ("gigapose_repo_dir", ""),
    ("gigapose_dinov2_repo_dir", ""),
    ("gigapose_data_dir", ""),
    ("gigapose_checkpoint_path", ""),
    ("gigapose_dataset_name", ""),
    ("gigapose_object_id", 0),
    ("gigapose_template_count", 162),
    ("gigapose_max_num_dets_per_forward", 4),
    ("gigapose_translation_scale", 0.001),
    ("gigapose_num_hypotheses", 5),
    ("gigapose_min_matching_score", 0.0),
    ("gigapose_persistent_worker", False),
    ("gigapose_refiner_bsz_images", 512),
    ("gigapose_refiner_bsz_objects", 8),
    ("gigapose_refiner_iterations", 5),
)


@dataclass(frozen=True)
class GigaPoseParameters:
    """GigaPose worker, dataset, checkpoint and refiner configuration."""

    detection_mask_topic: str
    gigapose_python: str
    gigapose_repo_dir: Path
    gigapose_dinov2_repo_dir: Path | None
    gigapose_data_dir: Path
    gigapose_checkpoint_path: Path
    gigapose_dataset_name: str
    gigapose_object_id: int
    gigapose_template_count: int
    gigapose_max_num_dets_per_forward: int
    gigapose_translation_scale: float
    gigapose_num_hypotheses: int
    gigapose_min_matching_score: float
    gigapose_persistent_worker: bool
    gigapose_refiner_bsz_images: int
    gigapose_refiner_bsz_objects: int
    gigapose_refiner_iterations: int


def resolve_gigapose_checkpoint(
    data_dir: Path,
    override: str | Path | None,
) -> Path:
    """Resolve an explicit checkpoint or the documented data-dir default."""
    value = str(override or '').strip()
    if value:
        return Path(value).expanduser()
    return data_dir / _DEFAULT_CHECKPOINT_RELATIVE_PATH


def declare_gigapose_parameters(node: ParameterNode) -> None:
    """Declare GigaPose worker, dataset, checkpoint and refiner inputs."""
    declare_parameters(node, _GIGAPOSE_PARAMETER_SPECS)


def read_gigapose_parameters(node: ParameterNode) -> GigaPoseParameters:
    """Normalize paths, limits and the default checkpoint fallback."""
    reader = ParameterReader(node)
    data_dir = reader.path("gigapose_data_dir")
    return GigaPoseParameters(
        detection_mask_topic=str(reader.value("detection_mask_topic")),
        gigapose_python=reader.text(
            "gigapose_python",
            fallback="",
        ),
        gigapose_repo_dir=reader.path("gigapose_repo_dir"),
        gigapose_dinov2_repo_dir=reader.optional_path("gigapose_dinov2_repo_dir"),
        gigapose_data_dir=data_dir,
        gigapose_checkpoint_path=resolve_gigapose_checkpoint(
            data_dir,
            reader.value('gigapose_checkpoint_path'),
        ),
        gigapose_dataset_name=reader.text("gigapose_dataset_name"),
        gigapose_object_id=int(reader.value("gigapose_object_id")),
        gigapose_template_count=max(1, int(reader.value("gigapose_template_count"))),
        gigapose_max_num_dets_per_forward=max(
            1,
            int(reader.value("gigapose_max_num_dets_per_forward")),
        ),
        gigapose_translation_scale=float(reader.value("gigapose_translation_scale")),
        gigapose_num_hypotheses=max(1, int(reader.value("gigapose_num_hypotheses"))),
        gigapose_min_matching_score=float(
            reader.value("gigapose_min_matching_score")
        ),
        gigapose_persistent_worker=reader.boolean("gigapose_persistent_worker"),
        gigapose_refiner_bsz_images=max(
            1,
            int(reader.value("gigapose_refiner_bsz_images")),
        ),
        gigapose_refiner_bsz_objects=max(
            1,
            int(reader.value("gigapose_refiner_bsz_objects")),
        ),
        gigapose_refiner_iterations=max(
            1,
            int(reader.value("gigapose_refiner_iterations")),
        ),
    )


def validate_gigapose_parameters(
    params: GigaPoseParameters,
    *,
    check_files: bool,
) -> None:
    """Validate BOP identity and optionally require external runtime assets."""
    require_non_empty(
        {
            "detection_mask_topic": params.detection_mask_topic,
            "gigapose_python": params.gigapose_python,
            "gigapose_repo_dir": (
                "" if params.gigapose_repo_dir == Path(".") else params.gigapose_repo_dir
            ),
            "gigapose_data_dir": (
                "" if params.gigapose_data_dir == Path(".") else params.gigapose_data_dir
            ),
            "gigapose_checkpoint_path": (
                ""
                if params.gigapose_checkpoint_path == Path(".")
                else params.gigapose_checkpoint_path
            ),
            "gigapose_dataset_name": params.gigapose_dataset_name,
        },
        context="GigaPose backend",
    )
    require_identifier(
        params.gigapose_dataset_name,
        "gigapose_dataset_name",
        context="GigaPose backend",
    )
    if params.gigapose_object_id <= 0:
        raise ValueError("gigapose_object_id должен быть положительным BOP object id")
    if (
        not math.isfinite(params.gigapose_min_matching_score)
        or not 0.0 <= params.gigapose_min_matching_score <= 1.0
    ):
        raise ValueError(
            "gigapose_min_matching_score должен быть конечным числом "
            "в диапазоне [0, 1]"
        )
    if not check_files:
        return
    if not (params.gigapose_repo_dir / "configs" / "test.yaml").is_file():
        raise FileNotFoundError(
            "GigaPose repo не найден или неполный: "
            f"{params.gigapose_repo_dir}"
        )
    if (
        params.gigapose_dinov2_repo_dir is not None
        and not (params.gigapose_dinov2_repo_dir / "hubconf.py").is_file()
    ):
        raise FileNotFoundError(
            "DINOv2 repo не найден или неполный: "
            f"{params.gigapose_dinov2_repo_dir}"
        )
    if shutil.which(params.gigapose_python) is None:
        raise FileNotFoundError(f"GigaPose python не найден: {params.gigapose_python}")
    if not params.gigapose_data_dir.is_dir():
        raise FileNotFoundError(
            f"GigaPose data directory не найдена: {params.gigapose_data_dir}"
        )
    if not params.gigapose_checkpoint_path.is_file():
        raise FileNotFoundError(
            f"GigaPose checkpoint не найден: {params.gigapose_checkpoint_path}"
        )


__all__ = [
    "GigaPoseParameters",
    "declare_gigapose_parameters",
    "read_gigapose_parameters",
    "resolve_gigapose_checkpoint",
    "validate_gigapose_parameters",
]
