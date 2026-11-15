"""
MegaPose asset, process and renderer parameters.

Responsibility:
    Declare and validate MegaPose model, mesh, renderer, subprocess, and depth
    settings as one typed backend configuration group.

Does not own:
    Asset installation or inference execution.
"""

from __future__ import annotations

from dataclasses import dataclass
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


VALID_MESH_UNITS = {"m", "meter", "meters", "mm", "millimeter", "millimeters"}
_MEGAPOSE_PARAMETER_SPECS: tuple[ParameterSpec, ...] = (
    ("megapose_python", ""),
    ("megapose_data_dir", ""),
    ("megapose_model", "auto"),
    ("object_label", ""),
    ("mesh_file_path", ""),
    ("mesh_units", "m"),
    ("persistent_worker", False),
    ("bsz_images", 64),
    ("bsz_objects", 8),
    ("renderer_workers", 1),
)


@dataclass(frozen=True)
class MegaPoseParameters:
    """MegaPose runtime and shared object-asset configuration."""

    megapose_python: str
    megapose_data_dir: Path
    megapose_model: str
    object_label: str
    mesh_file_path: Path
    mesh_units: str
    persistent_worker: bool
    bsz_images: int
    bsz_objects: int
    renderer_workers: int


def declare_megapose_parameters(node: ParameterNode) -> None:
    """Declare MegaPose process, renderer and shared object-asset inputs."""
    declare_parameters(node, _MEGAPOSE_PARAMETER_SPECS)


def read_megapose_parameters(node: ParameterNode) -> MegaPoseParameters:
    """Normalize one immutable MegaPose backend configuration."""
    reader = ParameterReader(node)
    return MegaPoseParameters(
        megapose_python=reader.text(
            "megapose_python",
            fallback="",
        ),
        megapose_data_dir=reader.path("megapose_data_dir"),
        megapose_model=reader.text("megapose_model"),
        object_label=reader.text("object_label"),
        mesh_file_path=reader.path("mesh_file_path"),
        mesh_units=reader.text("mesh_units").lower(),
        persistent_worker=reader.boolean("persistent_worker"),
        bsz_images=max(1, int(reader.value("bsz_images"))),
        bsz_objects=max(1, int(reader.value("bsz_objects"))),
        renderer_workers=max(1, int(reader.value("renderer_workers"))),
    )


def validate_megapose_parameters(
    params: MegaPoseParameters,
    *,
    check_files: bool,
) -> None:
    """Validate object metadata and optionally require the mesh asset."""
    if params.mesh_units not in VALID_MESH_UNITS:
        raise ValueError("mesh_units должен быть 'm' или 'mm'")
    require_non_empty(
        {
            "megapose_python": params.megapose_python,
            "megapose_data_dir": "" if params.megapose_data_dir == Path(".") else params.megapose_data_dir,
            "megapose_model": params.megapose_model,
            "object_label": params.object_label,
            "mesh_file_path": "" if params.mesh_file_path == Path(".") else params.mesh_file_path,
        },
        context="MegaPose backend",
    )
    require_identifier(
        params.object_label,
        "object_label",
        context="MegaPose backend",
    )
    if shutil.which(params.megapose_python) is None:
        raise FileNotFoundError(f"MegaPose python не найден: {params.megapose_python}")
    if check_files:
        if not params.megapose_data_dir.is_dir():
            raise FileNotFoundError(
                f"MegaPose data directory не найдена: {params.megapose_data_dir}"
            )
        if not params.mesh_file_path.is_file():
            raise FileNotFoundError(f"MegaPose mesh не найден: {params.mesh_file_path}")


__all__ = [
    "MegaPoseParameters",
    "VALID_MESH_UNITS",
    "declare_megapose_parameters",
    "read_megapose_parameters",
    "validate_megapose_parameters",
]
