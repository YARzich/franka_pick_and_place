#!/usr/bin/env python3


"""
Offline dataset-preparation CLI for the franka_motion GigaPose cube.

Responsibility:
- Create the minimal BOP-like dataset directory used by GigaPose.
- Scale/copy the configured cube mesh and material assets.
- Write models_info.json for the selected object id.

Does not own:
- Online pose-estimation inference.
- ROS node parameters, topics or launch orchestration.
- Generic mesh operations, which live in mesh_utils.py.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from manipulation_perception.pose.mesh_utils import (
    copy_material_assets,
    mesh_unit_scale_to_mm,
    obj_diameter_mm,
    write_scaled_obj,
)


def _default_cube_mesh_path() -> str:
    """Resolve the package-local cube mesh in source and install layouts."""
    try:
        package_share = Path(get_package_share_directory("franka_motion"))
    except PackageNotFoundError:
        package_share = Path(__file__).resolve().parents[2]
    return str(package_share / "models" / "foundationpose" / "cube_5cm.obj")


def main() -> int:
    """Validate CLI input and write one minimal millimetre-based BOP dataset."""
    parser = argparse.ArgumentParser(
        description=(
            "Create a minimal BOP-like GigaPose dataset directory for the "
            "franka_motion cube."
        )
    )
    data_dir_default = os.environ.get("FRANKA_GPU_GIGAPOSE_DATA_DIR", "").strip()
    parser.add_argument(
        "--gigapose-data-dir",
        default=data_dir_default or None,
        required=not data_dir_default,
        help="Dataset root (or set FRANKA_GPU_GIGAPOSE_DATA_DIR).",
    )
    parser.add_argument("--dataset-name", default="franka_cube")
    parser.add_argument("--object-id", type=int, default=1)
    parser.add_argument(
        "--mesh",
        default=_default_cube_mesh_path(),
    )
    parser.add_argument("--mesh-units", choices=["m", "mm"], default="m")
    args = parser.parse_args()

    if not args.gigapose_data_dir.strip():
        parser.error("--gigapose-data-dir must not be empty")
    data_dir = Path(args.gigapose_data_dir).expanduser()
    mesh_path = Path(args.mesh).expanduser()
    if not mesh_path.is_file():
        raise FileNotFoundError(f"Mesh не найден: {mesh_path}")
    if args.object_id <= 0:
        raise ValueError("--object-id должен быть положительным")

    dataset_dir = data_dir / "datasets" / args.dataset_name
    models_dir = dataset_dir / "models"
    models_dir.mkdir(parents=True, exist_ok=True)

    target_mesh = models_dir / f"obj_{args.object_id:06d}{mesh_path.suffix.lower()}"
    if mesh_path.suffix.lower() == ".obj":
        write_scaled_obj(mesh_path, target_mesh, mesh_unit_scale_to_mm(args.mesh_units))
    else:
        shutil.copy2(mesh_path, target_mesh)
    copy_material_assets(mesh_path, models_dir)

    models_info = {
        str(args.object_id): {
            "diameter": obj_diameter_mm(mesh_path, args.mesh_units),
        }
    }
    (models_dir / "models_info.json").write_text(json.dumps(models_info, indent=2) + "\n")
    print(f"GigaPose dataset prepared: {dataset_dir}")
    print(f"CAD model: {target_mesh}")
    print(f"models_info: {models_dir / 'models_info.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
