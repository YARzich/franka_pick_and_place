"""
Resolve resource paths used by table-camera launch parts.

Responsibility:
- Collect table-camera config, RViz, model and mesh paths into a typed container.
- Keep camera launch entrypoints from hardcoding package resource paths.

Does not own:
- Checking that optional backend dependencies are installed.
- Creating launch arguments or perception actions.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class TableCameraPaths:
    """Resources shared by camera perception and visualization actions."""

    camera_sdf: Path
    rviz_config: Path
    perception_config: Path
    yolo_model: Path
    foundationpose_mesh: Path
    foundationpose_texture: Path


def create_table_camera_paths(
    bringup_share: Path,
) -> TableCameraPaths:
    """Resolve robot/scenario assets from the self-contained package."""
    foundationpose_dir = bringup_share / "models" / "foundationpose"
    return TableCameraPaths(
        camera_sdf=(
            bringup_share / "description" / "gazebo" / "table_camera.sdf"
        ),
        rviz_config=(
            bringup_share / "config" / "rviz" /
            "franka_table_stereo_pointcloud.rviz"
        ),
        perception_config=(
            bringup_share / "config" / "perception" /
            "table_camera_perception.yaml"
        ),
        yolo_model=(
            bringup_share / "models" / "cube_yolo26s_isaac_seg_v5s.pt"
        ),
        foundationpose_mesh=foundationpose_dir / "cube_5cm.obj",
        foundationpose_texture=foundationpose_dir / "cube_5cm_texture.ppm",
    )
