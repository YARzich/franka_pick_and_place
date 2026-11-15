"""
Isaac-specific stereo table-camera profile, poses and process action.

Responsibility:
- Derive Isaac stereo camera intrinsics/poses from the shared Gazebo camera profile.
- Create the managed Isaac process action for table-camera scenarios.

Does not own:
- Gazebo camera spawning or bridge YAML generation.
- Perception pipeline node construction.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from launch.actions import ExecuteProcess

from .actions import create_managed_isaac_command
from .command import create_isaac_command
from .contracts import (
    IsaacLaunchPathsLike,
    LEFT_CAMERA_INFO_TOPIC,
    LEFT_IMAGE_TOPIC,
    RIGHT_CAMERA_INFO_TOPIC,
    RIGHT_IMAGE_TOPIC,
    STEREO_BASELINE_M,
    STEREO_LEFT_OPTICAL_FRAME_ID,
    STEREO_RIGHT_OPTICAL_FRAME_ID,
)
from .environment import create_isaac_process_environment
from .scenarios import IsaacMoveItScenario


@dataclass(frozen=True)
class IsaacStereoRigSpec:
    """Geometry shared by the Isaac runner arguments and the published camera TF."""

    center_position: tuple[float, float, float]
    baseline_m: float = STEREO_BASELINE_M
    pitch_down_rad: float = 0.78539816339
    yaw_rad: float = 3.14159265359
    look_distance_m: float = 1.0

    def __post_init__(self) -> None:
        """Reject geometry that cannot produce a valid runner/TF pair."""
        if len(self.center_position) != 3:
            raise ValueError("Stereo center position must contain exactly three values")
        geometry_values = (
            *self.center_position,
            self.baseline_m,
            self.pitch_down_rad,
            self.yaw_rad,
            self.look_distance_m,
        )
        if not all(math.isfinite(value) for value in geometry_values):
            raise ValueError("Stereo rig geometry must contain only finite values")
        if self.baseline_m <= 0.0:
            raise ValueError("Stereo baseline must be positive")
        if self.look_distance_m <= 0.0:
            raise ValueError("Stereo look distance must be positive")

    @classmethod
    def above_cube(
        cls,
        cube_position: tuple[float, float, float],
        *,
        stereo_baseline_m: float = STEREO_BASELINE_M,
    ) -> "IsaacStereoRigSpec":
        """Create the table-camera geometry used by the standard Isaac scenario."""
        return cls(
            center_position=(1.0, 0.0, cube_position[2] + 0.30),
            baseline_m=stereo_baseline_m,
        )

    def world_poses(self) -> tuple[dict[str, float], dict[str, float]]:
        """Return world poses for the left and right optical frames."""
        center_x, center_y, center_z = self.center_position
        half_baseline = 0.5 * self.baseline_m
        common_pose = {
            "x": center_x,
            "z": center_z,
            "roll": 0.0,
            "pitch": self.pitch_down_rad,
            "yaw": self.yaw_rad,
        }
        left_world_pose = {**common_pose, "y": center_y - half_baseline}
        right_world_pose = {**common_pose, "y": center_y + half_baseline}
        return left_world_pose, right_world_pose

    def runner_arguments(self) -> list[str]:
        """Return Isaac runner CLI arguments derived from this geometry."""
        center_x, center_y, center_z = self.center_position
        return [
            "--stereo-center-position",
            str(center_x),
            str(center_y),
            str(center_z),
            "--stereo-baseline-m",
            str(self.baseline_m),
            "--stereo-yaw-rad",
            str(self.yaw_rad),
            "--stereo-pitch-down-rad",
            str(self.pitch_down_rad),
            "--stereo-look-distance-m",
            str(self.look_distance_m),
            "--stereo-camera-local-z-offset-m",
            "0.0",
        ]


def load_gazebo_camera_profile(camera_sdf_path: Path) -> dict[str, float]:
    """Convert the canonical Gazebo camera model into Isaac intrinsics."""
    tree = ET.parse(camera_sdf_path)
    root = tree.getroot()
    left_camera = root.find(".//sensor[@name='table_stereo_left_camera']/camera")
    if left_camera is None:
        raise RuntimeError(
            f"Не найдена левая камера stereo в {camera_sdf_path}"
        )

    horizontal_fov_rad = float(left_camera.findtext("horizontal_fov", default="1.5707963268"))
    width = int(left_camera.findtext("image/width", default="960"))
    height = int(left_camera.findtext("image/height", default="576"))
    near_clip_m = float(left_camera.findtext("clip/near", default="0.05"))
    far_clip_m = float(left_camera.findtext("clip/far", default="20.0"))

    horizontal_aperture_mm = 20.0
    focal_length_mm = horizontal_aperture_mm / (2.0 * math.tan(horizontal_fov_rad * 0.5))
    vertical_aperture_mm = horizontal_aperture_mm * (float(height) / float(width))

    return {
        "width": width,
        "height": height,
        "horizontal_fov_rad": horizontal_fov_rad,
        "horizontal_aperture_mm": horizontal_aperture_mm,
        "vertical_aperture_mm": vertical_aperture_mm,
        "focal_length_mm": focal_length_mm,
        "near_clip_m": near_clip_m,
        "far_clip_m": far_clip_m,
    }


def create_isaac_stereo_world_poses(
    cube_position: tuple[float, float, float],
    *,
    stereo_baseline_m: float = STEREO_BASELINE_M,
) -> tuple[dict[str, float], dict[str, float]]:
    """Return legacy left/right world poses derived from the rig contract."""
    return IsaacStereoRigSpec.above_cube(
        cube_position,
        stereo_baseline_m=stereo_baseline_m,
    ).world_poses()


def create_isaac_table_camera_process(
    *,
    paths: IsaacLaunchPathsLike,
    scenario: IsaacMoveItScenario,
    camera_profile: dict[str, float],
    stereo_rig: IsaacStereoRigSpec | None = None,
    isaac_python: str,
    isaac_prim: str,
    headless: bool,
    fix_base: bool,
    time_steps: str,
    isaac_experience: str,
    rmw_implementation: str,
) -> ExecuteProcess:
    """Create a managed Isaac process with the stereo-camera CLI contract."""
    if stereo_rig is None:
        stereo_rig = IsaacStereoRigSpec.above_cube(scenario.cube_position)

    isaac_cmd = create_isaac_command(
        paths=paths,
        scenario=scenario,
        isaac_python=isaac_python,
        isaac_prim=isaac_prim,
        headless=headless,
        fix_base=fix_base,
        time_steps=time_steps,
        isaac_experience=isaac_experience,
    )
    isaac_cmd.extend(
        [
            "--enable-stereo-camera",
            *stereo_rig.runner_arguments(),
            "--stereo-width",
            str(int(camera_profile["width"])),
            "--stereo-height",
            str(int(camera_profile["height"])),
            "--stereo-focal-length-mm",
            str(float(camera_profile["focal_length_mm"])),
            "--stereo-horizontal-aperture-mm",
            str(float(camera_profile["horizontal_aperture_mm"])),
            "--stereo-vertical-aperture-mm",
            str(float(camera_profile["vertical_aperture_mm"])),
            "--stereo-focus-distance-m",
            "400.0",
            "--stereo-near-clip-m",
            str(float(camera_profile["near_clip_m"])),
            "--stereo-far-clip-m",
            str(float(camera_profile["far_clip_m"])),
            "--stereo-left-image-topic",
            LEFT_IMAGE_TOPIC,
            "--stereo-right-image-topic",
            RIGHT_IMAGE_TOPIC,
            "--stereo-left-camera-info-topic",
            LEFT_CAMERA_INFO_TOPIC,
            "--stereo-right-camera-info-topic",
            RIGHT_CAMERA_INFO_TOPIC,
            "--stereo-left-frame-id",
            STEREO_LEFT_OPTICAL_FRAME_ID,
            "--stereo-right-frame-id",
            STEREO_RIGHT_OPTICAL_FRAME_ID,
        ]
    )
    return ExecuteProcess(
        cmd=create_managed_isaac_command(paths, isaac_cmd),
        output="screen",
        additional_env=create_isaac_process_environment(isaac_python, rmw_implementation),
    )
