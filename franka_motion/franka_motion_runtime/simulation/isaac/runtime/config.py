"""
CLI config for the Franka Isaac Sim runtime entrypoint.

Responsibility:
- Own argparse declarations and convert raw CLI values into typed config objects.
- Keep public runner flags stable for launch files and direct script usage.
- Validate required asset paths before importing Isaac Sim.

Does not own:
- Isaac Sim imports, scene creation or ROS graph construction.
- Numeric/name invariants; those live in the pure validation module.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Sequence

from .cli import create_runner_parser
from .validation import validate_runner_config


@dataclass(frozen=True)
class CubeConfig:
    """Optional pick-cube geometry and physics settings."""

    add_cube: bool = False
    size: float = 0.05
    position: tuple[float, float, float] = (0.4, 0.0, 0.03)
    static_friction: float = 0.8
    dynamic_friction: float = 0.6
    restitution: float = 0.0
    prim: str = "/World/PickCube"


@dataclass(frozen=True)
class StereoCameraConfig:
    """Stereo sensor geometry, rendering and ROS publication settings."""

    enabled: bool = False
    center_position: tuple[float, float, float] = (1.0, 0.0, 1.0)
    baseline_m: float = 0.06
    yaw_rad: float = math.pi
    pitch_down_rad: float = math.pi / 4.0
    look_distance_m: float = 1.0
    width: int = 960
    height: int = 576
    focal_length_mm: float = 10.0
    horizontal_aperture_mm: float = 20.0
    vertical_aperture_mm: float = 15.0
    focus_distance_m: float = 400.0
    near_clip_m: float = 0.05
    far_clip_m: float = 20.0
    camera_local_z_offset_m: float = -0.045
    left_image_topic: str = "/table_stereo/left/image_raw"
    right_image_topic: str = "/table_stereo/right/image_raw"
    left_camera_info_topic: str = "/table_stereo/left/camera_info"
    right_camera_info_topic: str = "/table_stereo/right/camera_info"
    left_frame_id: str = "table_stereo_left_frame"
    right_frame_id: str = "table_stereo_right_frame"


@dataclass(frozen=True)
class IsaacRunnerConfig:
    """Complete typed configuration consumed by the Isaac runtime."""

    install_path: Path | None = None
    urdf_path: Path | None = None
    robot_usd: Path | None = None
    scene_prim: str = "/World/Franka"
    scene_usd: Path | None = None
    fix_base: bool = False
    headless: bool = False
    no_default_ground: bool = False
    cube: CubeConfig = CubeConfig()
    time_steps: int = 0
    experience: str = ""
    ros_distro: str = "jazzy"
    command_topic: str = "/isaac_joint_commands"
    state_topic: str = "/isaac_joint_states"
    stereo: StereoCameraConfig = StereoCameraConfig()


def parse_runner_config(argv: Sequence[str] | None = None) -> IsaacRunnerConfig:
    """Parse public CLI flags into an immutable runtime configuration."""
    args = create_runner_parser().parse_args(argv)
    return IsaacRunnerConfig(
        install_path=Path(args.install_path).expanduser() if args.install_path else None,
        urdf_path=_optional_path(args.urdf_path),
        robot_usd=_optional_path(args.robot_usd),
        scene_prim=args.scene_prim,
        scene_usd=_optional_path(args.scene_usd),
        fix_base=args.fix_base,
        headless=args.headless,
        no_default_ground=args.no_default_ground,
        cube=CubeConfig(
            add_cube=args.add_cube,
            size=args.cube_size,
            position=_float_triple(args.cube_position),
            static_friction=args.cube_static_friction,
            dynamic_friction=args.cube_dynamic_friction,
            restitution=args.cube_restitution,
            prim=args.cube_prim,
        ),
        time_steps=args.time_steps,
        experience=args.experience,
        ros_distro=args.ros_distro,
        command_topic=args.command_topic,
        state_topic=args.state_topic,
        stereo=StereoCameraConfig(
            enabled=args.enable_stereo_camera,
            center_position=_float_triple(args.stereo_center_position),
            baseline_m=args.stereo_baseline_m,
            yaw_rad=args.stereo_yaw_rad,
            pitch_down_rad=args.stereo_pitch_down_rad,
            look_distance_m=args.stereo_look_distance_m,
            width=args.stereo_width,
            height=args.stereo_height,
            focal_length_mm=args.stereo_focal_length_mm,
            horizontal_aperture_mm=args.stereo_horizontal_aperture_mm,
            vertical_aperture_mm=args.stereo_vertical_aperture_mm,
            focus_distance_m=args.stereo_focus_distance_m,
            near_clip_m=args.stereo_near_clip_m,
            far_clip_m=args.stereo_far_clip_m,
            camera_local_z_offset_m=args.stereo_camera_local_z_offset_m,
            left_image_topic=args.stereo_left_image_topic,
            right_image_topic=args.stereo_right_image_topic,
            left_camera_info_topic=args.stereo_left_camera_info_topic,
            right_camera_info_topic=args.stereo_right_camera_info_topic,
            left_frame_id=args.stereo_left_frame_id,
            right_frame_id=args.stereo_right_frame_id,
        ),
    )


def validate_runner_paths(config: IsaacRunnerConfig) -> None:
    """Validate semantic invariants and required filesystem resources."""
    validate_runner_config(config)
    if config.install_path is not None and not config.install_path.exists():
        raise FileNotFoundError(f"Isaac Sim не найден: {config.install_path}")
    if config.robot_usd is None and config.urdf_path is None:
        raise ValueError(
            "Нужно указать --robot-usd или --urdf-path "
            "для загрузки робота"
        )
    if config.robot_usd is not None and not config.robot_usd.exists():
        raise FileNotFoundError(f"USD робот не найден: {config.robot_usd}")
    if config.urdf_path is not None and not config.urdf_path.exists():
        raise FileNotFoundError(f"URDF робот не найден: {config.urdf_path}")
    if config.scene_usd is not None and not config.scene_usd.exists():
        raise FileNotFoundError(f"USD сцена не найдена: {config.scene_usd}")
    if config.experience and not Path(config.experience).exists():
        raise FileNotFoundError(f"Isaac experience не найден: {config.experience}")


def _optional_path(value: str) -> Path | None:
    return Path(value) if value else None


def _float_triple(values: Sequence[float]) -> tuple[float, float, float]:
    first, second, third = values
    return (float(first), float(second), float(third))
