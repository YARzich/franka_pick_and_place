"""
Argument parser for the Isaac Sim runner.

Responsibility:
- Declare legacy underscore and current kebab-case runner flags in one place.
- Preserve CLI defaults independently from typed config construction.

Does not own:
- Converting parsed values into dataclasses or validating their semantics.
- Starting Isaac Sim or resolving package resources.
"""

from __future__ import annotations

import argparse
import math


def create_runner_parser() -> argparse.ArgumentParser:
    """Create the parser for the stable Isaac runner CLI contract."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--install_path",
        help="Optional Isaac root; otherwise discover the installed isaacsim package.",
    )
    parser.add_argument("--urdf_path", "--urdf-path", default="", dest="urdf_path")
    parser.add_argument("--robot_usd", "--robot-usd", default="", dest="robot_usd")
    parser.add_argument(
        "--scene_prim",
        "--scene-prim",
        default="/World/Franka",
        dest="scene_prim",
    )
    parser.add_argument("--scene_usd", "--scene-usd", default="", dest="scene_usd")
    parser.add_argument("--fix_base", "--fix-base", action="store_true", dest="fix_base")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--no_default_ground",
        "--no-default-ground",
        action="store_true",
        dest="no_default_ground",
    )
    parser.add_argument("--add_cube", "--add-cube", action="store_true", dest="add_cube")
    parser.add_argument(
        "--cube_size",
        "--cube-size",
        type=float,
        default=0.05,
        dest="cube_size",
    )
    parser.add_argument(
        "--cube_position",
        "--cube-position",
        type=float,
        nargs=3,
        default=[0.4, 0.0, 0.03],
        dest="cube_position",
    )
    parser.add_argument(
        "--cube_static_friction",
        "--cube-static-friction",
        type=float,
        default=0.8,
        dest="cube_static_friction",
    )
    parser.add_argument(
        "--cube_dynamic_friction",
        "--cube-dynamic-friction",
        type=float,
        default=0.6,
        dest="cube_dynamic_friction",
    )
    parser.add_argument(
        "--cube_restitution",
        "--cube-restitution",
        type=float,
        default=0.0,
        dest="cube_restitution",
    )
    parser.add_argument(
        "--cube_prim",
        "--cube-prim",
        default="/World/PickCube",
        dest="cube_prim",
    )
    parser.add_argument("--time_steps", "--time-steps", type=int, default=0, dest="time_steps")
    parser.add_argument("--experience", default="")
    parser.add_argument("--ros_distro", default="jazzy")
    parser.add_argument(
        "--command_topic",
        "--command-topic",
        default="/isaac_joint_commands",
        dest="command_topic",
    )
    parser.add_argument(
        "--state_topic",
        "--state-topic",
        default="/isaac_joint_states",
        dest="state_topic",
    )
    _add_stereo_arguments(parser)
    return parser


def _add_stereo_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--enable_stereo_camera",
        "--enable-stereo-camera",
        action="store_true",
        dest="enable_stereo_camera",
    )
    parser.add_argument(
        "--stereo_center_position",
        "--stereo-center-position",
        type=float,
        nargs=3,
        default=[1.0, 0.0, 1.0],
        dest="stereo_center_position",
    )
    _add_typed_argument(
        parser,
        "stereo_baseline_m",
        argument_type=float,
        default=0.06,
    )
    _add_typed_argument(
        parser,
        "stereo_yaw_rad",
        argument_type=float,
        default=math.pi,
    )
    _add_typed_argument(
        parser,
        "stereo_pitch_down_rad",
        argument_type=float,
        default=math.pi / 4.0,
    )
    _add_typed_argument(
        parser,
        "stereo_look_distance_m",
        argument_type=float,
        default=1.0,
    )
    _add_typed_argument(parser, "stereo_width", argument_type=int, default=960)
    _add_typed_argument(parser, "stereo_height", argument_type=int, default=576)
    _add_typed_argument(
        parser,
        "stereo_focal_length_mm",
        argument_type=float,
        default=10.0,
    )
    _add_typed_argument(
        parser,
        "stereo_horizontal_aperture_mm",
        argument_type=float,
        default=20.0,
    )
    _add_typed_argument(
        parser,
        "stereo_vertical_aperture_mm",
        argument_type=float,
        default=15.0,
    )
    _add_typed_argument(
        parser,
        "stereo_focus_distance_m",
        argument_type=float,
        default=400.0,
    )
    _add_typed_argument(
        parser,
        "stereo_near_clip_m",
        argument_type=float,
        default=0.05,
    )
    _add_typed_argument(
        parser,
        "stereo_far_clip_m",
        argument_type=float,
        default=20.0,
    )
    _add_typed_argument(
        parser,
        "stereo_camera_local_z_offset_m",
        argument_type=float,
        default=-0.045,
    )
    _add_string_argument(
        parser,
        "stereo_left_image_topic",
        "/table_stereo/left/image_raw",
    )
    _add_string_argument(
        parser,
        "stereo_right_image_topic",
        "/table_stereo/right/image_raw",
    )
    _add_string_argument(
        parser,
        "stereo_left_camera_info_topic",
        "/table_stereo/left/camera_info",
    )
    _add_string_argument(
        parser,
        "stereo_right_camera_info_topic",
        "/table_stereo/right/camera_info",
    )
    _add_string_argument(parser, "stereo_left_frame_id", "table_stereo_left_frame")
    _add_string_argument(parser, "stereo_right_frame_id", "table_stereo_right_frame")


def _add_typed_argument(
    parser: argparse.ArgumentParser,
    name: str,
    *,
    argument_type: type,
    default: object,
) -> None:
    parser.add_argument(
        f"--{name}",
        f"--{name.replace('_', '-')}",
        type=argument_type,
        default=default,
        dest=name,
    )


def _add_string_argument(
    parser: argparse.ArgumentParser,
    name: str,
    default: str,
) -> None:
    parser.add_argument(
        f"--{name}",
        f"--{name.replace('_', '-')}",
        default=default,
        dest=name,
    )
