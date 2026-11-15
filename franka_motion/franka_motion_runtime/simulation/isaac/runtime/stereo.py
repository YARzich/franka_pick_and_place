"""
Stereo camera prim construction for the Isaac Sim runtime.

Responsibility:
- Build the table stereo rig USD prims from typed camera config.
- Keep camera geometry separate from ROS graph publication details.

Does not own:
- OmniGraph ROS image/camera_info publishers.
- Launch-time profile derivation from Gazebo SDF.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from .config import StereoCameraConfig


@dataclass(frozen=True)
class StereoCameraPaths:
    """USD prim paths created for one stereo rig."""

    left_camera_path: str
    right_camera_path: str
    rig_root_path: str


def create_stereo_camera_prims(
    stage,
    stereo: StereoCameraConfig,
    parent_path: str,
) -> StereoCameraPaths:
    """Create calibrated left/right camera prims and return their paths."""
    from pxr import Gf, UsdGeom

    rig_root_path = f"{parent_path.rstrip('/')}/table_stereo_rig"
    stage.DefinePrim(rig_root_path, "Xform")

    center_x, center_y, center_z = stereo.center_position
    half_baseline = 0.5 * stereo.baseline_m
    look_distance = max(stereo.look_distance_m, 0.05)
    look_direction = Gf.Vec3d(
        math.cos(stereo.pitch_down_rad) * math.cos(stereo.yaw_rad),
        math.cos(stereo.pitch_down_rad) * math.sin(stereo.yaw_rad),
        -math.sin(stereo.pitch_down_rad),
    )
    eye_local = Gf.Vec3d(center_x, center_y, center_z)
    target_local = eye_local + look_direction * look_distance

    rig_xform = UsdGeom.Xformable(stage.GetPrimAtPath(rig_root_path))
    rig_xform.ClearXformOpOrder()
    rig_transform = Gf.Matrix4d().SetLookAt(
        eye_local,
        target_local,
        Gf.Vec3d(0.0, 0.0, 1.0),
    ).GetInverse()
    rig_xform.AddTransformOp().Set(rig_transform)

    left_camera_path = f"{rig_root_path}/left_camera"
    right_camera_path = f"{rig_root_path}/right_camera"
    focus_distance = max(stereo.focus_distance_m, stereo.look_distance_m)
    camera_settings = (
        (left_camera_path, [-half_baseline, 0.0, stereo.camera_local_z_offset_m]),
        (right_camera_path, [half_baseline, 0.0, stereo.camera_local_z_offset_m]),
    )
    for camera_path, local_translation in camera_settings:
        camera = UsdGeom.Camera.Define(stage, camera_path)
        camera_xform = UsdGeom.Xformable(camera.GetPrim())
        camera_xform.ClearXformOpOrder()
        camera_xform.AddTranslateOp().Set(Gf.Vec3d(*local_translation))
        camera.GetProjectionAttr().Set("perspective")
        camera.GetFocalLengthAttr().Set(stereo.focal_length_mm)
        camera.GetHorizontalApertureAttr().Set(stereo.horizontal_aperture_mm)
        camera.GetVerticalApertureAttr().Set(stereo.vertical_aperture_mm)
        camera.GetFocusDistanceAttr().Set(focus_distance)
        camera.GetClippingRangeAttr().Set(Gf.Vec2f(stereo.near_clip_m, stereo.far_clip_m))

    return StereoCameraPaths(
        left_camera_path=left_camera_path,
        right_camera_path=right_camera_path,
        rig_root_path=rig_root_path,
    )


def create_stereo_visual_model(stage, rig_root_path: str, stereo: StereoCameraConfig) -> None:
    """Add a lightweight visible housing for GUI inspection of the rig."""
    from isaacsim.core.utils import prims as prims_utils
    from pxr import Gf, UsdGeom

    body_path = f"{rig_root_path}/body"
    if stage.GetPrimAtPath(body_path):
        return

    half_baseline = 0.5 * stereo.baseline_m
    left_lens_path = f"{rig_root_path}/left_lens"
    right_lens_path = f"{rig_root_path}/right_lens"
    prims_utils.create_prim(
        body_path,
        prim_type="Cube",
        translation=[0.0, 0.0, 0.065],
        scale=[0.10, 0.04, 0.035],
        attributes={"size": 1.0},
    )
    prims_utils.create_prim(
        left_lens_path,
        prim_type="Sphere",
        translation=[-half_baseline, 0.0, stereo.camera_local_z_offset_m + 0.015],
        scale=[0.014, 0.014, 0.014],
        attributes={"radius": 1.0},
    )
    prims_utils.create_prim(
        right_lens_path,
        prim_type="Sphere",
        translation=[half_baseline, 0.0, stereo.camera_local_z_offset_m + 0.015],
        scale=[0.014, 0.014, 0.014],
        attributes={"radius": 1.0},
    )

    display_colors = (
        (body_path, (0.2, 0.2, 0.2)),
        (left_lens_path, (0.05, 0.1, 0.18)),
        (right_lens_path, (0.05, 0.1, 0.18)),
    )
    for prim_path, color in display_colors:
        color_attribute = UsdGeom.Gprim(
            stage.GetPrimAtPath(prim_path)
        ).GetDisplayColorAttr()
        color_attribute.Set([Gf.Vec3f(*color)])
