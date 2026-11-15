#!/usr/bin/env python3
"""Responsibility: Generate a YOLO-format synthetic dataset for cube detection/segmentation in Isaac Sim."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import random
import shutil
import sys
import traceback
import xml.etree.ElementTree as ET
from dataclasses import dataclass


def _resolve_franka_motion_package_root() -> pathlib.Path:
    """Resolve sibling resources in this standalone source distribution."""
    package_root = pathlib.Path(__file__).resolve().parents[2] / "franka_motion"
    if not (package_root / "package.xml").is_file():
        raise FileNotFoundError("Keep training/ beside franka_motion/ in the source checkout")
    return package_root


def _prepare_isaac_environment(args: argparse.Namespace) -> None:
    """Use the shared bootstrap in the selected Isaac Python before any output writes."""
    package_root = _resolve_franka_motion_package_root()
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))
    from franka_motion_runtime.simulation.isaac.runtime.environment import (
        bootstrap_isaacsim_python,
        disable_warp,
        ensure_ros_bridge_environment,
        reexec_with_clean_env,
        resolve_isaac_root,
    )

    override = pathlib.Path(args.install_path) if args.install_path else None
    isaac_root = resolve_isaac_root(override)
    if not isaac_root.is_dir():
        raise FileNotFoundError(f"Isaac Sim not found: {isaac_root}")
    reexec_with_clean_env(isaac_root, args.ros_distro)
    bootstrap_isaacsim_python(isaac_root)
    ensure_ros_bridge_environment(isaac_root, args.ros_distro)
    disable_warp()


@dataclass(frozen=True)
class SplitPaths:
    """Hold YOLO output paths for a dataset split."""

    images_dir: pathlib.Path
    labels_dir: pathlib.Path


@dataclass(frozen=True)
class ImageTargetWindow:
    """Describe a normalized image area where the cube mask center should land."""

    name: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float


@dataclass(frozen=True)
class GazeboLightingProfile:
    """Lighting values mirrored from the Gazebo table world."""

    ambient: tuple[float, float, float]
    background: tuple[float, float, float]
    sun_direction: tuple[float, float, float]
    sun_intensity: float
    sun_diffuse: tuple[float, float, float]
    sun_specular: tuple[float, float, float]


def _load_camera_profile_from_sdf(camera_sdf_path: pathlib.Path) -> dict[str, float]:
    """Load the Gazebo stereo camera profile so Isaac SDG uses the same optics."""
    if not camera_sdf_path.is_file():
        raise FileNotFoundError(f"Camera SDF not found: {camera_sdf_path}")

    tree = ET.parse(camera_sdf_path)
    root = tree.getroot()
    left_camera = root.find(".//sensor[@name='table_stereo_left_camera']/camera")
    if left_camera is None:
        raise RuntimeError(f"Left stereo camera description not found in: {camera_sdf_path}")

    horizontal_fov = float(left_camera.findtext("horizontal_fov", default="1.5707963268"))
    width = int(left_camera.findtext("image/width", default="640"))
    height = int(left_camera.findtext("image/height", default="480"))
    near_clip = float(left_camera.findtext("clip/near", default="0.05"))
    far_clip = float(left_camera.findtext("clip/far", default="20.0"))

    horizontal_aperture_mm = 20.0
    focal_length_mm = horizontal_aperture_mm / (2.0 * math.tan(horizontal_fov * 0.5))
    vertical_aperture_mm = horizontal_aperture_mm * (float(height) / float(width))

    return {
        "horizontal_fov_rad": horizontal_fov,
        "width": width,
        "height": height,
        "near_clip_m": near_clip,
        "far_clip_m": far_clip,
        "horizontal_aperture_mm": horizontal_aperture_mm,
        "vertical_aperture_mm": vertical_aperture_mm,
        "focal_length_mm": focal_length_mm,
    }


def _parse_rgb_triplet(raw_value: str, fallback: tuple[float, float, float]) -> tuple[float, float, float]:
    """Parse SDF color vectors and ignore alpha if it is present."""
    try:
        values = [float(value) for value in raw_value.split()]
    except ValueError:
        return fallback
    if len(values) < 3:
        return fallback
    return (values[0], values[1], values[2])


def _load_gazebo_lighting_profile(world_sdf_path: pathlib.Path) -> GazeboLightingProfile:
    """Load Gazebo table-world lighting so synthetic data has a runtime-like base exposure."""
    fallback = GazeboLightingProfile(
        ambient=(0.4, 0.4, 0.4),
        background=(0.7, 0.7, 0.7),
        sun_direction=(-0.5, 0.1, -0.9),
        sun_intensity=1.0,
        sun_diffuse=(0.8, 0.8, 0.8),
        sun_specular=(0.2, 0.2, 0.2),
    )
    if not world_sdf_path.is_file():
        return fallback

    tree = ET.parse(world_sdf_path)
    root = tree.getroot()
    scene = root.find(".//scene")
    sun = root.find(".//light[@name='sun']")
    ambient = fallback.ambient
    background = fallback.background
    sun_direction = fallback.sun_direction
    sun_intensity = fallback.sun_intensity
    sun_diffuse = fallback.sun_diffuse
    sun_specular = fallback.sun_specular

    if scene is not None:
        ambient = _parse_rgb_triplet(scene.findtext("ambient", ""), ambient)
        background = _parse_rgb_triplet(scene.findtext("background", ""), background)
    if sun is not None:
        sun_direction = _parse_rgb_triplet(sun.findtext("direction", ""), sun_direction)
        sun_diffuse = _parse_rgb_triplet(sun.findtext("diffuse", ""), sun_diffuse)
        sun_specular = _parse_rgb_triplet(sun.findtext("specular", ""), sun_specular)
        try:
            sun_intensity = float(sun.findtext("intensity", default=str(sun_intensity)))
        except ValueError:
            sun_intensity = fallback.sun_intensity

    return GazeboLightingProfile(
        ambient=ambient,
        background=background,
        sun_direction=sun_direction,
        sun_intensity=sun_intensity,
        sun_diffuse=sun_diffuse,
        sun_specular=sun_specular,
    )


class CubeSyntheticDatasetGenerator:
    """Build a lightweight synthetic dataset tailored for the Franka table setup."""

    _CAMERA_POS_MIN = (0.78, -0.28, 0.18)
    _CAMERA_POS_MAX = (1.10, 0.28, 0.40)
    _LOOK_AT_MIN = (0.25, -0.10, 0.01)
    _LOOK_AT_MAX = (0.55, 0.10, 0.08)
    _CUBE_POS_MIN = (0.24, -0.18, 0.03)
    _CUBE_POS_MAX = (0.58, 0.18, 0.03)
    _DISTRACTOR_POS_MIN = (0.16, -0.24, 0.035)
    _DISTRACTOR_POS_MAX = (0.68, 0.24, 0.08)
    _ROBOT_POS_MIN = (-0.12, -0.28, 0.0)
    _ROBOT_POS_MAX = (0.22, 0.28, 0.0)
    _ROBOT_ARM_JOINTS = (
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    )
    _ROBOT_FINGER_JOINTS = ("panda_finger_joint1", "panda_finger_joint2")
    _ROBOT_JOINT_LIMITS_RAD = (
        (-2.8973, 2.8973),
        (-1.7628, 1.7628),
        (-2.8973, 2.8973),
        (-3.0718, 0.0175),
        (-2.8973, 2.8973),
        (-0.0175, 3.7525),
        (-2.8973, 2.8973),
    )
    _ROBOT_POSE_TEMPLATES_RAD = (
        (0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785),
        (-0.55, -1.10, 0.45, -2.30, -0.30, 1.95, 1.10),
        (0.65, -0.95, -0.55, -2.15, 0.50, 1.85, -0.80),
        (-1.05, -0.55, 0.90, -1.90, -0.70, 2.20, 0.35),
        (1.05, -0.65, -0.90, -1.85, 0.85, 2.05, -0.35),
        (-0.35, -1.45, -0.25, -2.65, 0.70, 2.55, 1.45),
        (0.35, -1.35, 0.25, -2.70, -0.70, 2.40, -1.45),
    )
    _LIGHT_POSITIONS = [
        (0.20, -0.45, 0.95),
        (0.40, 0.45, 0.90),
        (0.95, 0.00, 0.80),
    ]
    _DISTRACTOR_TYPES = ("Sphere", "Cylinder", "Cone")
    _MAX_POSITIVE_RETRIES = 12
    _EDGE_POSITIVE_RETRIES = 32

    def __init__(self, args: argparse.Namespace) -> None:
        self._args = args
        self._rng = random.Random(args.seed)
        self._package_share = _resolve_franka_motion_package_root()
        self._task = str(args.task)
        self._output_root = pathlib.Path(args.output_dir).expanduser().resolve()
        self._scene_usd = (
            pathlib.Path(args.scene_usd).expanduser().resolve()
            if args.scene_usd
            else self._package_share / "worlds" / "isaac" / "table_world_usd_clean.usd"
        )
        self._robot_usd = (
            pathlib.Path(args.robot_usd).expanduser().resolve()
            if args.robot_usd
            else self._package_share / "worlds" / "isaac" / "FrankaPanda" / "franka.usd"
        )
        self._camera_sdf = (
            pathlib.Path(args.camera_sdf).expanduser().resolve()
            if args.camera_sdf
            else self._package_share / "description" / "gazebo" / "table_camera.sdf"
        )
        self._gazebo_world_sdf = (
            pathlib.Path(args.gazebo_world_sdf).expanduser().resolve()
            if args.gazebo_world_sdf
            else self._package_share / "worlds" / "gazebo" / "table_world_camera.sdf"
        )
        self._camera_profile = _load_camera_profile_from_sdf(self._camera_sdf)
        self._gazebo_lighting = _load_gazebo_lighting_profile(self._gazebo_world_sdf)
        self._splits = {
            "train": SplitPaths(self._output_root / "images" / "train", self._output_root / "labels" / "train"),
            "val": SplitPaths(self._output_root / "images" / "val", self._output_root / "labels" / "val"),
        }
        self._simulation_app = None
        self._rep = None
        self._stage = None
        self._camera_transform_op = None
        self._rgb_annotator = None
        self._bbox_annotator = None
        self._segmentation_annotator = None
        self._cube_rig_prim = None
        self._cube_prim = None
        self._cube_gprim = None
        self._cube_imageable = None
        self._cube_translate_op = None
        self._cube_rotate_op = None
        self._cube_scale_op = None
        self._robot_rig_prim = None
        self._robot_prim = None
        self._robot_rig_translate_op = None
        self._robot_rig_rotate_op = None
        self._robot_articulation = None
        self._robot_arm_joint_indices = None
        self._robot_finger_joint_indices = None
        self._robot_joint_target_attrs = {}
        self._light_prims = []
        self._distractors = []
        self._image_width = int(args.width) if int(args.width) > 0 else int(self._camera_profile["width"])
        self._image_height = int(args.height) if int(args.height) > 0 else int(self._camera_profile["height"])
        self._image_target_windows = self._build_image_target_windows()
        self._next_image_target_index = 0
        self._last_camera_position = (1.0, 0.0, 1.35)
        self._last_cube_translation = (0.4, 0.0, 0.03)
        self._last_cube_size = 0.05
        self._last_cube_visible = True

    def run(self) -> None:
        """Generate train/val splits and emit a YOLO data.yaml file."""
        self._validate_source_assets()
        self._prepare_output_tree()
        print(f"Output dataset root: {self._output_root}", flush=True)
        print("Launching Isaac Sim for dataset generation", flush=True)
        self._launch_isaac()
        try:
            print("Preparing Isaac scene", flush=True)
            self._setup_scene()
            print("Isaac scene is ready, starting dataset generation", flush=True)
            self._generate_split("train", int(self._args.num_train))
            self._generate_split("val", int(self._args.num_val))
            self._write_data_yaml()
        finally:
            if self._simulation_app is not None:
                self._simulation_app.close()

    def _build_image_target_windows(self) -> list[ImageTargetWindow]:
        """Create balanced target zones from image center to all borders and corners."""
        if float(self._args.image_target_ratio) <= 0.0:
            return []

        centers = (0.14, 0.32, 0.50, 0.68, 0.86)
        tolerance = max(0.05, min(float(self._args.image_target_tolerance), 0.24))
        windows: list[ImageTargetWindow] = []
        for y_index, y_center in enumerate(centers):
            for x_index, x_center in enumerate(centers):
                windows.append(
                    ImageTargetWindow(
                        name=f"grid_{x_index}_{y_index}",
                        x_min=max(0.0, x_center - tolerance),
                        x_max=min(1.0, x_center + tolerance),
                        y_min=max(0.0, y_center - tolerance),
                        y_max=min(1.0, y_center + tolerance),
                    )
                )
        return windows

    def _prepare_output_tree(self) -> None:
        """Create or refresh the dataset directory structure."""
        if self._output_root.exists() and any(self._output_root.iterdir()):
            if not self._args.overwrite:
                raise FileExistsError(
                    f"Output directory is not empty: {self._output_root}. Use --overwrite to recreate it."
                )
            shutil.rmtree(self._output_root)

        for split_paths in self._splits.values():
            split_paths.images_dir.mkdir(parents=True, exist_ok=True)
            split_paths.labels_dir.mkdir(parents=True, exist_ok=True)

    def _validate_source_assets(self) -> None:
        """Reject missing scene inputs before replacing an existing output dataset."""
        if not self._scene_usd.is_file():
            raise FileNotFoundError(f"Scene USD not found: {self._scene_usd}")
        if not self._robot_usd.is_file():
            raise FileNotFoundError(f"Robot USD not found: {self._robot_usd}")

    def _launch_isaac(self) -> None:
        """Start Isaac Sim only after the process environment is cleaned up."""
        from isaacsim import SimulationApp

        self._simulation_app = SimulationApp({"headless": self._args.headless})

    def _setup_scene(self) -> None:
        """Load the table world, add the robot, camera, cube, distractors and annotators."""
        print("Setup scene: importing Isaac helpers", flush=True)
        import omni.replicator.core as rep
        import omni.usd
        from isaacsim.core.utils import prims as prims_utils
        from isaacsim.core.utils import stage as stage_utils
        from isaacsim.core.utils.extensions import enable_extension
        from isaacsim.core.utils.semantics import add_labels, remove_labels, upgrade_prim_semantics_to_labels

        print("Setup scene: enabling required extensions", flush=True)
        enable_extension("isaacsim.core.api")
        enable_extension("omni.replicator.core")
        self._simulation_app.update()

        print(f"Setup scene: opening stage {self._scene_usd}", flush=True)
        omni.usd.get_context().open_stage(str(self._scene_usd))
        self._stage = omni.usd.get_context().get_stage()
        if self._stage is None:
            raise RuntimeError("Failed to open Isaac stage for synthetic dataset generation")

        print("Setup scene: adding robot and semantic labels", flush=True)
        stage_utils.set_stage_up_axis("Z")
        prims_utils.create_prim("/World/FrankaRig", "Xform")
        stage_utils.add_reference_to_stage(str(self._robot_usd), "/World/FrankaRig/Franka")
        self._simulation_app.update()
        self._prepare_robot_root()
        self._prepare_robot_articulation()

        upgrade_prim_semantics_to_labels(self._stage.GetPseudoRoot(), include_descendants=True)
        remove_labels(self._stage.GetPseudoRoot(), include_descendants=True)

        prims_utils.create_prim("/World/SDG", "Xform")
        print("Setup scene: creating camera, lights, cube and distractors", flush=True)
        self._create_camera()
        print("Setup scene: camera created", flush=True)
        self._create_lights()
        print("Setup scene: lights created", flush=True)
        self._create_cube()
        print("Setup scene: cube created", flush=True)
        self._create_distractors()
        print("Setup scene: distractors created", flush=True)

        print("Setup scene: creating render product and annotators", flush=True)
        self._rep = rep
        self._rep.orchestrator.set_capture_on_play(False)
        render_product = self._rep.create.render_product(
            "/World/SDG/DataCamera", (self._image_width, self._image_height)
        )
        self._rgb_annotator = self._rep.AnnotatorRegistry.get_annotator("rgb")
        self._bbox_annotator = self._rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")
        self._rgb_annotator.attach(render_product)
        self._bbox_annotator.attach(render_product)
        if self._task == "segment":
            self._segmentation_annotator = self._rep.AnnotatorRegistry.get_annotator(
                "semantic_segmentation",
                init_params={"colorize": False},
            )
            self._segmentation_annotator.attach(render_product)

        add_labels(self._cube_prim, ["cube"])

        for _ in range(8):
            self._simulation_app.update()
        print("Setup scene: starting Replicator preview", flush=True)
        self._rep.orchestrator.preview()

    def _create_camera(self) -> None:
        """Create a perspective camera with the same base optics as the runtime stereo setup."""
        from pxr import Gf, UsdGeom

        camera_path = "/World/SDG/DataCamera"
        camera = UsdGeom.Camera.Define(self._stage, camera_path)
        camera.GetProjectionAttr().Set("perspective")
        camera.GetFocalLengthAttr().Set(float(self._camera_profile["focal_length_mm"]))
        camera.GetHorizontalApertureAttr().Set(float(self._camera_profile["horizontal_aperture_mm"]))
        camera.GetVerticalApertureAttr().Set(float(self._camera_profile["vertical_aperture_mm"]))
        camera.GetFocusDistanceAttr().Set(400.0)
        camera.GetClippingRangeAttr().Set(
            Gf.Vec2f(float(self._camera_profile["near_clip_m"]), float(self._camera_profile["far_clip_m"]))
        )

        xformable = UsdGeom.Xformable(camera.GetPrim())
        xformable.ClearXformOpOrder()
        self._camera_transform_op = xformable.AddTransformOp()

    def _prepare_robot_root(self) -> None:
        """Create reusable transform ops for the robot root so it can move in the frame."""
        from pxr import UsdGeom

        self._robot_rig_prim = self._stage.GetPrimAtPath("/World/FrankaRig")
        self._robot_prim = self._stage.GetPrimAtPath("/World/FrankaRig/Franka")
        if not self._robot_rig_prim.IsValid() or not self._robot_prim.IsValid():
            raise RuntimeError("Robot prim /World/FrankaRig/Franka was not created in the synthetic stage")

        robot_rig_xform = UsdGeom.Xformable(self._robot_rig_prim)
        robot_rig_xform.ClearXformOpOrder()
        self._robot_rig_translate_op = robot_rig_xform.AddTranslateOp()
        self._robot_rig_rotate_op = robot_rig_xform.AddRotateXYZOp()
        self._robot_rig_translate_op.Set((0.0, 0.0, 0.0))
        self._robot_rig_rotate_op.Set((0.0, 0.0, 0.0))

    def _prepare_robot_articulation(self) -> None:
        """Prepare fast Franka joint randomization for varied manipulator silhouettes."""
        robot_root_path = str(self._robot_prim.GetPath())
        self._robot_joint_target_attrs = self._collect_robot_joint_target_attrs()

        try:
            from isaacsim.core.simulation_manager import SimulationManager
            from isaacsim.core.prims import SingleArticulation

            SimulationManager.initialize_physics()
            if SimulationManager.get_physics_sim_view() is None:
                SimulationManager._create_simulation_view(None)

            self._robot_articulation = SingleArticulation(
                prim_path=robot_root_path,
                name="synthetic_franka",
                reset_xform_properties=False,
            )
            for _ in range(4):
                self._simulation_app.update()
            self._robot_articulation.initialize()
            dof_names = list(self._robot_articulation.dof_names)
            self._robot_arm_joint_indices = [
                dof_names.index(joint_name)
                for joint_name in self._ROBOT_ARM_JOINTS
                if joint_name in dof_names
            ]
            self._robot_finger_joint_indices = [
                dof_names.index(joint_name)
                for joint_name in self._ROBOT_FINGER_JOINTS
                if joint_name in dof_names
            ]
            if len(self._robot_arm_joint_indices) != len(self._ROBOT_ARM_JOINTS):
                raise RuntimeError(f"Unexpected Franka DOF names: {dof_names}")
            print(f"Robot articulation ready: {dof_names}", flush=True)
        except Exception as exc:
            # Fallback still varies drive targets in composed USD if tensor articulation is unavailable.
            print(f"Robot articulation unavailable, using USD joint target fallback: {exc}", flush=True)
            self._robot_articulation = None
            self._robot_arm_joint_indices = None
            self._robot_finger_joint_indices = None

    def _collect_robot_joint_target_attrs(self) -> dict[str, object]:
        """Cache USD drive target attrs for a lightweight fallback path."""
        attrs: dict[str, object] = {}
        robot_prefix = str(self._robot_prim.GetPath())
        for prim in self._stage.Traverse():
            prim_path = str(prim.GetPath())
            if not prim_path.startswith(robot_prefix):
                continue
            joint_name = prim.GetName()
            if joint_name in self._ROBOT_ARM_JOINTS:
                attr = prim.GetAttribute("drive:angular:physics:targetPosition")
            elif joint_name in self._ROBOT_FINGER_JOINTS:
                attr = prim.GetAttribute("drive:linear:physics:targetPosition")
            else:
                continue
            if attr:
                attrs[joint_name] = attr
        return attrs

    def _create_lights(self) -> None:
        """Add Gazebo-like base light plus fill lights for robust illumination randomization."""
        from pxr import Gf, UsdGeom, UsdLux

        dome = UsdLux.DomeLight.Define(self._stage, "/World/SDG/DomeLight")
        dome.CreateIntensityAttr(1800.0)
        dome.CreateColorAttr(Gf.Vec3f(*self._gazebo_lighting.ambient))
        self._light_prims.append(dome.GetPrim())

        # Runtime Gazebo uses a directional sun; keep it as the main lighting anchor.
        sun = UsdLux.DistantLight.Define(self._stage, "/World/SDG/GazeboSun")
        sun.CreateIntensityAttr(1200.0 * float(self._gazebo_lighting.sun_intensity))
        sun.CreateColorAttr(Gf.Vec3f(*self._gazebo_lighting.sun_diffuse))
        sun_xform = UsdGeom.Xformable(sun.GetPrim())
        sun_xform.ClearXformOpOrder()
        direction = Gf.Vec3d(*self._gazebo_lighting.sun_direction)
        if direction.GetLength() < 1.0e-6:
            direction = Gf.Vec3d(-0.5, 0.1, -0.9)
        direction.Normalize()
        sun_position = -10.0 * direction
        sun_xform.AddTransformOp().Set(
            Gf.Matrix4d().SetLookAt(sun_position, Gf.Vec3d(0.4, 0.0, 0.03), Gf.Vec3d(0.0, 0.0, 1.0)).GetInverse()
        )
        self._light_prims.append(sun.GetPrim())

        for index, position in enumerate(self._LIGHT_POSITIONS):
            light = UsdLux.SphereLight.Define(self._stage, f"/World/SDG/KeyLight_{index}")
            light.CreateRadiusAttr(0.03)
            light.CreateIntensityAttr(9000.0)
            light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
            light_xform = UsdGeom.Xformable(light.GetPrim())
            light_xform.ClearXformOpOrder()
            light_xform.AddTranslateOp().Set(Gf.Vec3d(*position))
            self._light_prims.append(light.GetPrim())

    def _create_cube(self) -> None:
        """Create the only labeled object in the dataset."""
        from pxr import UsdGeom

        cube_rig_path = "/World/SDG/CubeTargetRig"
        cube_path = f"{cube_rig_path}/CubeTarget"
        cube_rig = UsdGeom.Xform.Define(self._stage, cube_rig_path)
        cube = UsdGeom.Cube.Define(self._stage, cube_path)
        cube.CreateSizeAttr(1.0)
        self._cube_rig_prim = cube_rig.GetPrim()
        self._cube_prim = cube.GetPrim()
        self._cube_gprim = UsdGeom.Gprim(self._cube_prim)
        self._cube_imageable = UsdGeom.Imageable(self._cube_rig_prim)
        cube_xform = UsdGeom.Xformable(self._cube_rig_prim)
        cube_xform.ClearXformOpOrder()
        self._cube_translate_op = cube_xform.AddTranslateOp()
        self._cube_rotate_op = cube_xform.AddRotateXYZOp()
        self._cube_scale_op = cube_xform.AddScaleOp()
        self._cube_translate_op.Set((0.4, 0.0, 0.03))
        self._cube_rotate_op.Set((0.0, 0.0, 0.0))
        self._cube_scale_op.Set((0.05, 0.05, 0.05))

    def _create_distractors(self) -> None:
        """Pre-create a small pool of unlabeled geometric distractors."""
        from pxr import UsdGeom

        for index in range(int(self._args.max_distractors)):
            prim_type = self._DISTRACTOR_TYPES[index % len(self._DISTRACTOR_TYPES)]
            rig_path = f"/World/SDG/DistractorRig_{index}"
            prim_path = f"{rig_path}/Distractor_{index}"
            rig_prim = UsdGeom.Xform.Define(self._stage, rig_path).GetPrim()
            if prim_type == "Sphere":
                prim = UsdGeom.Sphere.Define(self._stage, prim_path).GetPrim()
                UsdGeom.Sphere(prim).CreateRadiusAttr(0.5)
            elif prim_type == "Cylinder":
                prim = UsdGeom.Cylinder.Define(self._stage, prim_path).GetPrim()
                UsdGeom.Cylinder(prim).CreateRadiusAttr(0.5)
                UsdGeom.Cylinder(prim).CreateHeightAttr(1.0)
            elif prim_type == "Cone":
                prim = UsdGeom.Cone.Define(self._stage, prim_path).GetPrim()
                UsdGeom.Cone(prim).CreateRadiusAttr(0.5)
                UsdGeom.Cone(prim).CreateHeightAttr(1.0)
            else:
                raise RuntimeError(f"Unsupported distractor type: {prim_type}")
            xformable = UsdGeom.Xformable(rig_prim)
            xformable.ClearXformOpOrder()
            translate_op = xformable.AddTranslateOp()
            rotate_op = xformable.AddRotateXYZOp()
            scale_op = xformable.AddScaleOp()
            translate_op.Set((2.0, 2.0, 2.0))
            rotate_op.Set((0.0, 0.0, 0.0))
            scale_op.Set((0.04, 0.04, 0.04))
            self._distractors.append(
                {
                    "rig_prim": rig_prim,
                    "prim": prim,
                    "imageable": UsdGeom.Imageable(rig_prim),
                    "gprim": UsdGeom.Gprim(prim),
                    "translate_op": translate_op,
                    "rotate_op": rotate_op,
                    "scale_op": scale_op,
                }
            )

    def _generate_split(self, split_name: str, num_samples: int) -> None:
        """Generate one dataset split and save images plus YOLO labels."""
        split_paths = self._splits[split_name]
        print(f"Generating split `{split_name}` with {num_samples} samples", flush=True)
        for index in range(num_samples):
            should_hide_cube = self._rng.random() < float(self._args.empty_frame_ratio)
            image_name = f"{split_name}_{index:06d}.png"
            image_path = split_paths.images_dir / image_name
            label_path = split_paths.labels_dir / f"{image_path.stem}.txt"

            if should_hide_cube:
                rgb_image, _ = self._render_sample(include_cube=False)
                self._save_rgb(rgb_image, image_path)
                label_path.write_text("", encoding="utf-8")
            else:
                rgb_image, label_lines = self._render_positive_sample(self._next_image_target_window())
                self._save_rgb(rgb_image, image_path)
                label_path.write_text("\n".join(label_lines) + "\n", encoding="utf-8")

            if (index + 1) % max(1, int(self._args.log_every)) == 0 or index == num_samples - 1:
                print(f"[{split_name}] generated {index + 1}/{num_samples}", flush=True)

    def _next_image_target_window(self) -> ImageTargetWindow | None:
        """Return the next target zone so positives are balanced across the image."""
        if not self._image_target_windows:
            return None
        if self._rng.random() > float(self._args.image_target_ratio):
            return None
        window = self._image_target_windows[self._next_image_target_index % len(self._image_target_windows)]
        self._next_image_target_index += 1
        return window

    def _render_positive_sample(self, target_window: ImageTargetWindow | None) -> tuple[object, list[str]]:
        """Keep randomizing until the cube is visible enough for a useful positive label."""
        require_edge_sample = self._target_touches_image_border(target_window)
        if target_window is None:
            require_edge_sample = self._rng.random() < float(self._args.edge_bias_ratio)
        max_retries = (
            self._EDGE_POSITIVE_RETRIES
            if require_edge_sample or target_window is not None
            else self._MAX_POSITIVE_RETRIES
        )
        fallback_sample: tuple[object, list[str]] | None = None
        for _ in range(max_retries):
            rgb_image, label_lines = self._render_sample(include_cube=True, edge_target=require_edge_sample)
            if label_lines:
                if fallback_sample is None:
                    fallback_sample = (rgb_image, label_lines)
                if target_window is not None and not self._labels_match_image_target(label_lines, target_window):
                    continue
                if require_edge_sample and not self._labels_touch_edge_band(label_lines):
                    continue
                return rgb_image, label_lines

        if fallback_sample is not None:
            return fallback_sample

        raise RuntimeError(
            "Could not generate a visible cube sample after multiple retries. "
            "Reduce camera randomization or lower the visibility threshold."
        )

    def _render_sample(self, include_cube: bool, edge_target: bool = False) -> tuple[object, list[str]]:
        """Randomize the scene once, render it and convert annotations to YOLO labels."""
        self._set_camera_pose()
        self._randomize_lights()
        self._randomize_robot()
        self._randomize_cube(include_cube=include_cube, edge_target=edge_target)
        self._randomize_distractors()

        self._rep.orchestrator.step(rt_subframes=int(self._args.rt_subframes))
        rgb_image = self._rgb_annotator.get_data(device="cpu")
        if include_cube and self._task == "segment":
            segmentation_data = self._segmentation_annotator.get_data(device="cpu")
            label_lines = self._segmentation_to_yolo_lines(segmentation_data)
        else:
            bbox_data = self._bbox_annotator.get_data(device="cpu")
            label_lines = self._bbox_to_yolo_lines(bbox_data) if include_cube else []
        return rgb_image, label_lines

    def _randomize_robot(self) -> None:
        """Keep the robot visible while varying base and joint poses."""
        from pxr import UsdGeom

        robot_imageable = UsdGeom.Imageable(self._robot_rig_prim)
        if float(self._args.robot_hide_ratio) > 0.0 and self._rng.random() < float(self._args.robot_hide_ratio):
            robot_imageable.MakeInvisible()
            return

        robot_imageable.MakeVisible()
        self._robot_rig_translate_op.Set(
            (
                self._rng.uniform(self._ROBOT_POS_MIN[0], self._ROBOT_POS_MAX[0]),
                self._rng.uniform(self._ROBOT_POS_MIN[1], self._ROBOT_POS_MAX[1]),
                self._ROBOT_POS_MIN[2],
            )
        )
        self._robot_rig_rotate_op.Set((0.0, 0.0, self._rng.uniform(-35.0, 35.0)))
        self._randomize_robot_joints()

    def _randomize_robot_joints(self) -> None:
        """Sample reachable-looking arm configurations and random gripper openings."""
        arm_positions = self._sample_robot_arm_positions()
        finger_position = self._rng.uniform(0.0, 0.04)
        finger_positions = [finger_position, finger_position]

        if self._robot_articulation is not None and self._robot_arm_joint_indices is not None:
            try:
                backend_utils = self._robot_articulation._backend_utils
                device = self._robot_articulation._device

                self._robot_articulation.set_joint_positions(
                    backend_utils.convert(arm_positions, device, dtype="float32"),
                    joint_indices=backend_utils.convert(
                        self._robot_arm_joint_indices,
                        device,
                        dtype="int32",
                    ),
                )
                if self._robot_finger_joint_indices:
                    self._robot_articulation.set_joint_positions(
                        backend_utils.convert(finger_positions, device, dtype="float32"),
                        joint_indices=backend_utils.convert(
                            self._robot_finger_joint_indices,
                            device,
                            dtype="int32",
                        ),
                    )
                return
            except Exception as exc:
                print(f"Robot tensor pose update failed, falling back to USD drive targets: {exc}", flush=True)
                self._robot_articulation = None

        for joint_name, joint_value in zip(self._ROBOT_ARM_JOINTS, arm_positions, strict=True):
            attr = self._robot_joint_target_attrs.get(joint_name)
            if attr:
                attr.Set(math.degrees(joint_value))
        for joint_name, joint_value in zip(self._ROBOT_FINGER_JOINTS, finger_positions, strict=True):
            attr = self._robot_joint_target_attrs.get(joint_name)
            if attr:
                attr.Set(joint_value)

    def _sample_robot_arm_positions(self) -> list[float]:
        """Blend template Franka poses with bounded noise inside joint limits."""
        template = self._rng.choice(self._ROBOT_POSE_TEMPLATES_RAD)
        positions: list[float] = []
        for value, (lower, upper) in zip(template, self._ROBOT_JOINT_LIMITS_RAD, strict=True):
            noisy_value = value + self._rng.uniform(-0.45, 0.45)
            positions.append(max(lower + 0.04, min(upper - 0.04, noisy_value)))
        return positions

    def _set_camera_pose(self) -> None:
        """Sample a practical top-front view around the work area and look back at the table."""
        from pxr import Gf

        position = Gf.Vec3d(
            self._rng.uniform(self._CAMERA_POS_MIN[0], self._CAMERA_POS_MAX[0]),
            self._rng.uniform(self._CAMERA_POS_MIN[1], self._CAMERA_POS_MAX[1]),
            self._rng.uniform(self._CAMERA_POS_MIN[2], self._CAMERA_POS_MAX[2]),
        )
        self._last_camera_position = (float(position[0]), float(position[1]), float(position[2]))
        look_at = Gf.Vec3d(
            self._rng.uniform(self._LOOK_AT_MIN[0], self._LOOK_AT_MAX[0]),
            self._rng.uniform(self._LOOK_AT_MIN[1], self._LOOK_AT_MAX[1]),
            self._rng.uniform(self._LOOK_AT_MIN[2], self._LOOK_AT_MAX[2]),
        )
        up_axis = Gf.Vec3d(0.0, 0.0, 1.0)
        self._camera_transform_op.Set(Gf.Matrix4d().SetLookAt(position, look_at, up_axis).GetInverse())

    def _randomize_lights(self) -> None:
        """Vary light intensity and color around Gazebo/Isaac-like base profiles."""
        from pxr import Gf, UsdLux

        profile = self._rng.choices(
            ("gazebo_anchor", "isaac_bright", "dim", "overexposed", "warm", "cool"),
            weights=(0.34, 0.18, 0.14, 0.10, 0.12, 0.12),
            k=1,
        )[0]

        for light_prim in self._light_prims:
            if light_prim.GetTypeName() == "DomeLight":
                dome = UsdLux.DomeLight(light_prim)
                if profile == "gazebo_anchor":
                    intensity = self._rng.uniform(1400.0, 2600.0)
                    base_color = self._gazebo_lighting.ambient
                elif profile == "isaac_bright":
                    intensity = self._rng.uniform(2400.0, 5200.0)
                    base_color = self._gazebo_lighting.background
                elif profile == "dim":
                    intensity = self._rng.uniform(500.0, 1200.0)
                    base_color = (0.72, 0.76, 0.82)
                elif profile == "overexposed":
                    intensity = self._rng.uniform(5200.0, 9000.0)
                    base_color = (1.0, 1.0, 1.0)
                elif profile == "warm":
                    intensity = self._rng.uniform(1800.0, 4600.0)
                    base_color = (1.0, 0.88, 0.72)
                else:
                    intensity = self._rng.uniform(1800.0, 4600.0)
                    base_color = (0.72, 0.86, 1.0)
                dome.GetIntensityAttr().Set(intensity)
                dome.GetColorAttr().Set(
                    Gf.Vec3f(
                        max(0.0, min(1.0, base_color[0] * self._rng.uniform(0.88, 1.12))),
                        max(0.0, min(1.0, base_color[1] * self._rng.uniform(0.88, 1.12))),
                        max(0.0, min(1.0, base_color[2] * self._rng.uniform(0.88, 1.12))),
                    )
                )
                continue

            if light_prim.GetTypeName() == "DistantLight":
                sun = UsdLux.DistantLight(light_prim)
                if profile == "gazebo_anchor":
                    intensity = self._rng.uniform(700.0, 1800.0)
                    color = self._gazebo_lighting.sun_diffuse
                elif profile == "dim":
                    intensity = self._rng.uniform(120.0, 700.0)
                    color = (0.78, 0.82, 0.90)
                elif profile == "overexposed":
                    intensity = self._rng.uniform(1800.0, 3800.0)
                    color = (1.0, 1.0, 1.0)
                elif profile == "warm":
                    intensity = self._rng.uniform(900.0, 2600.0)
                    color = (1.0, 0.84, 0.68)
                elif profile == "cool":
                    intensity = self._rng.uniform(900.0, 2600.0)
                    color = (0.72, 0.86, 1.0)
                else:
                    intensity = self._rng.uniform(1000.0, 2800.0)
                    color = (0.92, 0.94, 1.0)
                sun.GetIntensityAttr().Set(intensity)
                sun.GetColorAttr().Set(Gf.Vec3f(*color))
                continue

            sphere = UsdLux.SphereLight(light_prim)
            if profile == "gazebo_anchor":
                intensity = self._rng.uniform(2500.0, 9500.0)
            elif profile == "dim":
                intensity = self._rng.uniform(300.0, 2600.0)
            elif profile == "overexposed":
                intensity = self._rng.uniform(9000.0, 22000.0)
            else:
                intensity = self._rng.uniform(4500.0, 15000.0)
            sphere.GetIntensityAttr().Set(intensity)
            sphere.GetColorAttr().Set(
                Gf.Vec3f(
                    self._rng.uniform(0.72, 1.0),
                    self._rng.uniform(0.72, 1.0),
                    self._rng.uniform(0.72, 1.0),
                )
            )

    def _randomize_cube(self, include_cube: bool, edge_target: bool = False) -> None:
        """Move the target cube inside the workspace and randomize its appearance."""
        from pxr import Gf

        if include_cube:
            self._cube_imageable.MakeVisible()
        else:
            self._cube_imageable.MakeInvisible()

        cube_size = self._rng.uniform(float(self._args.cube_size_min_m), float(self._args.cube_size_max_m))
        cube_translation = self._sample_cube_translation(cube_size=cube_size, edge_target=edge_target)
        self._last_cube_visible = include_cube
        self._last_cube_translation = cube_translation
        self._last_cube_size = cube_size
        self._cube_translate_op.Set(cube_translation)
        self._cube_rotate_op.Set(
            (
                self._rng.uniform(-22.0, 22.0),
                self._rng.uniform(-22.0, 22.0),
                self._rng.uniform(0.0, 360.0),
            )
        )
        self._cube_scale_op.Set((cube_size, cube_size, cube_size))
        self._cube_gprim.GetDisplayColorAttr().Set(
            [
                Gf.Vec3f(
                    self._rng.uniform(0.10, 0.95),
                    self._rng.uniform(0.10, 0.95),
                    self._rng.uniform(0.10, 0.95),
                )
            ]
        )

    def _sample_cube_translation(self, cube_size: float, edge_target: bool) -> tuple[float, float, float]:
        """Bias a subset of cube placements toward workspace edges so they land near image borders."""
        x_min, y_min, z_min = self._CUBE_POS_MIN
        x_max, y_max, _ = self._CUBE_POS_MAX
        z_value = max(z_min, cube_size * 0.5 + 0.002)
        if not edge_target:
            return (
                self._rng.uniform(x_min, x_max),
                self._rng.uniform(y_min, y_max),
                z_value,
            )

        edge_ratio = max(0.05, min(float(self._args.workspace_edge_ratio), 0.45))
        x_span = x_max - x_min
        y_span = y_max - y_min
        choose_x_edge = self._rng.random() < 0.5
        choose_low_edge = self._rng.random() < 0.5

        if choose_x_edge:
            if choose_low_edge:
                x_value = self._rng.uniform(x_min, x_min + x_span * edge_ratio)
            else:
                x_value = self._rng.uniform(x_max - x_span * edge_ratio, x_max)
            y_value = self._rng.uniform(y_min, y_max)
        else:
            x_value = self._rng.uniform(x_min, x_max)
            if choose_low_edge:
                y_value = self._rng.uniform(y_min, y_min + y_span * edge_ratio)
            else:
                y_value = self._rng.uniform(y_max - y_span * edge_ratio, y_max)

        return (x_value, y_value, z_value)

    def _randomize_distractors(self) -> None:
        """Scatter unlabeled primitives and sometimes place them as partial cube occluders."""
        from pxr import Gf

        visible_count = self._rng.randint(0, len(self._distractors))
        occluder_count = 0
        if self._last_cube_visible and self._rng.random() < float(self._args.occluder_ratio):
            max_occluders = max(0, min(int(self._args.max_occluders), len(self._distractors)))
            if max_occluders > 0:
                occluder_count = self._rng.randint(1, max_occluders)
                visible_count = max(visible_count, occluder_count)

        for index, distractor in enumerate(self._distractors):
            if index < visible_count:
                distractor["imageable"].MakeVisible()
                if index < occluder_count:
                    size, position = self._sample_occluder_pose()
                else:
                    size = self._rng.uniform(0.025, 0.08)
                    position = (
                        self._rng.uniform(self._DISTRACTOR_POS_MIN[0], self._DISTRACTOR_POS_MAX[0]),
                        self._rng.uniform(self._DISTRACTOR_POS_MIN[1], self._DISTRACTOR_POS_MAX[1]),
                        self._rng.uniform(self._DISTRACTOR_POS_MIN[2], self._DISTRACTOR_POS_MAX[2]),
                    )
                distractor["translate_op"].Set(position)
                distractor["rotate_op"].Set(
                    (
                        self._rng.uniform(0.0, 360.0),
                        self._rng.uniform(0.0, 360.0),
                        self._rng.uniform(0.0, 360.0),
                    )
                )
                distractor["scale_op"].Set((size, size, size))
                distractor["gprim"].GetDisplayColorAttr().Set(
                    [
                        Gf.Vec3f(
                            self._rng.uniform(0.05, 0.95),
                            self._rng.uniform(0.05, 0.95),
                            self._rng.uniform(0.05, 0.95),
                        )
                    ]
                )
            else:
                distractor["imageable"].MakeInvisible()

    def _sample_occluder_pose(self) -> tuple[float, tuple[float, float, float]]:
        """Place a distractor between camera and cube to create realistic partial masks."""
        cube_x, cube_y, _cube_z = self._last_cube_translation
        camera_x, camera_y, _camera_z = self._last_camera_position
        view_x = camera_x - cube_x
        view_y = camera_y - cube_y
        view_norm = math.hypot(view_x, view_y)
        if view_norm < 1.0e-6:
            view_x, view_y = 1.0, 0.0
            view_norm = 1.0
        view_x /= view_norm
        view_y /= view_norm
        tangent_x = -view_y
        tangent_y = view_x

        size = self._rng.uniform(self._last_cube_size * 0.45, self._last_cube_size * 1.35)
        forward_offset = self._rng.uniform(self._last_cube_size * 0.15, self._last_cube_size * 0.85)
        side_offset = self._rng.uniform(-self._last_cube_size * 0.70, self._last_cube_size * 0.70)
        x_value = cube_x + view_x * forward_offset + tangent_x * side_offset
        y_value = cube_y + view_y * forward_offset + tangent_y * side_offset
        x_value = max(self._DISTRACTOR_POS_MIN[0], min(self._DISTRACTOR_POS_MAX[0], x_value))
        y_value = max(self._DISTRACTOR_POS_MIN[1], min(self._DISTRACTOR_POS_MAX[1], y_value))
        z_value = max(self._DISTRACTOR_POS_MIN[2], size * 0.5 + 0.002)
        return size, (x_value, y_value, z_value)

    def _bbox_to_yolo_lines(self, bbox_payload: dict) -> list[str]:
        """Convert Isaac tight bbox output into YOLO normalized labels for the `cube` class."""
        gt_bbox = bbox_payload.get("data")
        bbox_info = bbox_payload.get("info", {})
        id_to_labels = bbox_info.get("idToLabels", {})
        if gt_bbox is None:
            return []

        lines = []
        min_area_ratio = float(self._args.min_box_area_ratio)
        for bbox in gt_bbox:
            semantic_id = int(bbox["semanticId"])
            labels_info = id_to_labels.get(str(semantic_id), {})
            class_name = labels_info.get("class", "") if isinstance(labels_info, dict) else str(labels_info)
            if class_name != "cube":
                continue

            x_min = max(0.0, float(bbox["x_min"]))
            y_min = max(0.0, float(bbox["y_min"]))
            x_max = min(float(self._image_width), float(bbox["x_max"]))
            y_max = min(float(self._image_height), float(bbox["y_max"]))
            width_px = x_max - x_min
            height_px = y_max - y_min
            if width_px <= 1.0 or height_px <= 1.0:
                continue

            area_ratio = (width_px * height_px) / float(self._image_width * self._image_height)
            if area_ratio < min_area_ratio:
                continue

            x_center = ((x_min + x_max) * 0.5) / float(self._image_width)
            y_center = ((y_min + y_max) * 0.5) / float(self._image_height)
            width_norm = width_px / float(self._image_width)
            height_norm = height_px / float(self._image_height)
            lines.append(f"0 {x_center:.6f} {y_center:.6f} {width_norm:.6f} {height_norm:.6f}")

        return lines

    @staticmethod
    def _id_labels_match(labels_info: object, expected_class: str) -> bool:
        """Check Replicator idToLabels values across old and new payload shapes."""
        if isinstance(labels_info, dict):
            class_value = str(labels_info.get("class", ""))
            if class_value == expected_class:
                return True
            return expected_class in {str(value) for value in labels_info.values()}
        return str(labels_info) == expected_class

    def _semantic_ids_for_class(self, segmentation_payload: dict, expected_class: str) -> list[int]:
        """Extract semantic ids belonging to the target class from a Replicator payload."""
        info = segmentation_payload.get("info", {})
        id_to_labels = info.get("idToLabels", {}) if isinstance(info, dict) else {}
        semantic_ids: list[int] = []
        for semantic_id, labels_info in id_to_labels.items():
            if self._id_labels_match(labels_info, expected_class):
                semantic_ids.append(int(semantic_id))
        return semantic_ids

    def _segmentation_to_yolo_lines(self, segmentation_payload: dict) -> list[str]:
        """Convert Isaac semantic segmentation output into YOLO polygon labels for the cube."""
        import cv2
        import numpy as np

        segmentation_data = segmentation_payload.get("data") if isinstance(segmentation_payload, dict) else None
        if segmentation_data is None:
            return []

        segmentation_array = np.asarray(segmentation_data)
        if segmentation_array.ndim != 2:
            raise RuntimeError(
                "semantic_segmentation annotator returned colorized data; expected a single-channel id map"
            )

        semantic_ids = self._semantic_ids_for_class(segmentation_payload, "cube")
        if semantic_ids:
            mask = np.isin(segmentation_array, semantic_ids)
        else:
            # В этой сцене только куб получает semantic label, поэтому non-zero fallback остается безопасным.
            mask = segmentation_array > 0

        mask_area_px = int(np.count_nonzero(mask))
        min_area_ratio = float(self._args.min_box_area_ratio)
        if mask_area_px < int(self._args.seg_min_component_area_px):
            return []
        if mask_area_px / float(self._image_width * self._image_height) < min_area_ratio:
            return []

        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return []

        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < float(self._args.seg_min_component_area_px):
            return []

        contour = self._approximate_contour(contour)
        points = contour.reshape(-1, 2)
        if len(points) < 3:
            return []

        coords: list[str] = []
        for x_px, y_px in points:
            x_norm = max(0.0, min(1.0, float(x_px) / float(self._image_width)))
            y_norm = max(0.0, min(1.0, float(y_px) / float(self._image_height)))
            coords.append(f"{x_norm:.6f}")
            coords.append(f"{y_norm:.6f}")

        return [f"0 {' '.join(coords)}"]

    def _approximate_contour(self, contour):
        """Simplify a mask contour to a compact polygon while preserving cube edges."""
        import cv2

        max_points = max(3, int(self._args.seg_max_points))
        perimeter = cv2.arcLength(contour, True)
        epsilon_ratio = max(0.0, float(self._args.seg_approx_epsilon_ratio))
        epsilon = max(0.5, epsilon_ratio * perimeter)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        while len(approx) > max_points:
            epsilon *= 1.5
            approx = cv2.approxPolyDP(contour, epsilon, True)

        if len(approx) < 3:
            approx = cv2.convexHull(contour)
        return approx

    @staticmethod
    def _target_touches_image_border(target_window: ImageTargetWindow | None) -> bool:
        """Check whether a target window belongs to the edge/corner curriculum."""
        if target_window is None:
            return False
        return (
            target_window.x_min <= 0.02
            or target_window.y_min <= 0.02
            or target_window.x_max >= 0.98
            or target_window.y_max >= 0.98
        )

    @staticmethod
    def _label_center_xy(label_line: str) -> tuple[float, float] | None:
        """Extract a normalized bbox center from detect or segment YOLO labels."""
        parts = label_line.split()
        if len(parts) == 5:
            return float(parts[1]), float(parts[2])

        coords = [float(value) for value in parts[1:]]
        xs = coords[0::2]
        ys = coords[1::2]
        if len(xs) < 3 or len(ys) < 3:
            return None
        return (min(xs) + max(xs)) * 0.5, (min(ys) + max(ys)) * 0.5

    def _labels_match_image_target(
        self,
        label_lines: list[str],
        target_window: ImageTargetWindow,
    ) -> bool:
        """Accept labels whose visible mask center lands in the requested image area."""
        for line in label_lines:
            center = self._label_center_xy(line)
            if center is None:
                continue
            x_center, y_center = center
            if target_window.x_min <= x_center <= target_window.x_max:
                if target_window.y_min <= y_center <= target_window.y_max:
                    return True
        return False

    def _labels_touch_edge_band(self, label_lines: list[str]) -> bool:
        """Accept a positive sample when the visible cube is close enough to the image border."""
        edge_band_ratio = max(0.05, min(float(self._args.edge_band_ratio), 0.35))
        for line in label_lines:
            parts = line.split()
            if len(parts) == 5:
                x_center = float(parts[1])
                y_center = float(parts[2])
                width_norm = float(parts[3])
                height_norm = float(parts[4])
                x_min = x_center - width_norm * 0.5
                x_max = x_center + width_norm * 0.5
                y_min = y_center - height_norm * 0.5
                y_max = y_center + height_norm * 0.5
            else:
                coords = [float(value) for value in parts[1:]]
                xs = coords[0::2]
                ys = coords[1::2]
                if len(xs) < 3 or len(ys) < 3:
                    continue
                x_min = min(xs)
                x_max = max(xs)
                y_min = min(ys)
                y_max = max(ys)
            if x_min <= edge_band_ratio or x_max >= 1.0 - edge_band_ratio:
                return True
            if y_min <= edge_band_ratio or y_max >= 1.0 - edge_band_ratio:
                return True
        return False

    def _save_rgb(self, rgb_image, image_path: pathlib.Path) -> None:
        """Persist the RGB frame as a PNG image."""
        from PIL import Image
        import numpy as np

        rgb_array = np.asarray(rgb_image[..., :3], dtype=np.uint8)
        if not bool(self._args.disable_photometric_augmentation):
            rgb_array = self._augment_rgb_photometrics(rgb_array)
        Image.fromarray(rgb_array).save(image_path)

    def _augment_rgb_photometrics(self, rgb_array):
        """Normalize unusably dark renders and add exposure/color-temperature variety."""
        import cv2
        import numpy as np

        image = rgb_array.astype(np.float32)
        gray = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2GRAY).astype(np.float32)
        current_luma = max(1.0, float(gray.mean()))
        target_min = max(25.0, min(float(self._args.photometric_luma_min), 220.0))
        target_max = max(target_min + 1.0, min(float(self._args.photometric_luma_max), 245.0))
        target_luma = self._rng.uniform(target_min, target_max)

        # Most samples are exposure-normalized into a Gazebo-like useful range; some keep
        # their natural render exposure so the model still sees genuinely dim/bright frames.
        if current_luma < target_min or current_luma > target_max or self._rng.random() < 0.82:
            image *= target_luma / current_luma

        contrast = self._rng.uniform(0.72, 1.30)
        image = (image - 127.5) * contrast + 127.5

        gamma = self._rng.uniform(0.78, 1.34)
        image = 255.0 * np.power(np.clip(image, 0.0, 255.0) / 255.0, gamma)

        # RGB channel gains emulate warm/cool lights and camera white-balance drift.
        gains = np.array(
            [
                self._rng.uniform(0.86, 1.18),
                self._rng.uniform(0.88, 1.12),
                self._rng.uniform(0.86, 1.18),
            ],
            dtype=np.float32,
        )
        image *= gains.reshape(1, 1, 3)

        if self._rng.random() < 0.45:
            noise_sigma = self._rng.uniform(0.5, 5.0)
            image += np.random.default_rng(self._rng.randint(0, 2**31 - 1)).normal(
                0.0,
                noise_sigma,
                image.shape,
            )

        if self._rng.random() < 0.18:
            image = cv2.GaussianBlur(image, (0, 0), self._rng.uniform(0.25, 0.9))

        return np.clip(image, 0.0, 255.0).astype(np.uint8)

    def _write_data_yaml(self) -> None:
        """Write the minimal YOLO dataset manifest next to the generated files."""
        yaml_path = self._output_root / "data.yaml"
        yaml_text = (
            f"path: {json.dumps(str(self._output_root))}\n"
            "train: images/train\n"
            "val: images/val\n"
            "names:\n"
            "  0: cube\n"
        )
        yaml_path.write_text(yaml_text, encoding="utf-8")
        print(f"Dataset manifest written to: {yaml_path}", flush=True)


def _build_arg_parser() -> argparse.ArgumentParser:
    """Define the user-facing CLI for dataset generation."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--install_path", default="", help="Optional resource-root override; default: selected Isaac Python")
    parser.add_argument("--scene_usd", default="")
    parser.add_argument("--robot_usd", default="")
    parser.add_argument("--camera_sdf", default="")
    parser.add_argument("--gazebo_world_sdf", default="")
    parser.add_argument("--task", choices=("detect", "segment"), default="detect")
    parser.add_argument("--output_dir", required=True, help="Explicit output dataset directory")
    parser.add_argument("--num_train", type=int, default=4000)
    parser.add_argument("--num_val", type=int, default=800)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--width", type=int, default=0)
    parser.add_argument("--height", type=int, default=0)
    parser.add_argument("--rt_subframes", type=int, default=4)
    parser.add_argument("--empty_frame_ratio", type=float, default=0.10)
    parser.add_argument("--robot_hide_ratio", type=float, default=0.0)
    parser.add_argument("--edge_bias_ratio", type=float, default=0.45)
    parser.add_argument("--edge_band_ratio", type=float, default=0.15)
    parser.add_argument("--workspace_edge_ratio", type=float, default=0.18)
    parser.add_argument("--image_target_ratio", type=float, default=0.90)
    parser.add_argument("--image_target_tolerance", type=float, default=0.16)
    parser.add_argument("--max_distractors", type=int, default=4)
    parser.add_argument("--occluder_ratio", type=float, default=0.35)
    parser.add_argument("--max_occluders", type=int, default=2)
    parser.add_argument("--min_box_area_ratio", type=float, default=0.002)
    parser.add_argument("--seg_approx_epsilon_ratio", type=float, default=0.01)
    parser.add_argument("--seg_max_points", type=int, default=32)
    parser.add_argument("--seg_min_component_area_px", type=int, default=12)
    parser.add_argument("--cube_size_min_m", type=float, default=0.04)
    parser.add_argument("--cube_size_max_m", type=float, default=0.06)
    parser.add_argument("--disable_photometric_augmentation", action="store_true")
    parser.add_argument("--photometric_luma_min", type=float, default=70.0)
    parser.add_argument("--photometric_luma_max", type=float, default=205.0)
    parser.add_argument("--log_every", type=int, default=50)
    parser.add_argument("--ros_distro", default="jazzy")
    return parser


def main() -> None:
    """Run the generator end-to-end."""
    args = _build_arg_parser().parse_args()
    _prepare_isaac_environment(args)
    generator = CubeSyntheticDatasetGenerator(args)
    try:
        generator.run()
    except Exception:
        traceback.print_exc()
        raise


if __name__ == "__main__":
    main()
