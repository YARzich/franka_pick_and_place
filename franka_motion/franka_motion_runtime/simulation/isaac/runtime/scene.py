"""
Isaac scene construction helpers for the Franka runtime.

Responsibility:
- Create the World, load robot assets and add optional scene/cube objects.
- Hide Isaac-specific imports behind runtime functions.

Does not own:
- CLI parsing, environment bootstrap or OmniGraph ROS bridge specs.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable

from .config import CubeConfig, IsaacRunnerConfig


# Mirror Isaac's Franka-like per-joint position-drive profile while retaining
# the articulation's authored effort limits.
_POSITION_DRIVE_TYPE = 1
_URDF_DRIVE_STIFFNESS = 1_000.0
_URDF_DRIVE_DAMPING = 100.0
_FRANKA_FINGER_DRIVE_STIFFNESS = 100.0
_FRANKA_FINGER_DRIVE_DAMPING = 20.0
_FRANKA_SOLVER_ITERATIONS = 64
_FRANKA_ARM_DOF_NAMES = tuple(f"panda_joint{index}" for index in range(1, 8))
_FRANKA_ARM_DRIVE_STIFFNESSES = (
    60_000.0,
    60_000.0,
    60_000.0,
    60_000.0,
    25_000.0,
    15_000.0,
    5_000.0,
)
_FRANKA_ARM_DRIVE_DAMPINGS = (3_000.0,) * len(_FRANKA_ARM_DOF_NAMES)
_FRANKA_DOF_NAMES = _FRANKA_ARM_DOF_NAMES + (
    "panda_finger_joint1",
    "panda_finger_joint2",
)


def create_world(*, no_default_ground: bool):
    """Create the Isaac world and optionally add its default ground plane."""
    from isaacsim.core.api import World
    from isaacsim.core.utils import stage as stage_utils

    world = World(stage_units_in_meters=1.0)
    if not no_default_ground:
        world.scene.add_default_ground_plane()
    stage_utils.set_stage_up_axis("Z")
    return world


def load_robot_articulation(stage, config: IsaacRunnerConfig) -> str:
    """Load a USD robot or import URDF and return its articulation path."""
    from isaacsim.core.utils import prims as prims_utils
    from isaacsim.core.utils import stage as stage_utils

    target_path = config.scene_prim.rstrip("/") or "/World/Franka"
    _ensure_parent_prim(stage, prims_utils, target_path)

    if config.robot_usd is not None:
        prims_utils.create_prim(target_path, "Xform")
        stage_utils.add_reference_to_stage(str(config.robot_usd), target_path)
        return target_path
    if config.urdf_path is None:
        raise RuntimeError("Robot asset path is missing after validation.")
    return _import_urdf_robot(config.urdf_path, target_path, config.fix_base)


def add_scene_reference(scene_usd: Path | None) -> None:
    """Add an optional environment USD reference under ``/World/Scene``."""
    if scene_usd is None:
        return
    from isaacsim.core.utils import stage as stage_utils

    stage_utils.add_reference_to_stage(str(scene_usd), "/World/Scene")


def add_pick_cube(world, cube_config: CubeConfig):
    """Add and return the configured dynamic pick cube when enabled."""
    if not cube_config.add_cube:
        return None
    from isaacsim.core.api.materials.physics_material import PhysicsMaterial
    from isaacsim.core.api.objects import DynamicCuboid
    import numpy as np

    cube_material = PhysicsMaterial(
        prim_path="/World/Physics_Materials/pick_cube_material",
        static_friction=cube_config.static_friction,
        dynamic_friction=cube_config.dynamic_friction,
        restitution=cube_config.restitution,
    )
    cube = DynamicCuboid(
        prim_path=cube_config.prim,
        name="pick_cube",
        position=np.array(cube_config.position, dtype=float),
        size=cube_config.size,
        color=np.array([0.9, 0.2, 0.2], dtype=float),
        mass=0.02,
        physics_material=cube_material,
    )
    return world.scene.add(cube)


def add_franka_robot(world, articulation_path: str):
    """Wrap the loaded articulation in Isaac's generic robot class."""
    from isaacsim.core.api.robots import Robot

    return world.scene.add(Robot(prim_path=articulation_path, name="franka"))


def initialize_robot(
    world,
    robot,
    *,
    max_attempts: int = 240,
    should_stop: Callable[[], bool] | None = None,
) -> bool:
    """Reset and step until initialized or startup cancellation is requested."""
    world.reset()
    for _ in range(max_attempts):
        if should_stop is not None and should_stop():
            return False
        robot.initialize()
        if robot.handles_initialized and robot.dof_names:
            _configure_franka_articulation_drives(robot)
            return True
        world.step(render=False)
    return False


def _ensure_parent_prim(stage, prims_utils, target_path: str) -> None:
    target_parent = str(Path(target_path).parent)
    if target_parent and target_parent != "/" and not stage.GetPrimAtPath(target_parent):
        prims_utils.create_prim(target_parent, "Xform")


def _import_urdf_robot(urdf_path: Path, target_path: str, fix_base: bool) -> str:
    import omni.kit.commands

    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not status:
        raise RuntimeError(
            "Не удалось создать конфигурацию импорта URDF"
        )
    _configure_franka_urdf_import(import_config, fix_base=fix_base)

    status, imported_prim = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(urdf_path),
        import_config=import_config,
        dest_path="",
        get_articulation_root=True,
    )
    if not status:
        raise RuntimeError("Импорт URDF Franka не удался")

    imported_path = str(imported_prim) if imported_prim else ""
    if imported_path and imported_path != target_path:
        omni.kit.commands.execute("MovePrim", path_from=imported_path, path_to=target_path)
        return target_path
    return imported_path or target_path


def _configure_franka_urdf_import(import_config, *, fix_base: bool) -> None:
    """Apply deterministic position-drive gains to a headless Franka import."""
    import_config.fix_base = fix_base
    import_config.merge_fixed_joints = True
    import_config.import_inertia_tensor = True
    import_config.default_drive_type = _POSITION_DRIVE_TYPE
    import_config.default_drive_strength = _URDF_DRIVE_STIFFNESS
    import_config.default_position_drive_damping = _URDF_DRIVE_DAMPING


def _configure_franka_articulation_drives(robot) -> None:
    """Set and verify stable gains on the loaded USD or URDF articulation."""
    import numpy as np

    dof_names = tuple(robot.dof_names)
    if len(dof_names) != len(set(dof_names)) or set(dof_names) != set(
        _FRANKA_DOF_NAMES
    ):
        raise RuntimeError(
            "Articulation Franka имеет неожиданный набор DOF: "
            + ", ".join(dof_names)
        )

    controller = robot.get_articulation_controller()
    current_kps, current_kds = controller.get_gains()
    expected_kps = np.asarray(current_kps, dtype=float).copy()
    expected_kds = np.asarray(current_kds, dtype=float).copy()
    if expected_kps.shape != (len(dof_names),) or expected_kds.shape != (
        len(dof_names),
    ):
        raise RuntimeError("Isaac вернул неожиданный размер drive gains Franka")
    for joint_name, stiffness, damping in zip(
        _FRANKA_ARM_DOF_NAMES,
        _FRANKA_ARM_DRIVE_STIFFNESSES,
        _FRANKA_ARM_DRIVE_DAMPINGS,
        strict=True,
    ):
        joint_index = dof_names.index(joint_name)
        expected_kps[joint_index] = stiffness
        expected_kds[joint_index] = damping
    for joint_name in ("panda_finger_joint1", "panda_finger_joint2"):
        joint_index = dof_names.index(joint_name)
        expected_kps[joint_index] = _FRANKA_FINGER_DRIVE_STIFFNESS
        expected_kds[joint_index] = _FRANKA_FINGER_DRIVE_DAMPING
    controller.set_gains(kps=expected_kps, kds=expected_kds)
    robot.set_solver_position_iteration_count(_FRANKA_SOLVER_ITERATIONS)
    robot.set_solver_velocity_iteration_count(_FRANKA_SOLVER_ITERATIONS)
    actual_kps, actual_kds = controller.get_gains()
    if not np.allclose(actual_kps, expected_kps, rtol=0.0, atol=1e-6) or not np.allclose(
        actual_kds,
        expected_kds,
        rtol=0.0,
        atol=1e-6,
    ):
        raise RuntimeError("Isaac не применил заданные drive gains Franka")
    if (
        robot.get_solver_position_iteration_count() != _FRANKA_SOLVER_ITERATIONS
        or robot.get_solver_velocity_iteration_count() != _FRANKA_SOLVER_ITERATIONS
    ):
        raise RuntimeError("Isaac не применил solver iterations Franka")
