"""
Test Isaac launch arguments, command construction, and stereo geometry.

Responsibility:
- Preserve public launch defaults and package-local simulator helper contracts.
"""

from pathlib import Path
import sys
from types import SimpleNamespace

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
import pytest


_PACKAGE_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_PACKAGE_ROOT / "launch"))

from franka_motion_parts.arguments import declare_common_isaac_arguments  # noqa: E402
import franka_motion_parts.paths as public_paths  # noqa: E402
from franka_motion_runtime.simulation.isaac.actions import (  # noqa: E402
    create_managed_isaac_command,
)
from franka_motion_parts.isaac.command import create_isaac_command  # noqa: E402
from franka_motion_parts.isaac.scenarios import IsaacMoveItScenario  # noqa: E402
from franka_motion_parts.isaac.table_camera import (  # noqa: E402
    IsaacStereoRigSpec,
    create_isaac_stereo_world_poses,
)
from franka_motion_runtime.simulation.isaac.command import (  # noqa: E402
    create_isaac_command as canonical_create_isaac_command,
)


def _default_value(argument: DeclareLaunchArgument) -> str:
    return perform_substitutions(LaunchContext(), argument.default_value)


def _argument_defaults(
    arguments: list[DeclareLaunchArgument],
) -> dict[str, str]:
    return {argument.name: _default_value(argument) for argument in arguments}


def _isaac_paths() -> SimpleNamespace:
    package_share = Path("/pkg/share/franka_motion")
    isaac_worlds = package_share / "worlds" / "isaac"
    return SimpleNamespace(
        package_share=package_share,
        managed_process_wrapper=Path(
            "/pkg/lib/franka_motion/managed_process_wrapper.py"
        ),
        isaac_worlds=isaac_worlds,
        panda_usd=isaac_worlds / "FrankaPanda" / "franka.usd",
        isaac_script=package_share / "scripts" / "run_franka_isaac_sim.py",
    )


def test_compatibility_command_import_is_the_canonical_symbol() -> None:
    """The historical helper path must not retain another implementation."""
    assert create_isaac_command is canonical_create_isaac_command


def test_managed_isaac_command_allows_graceful_bounded_shutdown() -> None:
    """The runner must receive SIGTERM before the wrapper escalates to SIGKILL."""
    wrapped = create_managed_isaac_command(
        _isaac_paths(),
        ["/isaac/python.sh", "runner.py"],
    )

    assert wrapped == [
        "/pkg/lib/franka_motion/managed_process_wrapper.py",
        "--child-name",
        "isaac_sim",
        "--shutdown-signal",
        "term",
        "--shutdown-timeout-sec",
        "10.0",
        "--kill-timeout-sec",
        "2.0",
        "--",
        "/isaac/python.sh",
        "runner.py",
    ]


def test_common_isaac_argument_defaults_are_stable(monkeypatch) -> None:
    """Public launch defaults must retain simulated-time and ROS bridge policy."""
    monkeypatch.delenv("ISAAC_SIM_ROOT", raising=False)
    monkeypatch.delenv("FRANKA_ISAAC_PYTHON", raising=False)
    defaults = _argument_defaults(declare_common_isaac_arguments())

    assert defaults["use_sim_time"] == "true"
    assert defaults["headless"] == "false"
    assert defaults["fix_base"] == "true"
    assert defaults["isaac_python"] == "python3"
    assert defaults["isaac_robot_prim"] == "/World/Franka"
    assert defaults["time_steps"] == "0"
    assert defaults["wait_for_clock"] == "true"
    assert defaults["ros_domain_id"] == "0"
    assert defaults["rmw_implementation"] == "rmw_fastrtps_cpp"


def test_common_isaac_python_default_honors_environment_override(
    monkeypatch,
) -> None:
    """ISAAC_SIM_ROOT must remain the supported machine-local override."""
    monkeypatch.delenv("FRANKA_ISAAC_PYTHON", raising=False)
    monkeypatch.setenv("ISAAC_SIM_ROOT", "/opt/isaac-sim")
    defaults = _argument_defaults(declare_common_isaac_arguments())

    assert defaults["isaac_python"] == "/opt/isaac-sim/python.sh"


def test_stereo_rig_drives_runner_arguments_and_camera_tf() -> None:
    """Runner flags and TF poses must derive from one immutable rig spec."""
    rig = IsaacStereoRigSpec(
        center_position=(1.2, -0.1, 0.9),
        baseline_m=0.08,
        pitch_down_rad=0.7,
        yaw_rad=3.0,
        look_distance_m=1.5,
    )

    arguments = rig.runner_arguments()
    center_index = arguments.index("--stereo-center-position")
    center = tuple(
        float(value) for value in arguments[center_index + 1:center_index + 4]
    )
    left_pose, right_pose = rig.world_poses()

    assert center == rig.center_position
    assert arguments[arguments.index("--stereo-baseline-m") + 1] == "0.08"
    assert left_pose["pitch"] == right_pose["pitch"] == 0.7
    assert right_pose["y"] - left_pose["y"] == pytest.approx(0.08)


def test_stereo_compatibility_helper_uses_rig_spec() -> None:
    """The public pose helper must delegate to the canonical geometry object."""
    cube_position = (0.5, 0.1, 0.03)

    assert create_isaac_stereo_world_poses(cube_position) == (
        IsaacStereoRigSpec.above_cube(cube_position).world_poses()
    )


@pytest.mark.parametrize(
    "kwargs",
    [
        {"center_position": (1.0, 2.0)},
        {"center_position": (1.0, 2.0, 3.0), "baseline_m": 0.0},
        {"center_position": (1.0, 2.0, 3.0), "look_distance_m": -1.0},
    ],
)
def test_stereo_rig_rejects_invalid_geometry(kwargs: dict[str, object]) -> None:
    """Invalid geometry must fail before the heavy simulator starts."""
    with pytest.raises(ValueError):
        IsaacStereoRigSpec(**kwargs)


def test_command_includes_scenario_runtime_arguments() -> None:
    """Command construction must use local assets and stable runner flags."""
    paths = _isaac_paths()
    scenario = IsaacMoveItScenario(
        scene_usd_file="table_world_usd_clean.usd",
        include_isaac_experience_arg=True,
        no_default_ground=True,
        add_cube=True,
        cube_size=0.07,
        cube_position=(0.4, -0.1, 0.05),
    )

    command = create_isaac_command(
        paths=paths,
        scenario=scenario,
        isaac_python="/isaac/python.sh",
        isaac_prim="/World/Robot",
        headless=True,
        fix_base=True,
        time_steps="120",
        isaac_experience="/tmp/app.kit",
    )

    assert command[:2] == ["/isaac/python.sh", str(paths.isaac_script)]
    assert "--install_path" not in command
    assert command[command.index("--robot-usd") + 1] == str(paths.panda_usd)
    assert command[command.index("--scene-prim") + 1] == "/World/Robot"
    assert command[command.index("--scene-usd") + 1] == str(
        paths.isaac_worlds / "table_world_usd_clean.usd"
    )
    assert command[command.index("--experience") + 1] == "/tmp/app.kit"
    assert command[command.index("--time-steps") + 1] == "120"
    assert "--headless" in command
    assert "--fix-base" in command
    assert "--no-default-ground" in command


def test_command_omits_disabled_optional_arguments() -> None:
    """Disabled scenario features must not leak empty flags to Isaac."""
    command = create_isaac_command(
        paths=_isaac_paths(),
        scenario=IsaacMoveItScenario(include_isaac_experience_arg=True),
        isaac_python="/isaac/python.sh",
        isaac_prim="/World/Robot",
        headless=False,
        fix_base=False,
        time_steps="0",
        isaac_experience="",
    )

    for option in (
        "--scene-usd",
        "--add-cube",
        "--headless",
        "--experience",
        "--fix-base",
        "--time-steps",
    ):
        assert option not in command


def test_command_accepts_self_contained_bringup_path_contract(monkeypatch) -> None:
    """Public path factories must resolve Isaac assets from franka_motion."""
    monkeypatch.setattr(
        public_paths,
        "get_package_share_directory",
        lambda package_name: f"/share/{package_name}",
    )
    monkeypatch.setattr(
        public_paths,
        "get_package_prefix",
        lambda package_name: f"/prefix/{package_name}",
    )
    paths = public_paths.create_isaac_launch_paths()

    command = create_isaac_command(
        paths=paths,
        scenario=IsaacMoveItScenario(scene_usd_file="table.usd"),
        isaac_python="/isaac/python.sh",
        isaac_prim="/World/Robot",
        headless=False,
        fix_base=False,
        time_steps="0",
        isaac_experience="",
    )

    assert command[command.index("--scene-usd") + 1] == (
        "/share/franka_motion/worlds/isaac/table.usd"
    )


def test_explicit_isaac_interpreter_overrides_binary_root(monkeypatch):
    """A venv interpreter must not be mistaken for an installation directory."""
    monkeypatch.setenv('ISAAC_SIM_ROOT', '/binary/install')
    monkeypatch.setenv('FRANKA_ISAAC_PYTHON', '/venv/bin/python')
    assert _argument_defaults(declare_common_isaac_arguments())['isaac_python'] == '/venv/bin/python'
