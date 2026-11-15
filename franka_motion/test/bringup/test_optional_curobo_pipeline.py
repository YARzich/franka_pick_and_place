"""
Test the Franka-side optional cuRobo pipeline without importing CUDA or cuRobo.

Responsibility:
- Prove fail-fast selection, request-owned scene config, and OMPL independence.

Does not own:
- PlannerManager conversion internals, GPU planning, or simulator runtime.
"""

from pathlib import Path
from types import SimpleNamespace

from ament_index_python.packages import PackageNotFoundError
import pytest
import yaml

from franka_motion_runtime.bringup.contracts import (
    InvalidBringupSelection,
    PlannerBackend,
)
from franka_motion_runtime.bringup.launch_parts import moveit

_PACKAGE_ROOT = Path(__file__).resolve().parents[2]


def test_ompl_selection_never_discovers_the_optional_package(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The default path must work when manipulation_curobo is absent."""
    def unexpected_lookup(package_name: str) -> str:
        raise AssertionError(f"unexpected optional lookup: {package_name}")

    monkeypatch.setattr(moveit, "get_package_share_directory", unexpected_lookup)
    monkeypatch.setattr(moveit, "get_package_prefix", unexpected_lookup)

    moveit.validate_planner_backend(
        PlannerBackend.MOVEIT,
        octomap_profile=moveit.OctomapProfile.DISABLED,
    )
    assert moveit.planning_pipeline_name(PlannerBackend.MOVEIT) == "ompl"


def test_explicit_curobo_selection_fails_before_launch_when_package_is_missing(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """An explicit optional selection reports the missing package directly."""
    def missing_package(package_name: str) -> str:
        raise PackageNotFoundError(package_name)

    monkeypatch.setattr(moveit, "get_package_share_directory", missing_package)

    with pytest.raises(RuntimeError, match="requires the optional"):
        moveit.validate_planner_backend(
            PlannerBackend.CUROBO,
            octomap_profile=moveit.OctomapProfile.DISABLED,
        )


def test_explicit_curobo_selection_rejects_an_incomplete_plugin_install(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Fail before launch when the package exists without plugin resources."""
    package_share = tmp_path / "share" / "manipulation_curobo"
    package_share.mkdir(parents=True)
    (package_share / "curobo_moveit_plugin_description.xml").write_text(
        "<library/>",
        encoding="utf-8",
    )
    monkeypatch.setattr(
        moveit,
        "get_package_share_directory",
        lambda package_name: str(package_share),
    )
    monkeypatch.setattr(
        moveit,
        "get_package_prefix",
        lambda package_name: str(tmp_path),
    )

    with pytest.raises(RuntimeError, match="installed incompletely"):
        moveit.validate_planner_backend(
            PlannerBackend.CUROBO,
            octomap_profile=moveit.OctomapProfile.DISABLED,
        )


def test_curobo_table_camera_fails_before_optional_package_discovery(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Unsupported Octomap data is rejected even on a host with no plugin."""
    def unexpected_lookup(package_name: str) -> str:
        raise AssertionError(f"lookup must not run: {package_name}")

    monkeypatch.setattr(moveit, "get_package_share_directory", unexpected_lookup)
    with pytest.raises(InvalidBringupSelection, match="Octomap"):
        moveit.validate_planner_backend(
            PlannerBackend.CUROBO,
            octomap_profile=moveit.OctomapProfile.TABLE_CAMERA,
        )


def test_curobo_pipeline_uses_only_the_canonical_planner_manager() -> None:
    """Ensure MoveIt config contains one plugin and no action/bridge settings."""
    config = yaml.safe_load(
        (_PACKAGE_ROOT / "config" / "moveit" / "curobo_planning.yaml")
        .read_text(encoding="utf-8")
    )

    assert config["planning_plugins"] == [
        "manipulation_curobo/CuRoboPlanner"
    ]
    assert config["motion_gen_config"] == (
        "package://franka_motion/config/curobo/"
        "panda_curobo_motion_gen.yaml"
    )
    assert config["ee_link"] == "panda_hand"
    assert "move_group" not in config
    assert "action" not in " ".join(config).lower()


def test_motion_gen_profile_declares_only_attachment_mapping() -> None:
    """Attached geometry is explicit while robot ACM remains request-owned."""
    profile = yaml.safe_load(
        (_PACKAGE_ROOT / "config" / "curobo" / "panda_curobo_motion_gen.yaml")
        .read_text(encoding="utf-8")
    )["motion_gen"]

    assert "world_cfg" not in profile
    assert profile["attached_link_map"] == {
        "panda_hand": "attached_object"
    }
    assert profile["attached_touch_links"] == {
        "panda_hand": [
            "panda_hand",
            "panda_leftfinger",
            "panda_rightfinger",
        ]
    }
    assert "self_collision_allow" not in profile


def test_franka_manifest_does_not_install_the_optional_backend() -> None:
    """Package management for the OMPL app must not pull the research backend."""
    manifest = (_PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    assert "manipulation_curobo" not in manifest


@pytest.mark.parametrize("simulator", ["gazebo", "isaac"])
def test_every_move_group_factory_replaces_the_default_action_capabilities(
    simulator: str,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Every leaf installs exact planning and cancelable trajectory execution."""
    monkeypatch.setattr(
        moveit,
        "Node",
        lambda **kwargs: SimpleNamespace(**kwargs),
    )
    moveit_config = SimpleNamespace(
        to_dict=lambda: {
            "planning_pipelines": ["ompl"],
            "default_planning_pipeline": "ompl",
        }
    )
    expected_capabilities = {
        "capabilities": (
            "manipulation_moveit/ExactPlanningSceneMoveGroupCapability "
            "manipulation_moveit/CancelableExecuteTrajectoryCapability"
        ),
        "disable_capabilities": (
            "move_group/MoveGroupMoveAction "
            "move_group/MoveGroupExecuteTrajectoryAction"
        ),
    }
    if simulator == "gazebo":
        node = moveit.create_gazebo_move_group_node(
            use_sim_time=True,
            moveit_config=moveit_config,
            managed_process_wrapper_path=tmp_path / "managed_process",
        )
    else:
        trajectory_execution = tmp_path / "trajectory_execution.yaml"
        trajectory_execution.write_text(
            "move_group:\n  ros__parameters: {}\n",
            encoding="utf-8",
        )
        node = moveit.create_isaac_move_group_node(
            paths=SimpleNamespace(
                managed_process_wrapper=tmp_path / "managed_process",
                moveit_trajectory_execution_config=trajectory_execution,
            ),
            use_sim_time=True,
            moveit_config=moveit_config,
        )

    assert node.package == "moveit_ros_move_group"
    assert node.executable == "move_group"
    assert node.parameters[-1] == expected_capabilities
    assert any(
        isinstance(parameters, dict)
        and parameters.get("default_planning_pipeline") == "ompl"
        for parameters in node.parameters
    )


def test_exact_capability_preserves_canonical_action_and_backend_selection() -> None:
    """Replacement capability must not add a second endpoint or planner policy."""
    backend = yaml.safe_load(
        (_PACKAGE_ROOT / "config" / "manipulation" / "moveit_backend.yaml")
        .read_text(encoding="utf-8")
    )
    parameters = next(iter(backend.values()))["ros__parameters"]

    assert parameters["moveit.move_group_action"] == "/move_action"
    assert parameters["moveit.execute_trajectory_action"] == "/execute_trajectory"
    assert moveit.planning_pipeline_name(PlannerBackend.MOVEIT) == "ompl"
    assert moveit.planning_pipeline_name(PlannerBackend.CUROBO) == "curobo"
    assert set(moveit.manipulation_move_group_capabilities()) == {
        "capabilities",
        "disable_capabilities",
    }


def test_maintained_sources_construct_move_group_only_in_shared_factories() -> None:
    """Prevent a leaf from bypassing the exact capability configuration."""
    production_roots = (
        _PACKAGE_ROOT / "launch",
        _PACKAGE_ROOT / "franka_motion_runtime" / "bringup",
    )
    direct_owners = []
    for root in production_roots:
        for source_path in root.rglob("*.py"):
            source = source_path.read_text(encoding="utf-8")
            if 'package="moveit_ros_move_group"' in source:
                direct_owners.append(source_path.relative_to(_PACKAGE_ROOT))

    assert direct_owners == [
        Path("franka_motion_runtime/bringup/launch_parts/moveit.py")
    ]
    factory_source = (
        _PACKAGE_ROOT
        / "franka_motion_runtime"
        / "bringup"
        / "launch_parts"
        / "moveit.py"
    ).read_text(encoding="utf-8")
    assert factory_source.count("manipulation_move_group_capabilities(),") == 2
