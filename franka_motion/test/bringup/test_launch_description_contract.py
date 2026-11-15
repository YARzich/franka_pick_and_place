"""
Test generic bringup launch descriptions without ROS nodes or processes.

Responsibility: Preserve typed arguments, wrapper forwarding, leaf selection, and validation.
"""

from pathlib import Path

from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, TextSubstitution
import pytest

from franka_motion_runtime.bringup.contracts import InvalidBringupSelection
import franka_motion_runtime.bringup.launch_factory as launch_factory
from franka_motion_runtime.bringup.legacy_routes import PUBLIC_COMPATIBILITY_LAUNCHES


def _text_substitutions_to_string(substitutions) -> str:
    return "".join(
        substitution.text
        for substitution in substitutions
        if isinstance(substitution, TextSubstitution)
    )


def _default_value(argument: DeclareLaunchArgument) -> str:
    return _text_substitutions_to_string(argument.default_value)


def _launch_configuration_name(configuration: LaunchConfiguration) -> str:
    return _text_substitutions_to_string(configuration.variable_name)


def _forwarded_arguments(
    include: IncludeLaunchDescription,
) -> dict[str, object]:
    return dict(include.launch_arguments)


def _expanded_source_name(include: IncludeLaunchDescription) -> str:
    """Load a source through the public API and return its resolved basename."""
    source = include.launch_description_source
    source.get_launch_description(LaunchContext())
    return Path(source.location).name


@pytest.fixture
def fake_leaf_share(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """Create all expected leaf basenames without loading their implementation."""
    launch_directory = tmp_path / "launch"
    launch_directory.mkdir()
    for launch_file in PUBLIC_COMPATIBILITY_LAUNCHES:
        (launch_directory / launch_file).write_text(
            "from launch import LaunchDescription\n\n"
            "def generate_launch_description():\n"
            "    return LaunchDescription([])\n",
            encoding="utf-8",
        )

    monkeypatch.setattr(
        launch_factory,
        "get_package_share_directory",
        lambda package_name: str(tmp_path),
    )
    return tmp_path


def test_generic_description_declares_typed_public_arguments() -> None:
    """The generic entry point must expose one small, stable selector surface."""
    description = launch_factory.create_typed_bringup_description()
    entities = list(description.entities)
    arguments = [
        entity
        for entity in entities
        if isinstance(entity, DeclareLaunchArgument)
    ]

    assert [argument.name for argument in arguments] == [
        "simulator",
        "scenario",
        "planner_backend",
        "use_sim_time",
    ]
    assert {
        argument.name: _default_value(argument) for argument in arguments
    } == {
        "simulator": "gazebo",
        "scenario": "empty",
        "planner_backend": "moveit",
        "use_sim_time": "true",
    }
    assert len(entities) == 5
    assert isinstance(entities[-1], OpaqueFunction)


@pytest.mark.parametrize("launch_file", PUBLIC_COMPATIBILITY_LAUNCHES)
def test_wrapper_includes_exact_leaf_and_forwards_backend_and_sim_time(
    launch_file: str,
    fake_leaf_share: Path,
) -> None:
    """Every stable wrapper must include its matching leaf with ROS clock policy."""
    description = launch_factory.create_compatibility_wrapper_description(
        launch_file
    )
    entities = list(description.entities)

    assert len(entities) == 2
    assert isinstance(entities[0], DeclareLaunchArgument)
    assert entities[0].name == "use_sim_time"
    assert _default_value(entities[0]) == "true"
    assert isinstance(entities[1], IncludeLaunchDescription)
    assert _expanded_source_name(entities[1]) == launch_file

    forwarded_arguments = _forwarded_arguments(entities[1])
    assert set(forwarded_arguments) == {"planner_backend", "use_sim_time"}
    assert (
        _launch_configuration_name(forwarded_arguments["use_sim_time"])
        == "use_sim_time"
    )
    assert forwarded_arguments["planner_backend"] == "moveit"


def test_generic_selection_resolves_after_validation(
    fake_leaf_share: Path,
) -> None:
    """A valid selector tuple must construct exactly one expected include."""
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "simulator": "isaac",
            "scenario": "table_camera",
            "planner_backend": "moveit",
            "use_sim_time": "true",
        }
    )

    actions = launch_factory._select_leaf_launch(
        context,
        simulator=LaunchConfiguration("simulator"),
        scenario=LaunchConfiguration("scenario"),
        planner_backend=LaunchConfiguration("planner_backend"),
        use_sim_time=LaunchConfiguration("use_sim_time"),
    )

    assert len(actions) == 1
    assert isinstance(actions[0], IncludeLaunchDescription)
    assert (
        _expanded_source_name(actions[0])
        == "franka_isaac_moveit_table_camera.launch.py"
    )
    forwarded_arguments = _forwarded_arguments(actions[0])
    assert (
        _launch_configuration_name(forwarded_arguments["use_sim_time"])
        == "use_sim_time"
    )
    assert forwarded_arguments["planner_backend"] == "moveit"


def test_curobo_selection_uses_the_same_moveit_leaf(
    fake_leaf_share: Path,
) -> None:
    """The optional planner must not route through a second launch graph."""
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "simulator": "gazebo",
            "scenario": "table",
            "planner_backend": "curobo",
            "use_sim_time": "true",
        }
    )

    actions = launch_factory._select_leaf_launch(
        context,
        simulator=LaunchConfiguration("simulator"),
        scenario=LaunchConfiguration("scenario"),
        planner_backend=LaunchConfiguration("planner_backend"),
        use_sim_time=LaunchConfiguration("use_sim_time"),
    )

    assert _expanded_source_name(actions[0]) == (
        "franka_gazebo_moveit_table.launch.py"
    )
    assert _forwarded_arguments(actions[0])["planner_backend"] == "curobo"


def test_invalid_combination_fails_before_include_construction(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Unsupported configuration must fail before any leaf side effect."""
    include_was_requested = False

    def record_include_request(*args, **kwargs):
        nonlocal include_was_requested
        include_was_requested = True
        raise AssertionError("include construction must not be reached")

    monkeypatch.setattr(
        launch_factory,
        "create_leaf_launch_include",
        record_include_request,
    )
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "simulator": "gazebo",
            "scenario": "table_camera",
            "planner_backend": "curobo",
            "use_sim_time": "true",
        }
    )

    with pytest.raises(InvalidBringupSelection, match="Octomap"):
        launch_factory._select_leaf_launch(
            context,
            simulator=LaunchConfiguration("simulator"),
            scenario=LaunchConfiguration("scenario"),
            planner_backend=LaunchConfiguration("planner_backend"),
            use_sim_time=LaunchConfiguration("use_sim_time"),
        )

    assert not include_was_requested


def test_missing_leaf_file_fails_explicitly(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Packaging omissions must not turn into a late launch import error."""
    monkeypatch.setattr(
        launch_factory,
        "get_package_share_directory",
        lambda package_name: str(tmp_path),
    )

    with pytest.raises(FileNotFoundError, match="Required leaf launch"):
        launch_factory.create_compatibility_wrapper_description(
            "franka_gazebo_moveit.launch.py"
        )
