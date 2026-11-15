"""
Test typed simulator, planner, and scenario backend selection.

Responsibility: Keep selector domains and shared MoveIt-leaf routing closed and explicit.
"""

import pytest

from franka_motion_runtime.bringup.contracts import (
    BringupSelection,
    InvalidBringupSelection,
    PlannerBackend,
    ScenarioProfile,
    SimulatorBackend,
)
from franka_motion_runtime.bringup.legacy_routes import (
    PRODUCTION_LEAF_PACKAGE,
    PUBLIC_COMPATIBILITY_LAUNCHES,
    ROUTE_BY_SELECTION,
    route_for_legacy_launch_name,
    route_for_selection,
)

_EXPECTED_PUBLIC_LAUNCHES = (
    "franka_gazebo_moveit.launch.py",
    "franka_gazebo_moveit_table.launch.py",
    "franka_gazebo_moveit_table_camera.launch.py",
    "franka_isaac_moveit.launch.py",
    "franka_isaac_moveit_table.launch.py",
    "franka_isaac_moveit_table_camera.launch.py",
)


def test_selector_enums_are_closed_and_documented() -> None:
    """The generic router accepts only stable public selector values."""
    assert tuple(member.value for member in SimulatorBackend) == (
        "gazebo",
        "isaac",
    )
    assert tuple(member.value for member in ScenarioProfile) == (
        "empty",
        "table",
        "table_camera",
    )
    assert tuple(member.value for member in PlannerBackend) == (
        "moveit",
        "curobo",
    )


def test_both_planners_route_to_the_same_moveit_leaves() -> None:
    """Planner selection must not create a parallel launch/lifecycle graph."""
    assert PUBLIC_COMPATIBILITY_LAUNCHES == _EXPECTED_PUBLIC_LAUNCHES
    assert len(ROUTE_BY_SELECTION) == 10

    for simulator in SimulatorBackend:
        for scenario in (ScenarioProfile.EMPTY, ScenarioProfile.TABLE):
            moveit_route = route_for_selection(
                BringupSelection(simulator, scenario, PlannerBackend.MOVEIT)
            )
            curobo_route = route_for_selection(
                BringupSelection(simulator, scenario, PlannerBackend.CUROBO)
            )
            assert moveit_route.package_name == PRODUCTION_LEAF_PACKAGE
            assert curobo_route.package_name == PRODUCTION_LEAF_PACKAGE
            assert curobo_route.launch_file == moveit_route.launch_file
            assert moveit_route.planner_backend is PlannerBackend.MOVEIT
            assert curobo_route.planner_backend is PlannerBackend.CUROBO


@pytest.mark.parametrize("launch_file", _EXPECTED_PUBLIC_LAUNCHES)
def test_stable_launch_name_defaults_to_moveit(launch_file: str) -> None:
    """Direct stable launch names preserve the dependency-free OMPL default."""
    route = route_for_legacy_launch_name(launch_file)
    assert route.launch_file == launch_file
    assert route.planner_backend is PlannerBackend.MOVEIT


@pytest.mark.parametrize(
    ("argument_name", "selection_kwargs"),
    (
        (
            "simulator",
            {
                "simulator": "unknown",
                "scenario": "empty",
                "planner_backend": "moveit",
            },
        ),
        (
            "scenario",
            {
                "simulator": "gazebo",
                "scenario": "unknown",
                "planner_backend": "moveit",
            },
        ),
        (
            "planner_backend",
            {
                "simulator": "gazebo",
                "scenario": "empty",
                "planner_backend": "unknown",
            },
        ),
    ),
)
def test_unknown_selector_reports_argument_and_allowed_values(
    argument_name: str,
    selection_kwargs: dict[str, str],
) -> None:
    """Malformed strings fail at the typed configuration boundary."""
    with pytest.raises(InvalidBringupSelection, match=argument_name) as error:
        BringupSelection.from_strings(**selection_kwargs)

    assert "expected one of" in str(error.value)


@pytest.mark.parametrize("simulator", ("gazebo", "isaac"))
def test_curobo_rejects_table_camera_octomap(simulator: str) -> None:
    """The adapter must never silently omit table-camera occupancy data."""
    with pytest.raises(InvalidBringupSelection, match="Octomap"):
        BringupSelection.from_strings(
            simulator=simulator,
            scenario="table_camera",
            planner_backend="curobo",
        )


def test_unknown_compatibility_name_fails_explicitly() -> None:
    """Removed cuRobo aliases must not silently route to another profile."""
    with pytest.raises(InvalidBringupSelection, match="Unknown compatibility"):
        route_for_legacy_launch_name("franka_gazebo_curobo.launch.py")
