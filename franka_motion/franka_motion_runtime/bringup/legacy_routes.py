"""
Stable mapping from typed bringup choices to canonical MoveIt leaves.

Responsibility:
- Route each simulator/scenario/planner selection to one maintained MoveIt leaf.
- Carry planner selection as an explicit leaf launch argument.

Does not own:
- Leaf launch arguments, ROS names or startup ordering.
- Planner implementation or optional-package discovery.
"""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping

from franka_motion_runtime.bringup.contracts import (
    BringupSelection,
    InvalidBringupSelection,
    PlannerBackend,
    ScenarioProfile,
    SimulatorBackend,
)

PRODUCTION_LEAF_PACKAGE = "franka_motion"


@dataclass(frozen=True)
class LegacyLaunchRoute:
    """Package, launch basename and pipeline selected by the router."""

    package_name: str
    launch_file: str
    planner_backend: PlannerBackend


_MOVEIT_SELECTION_BY_LAUNCH = {
    "franka_gazebo_moveit.launch.py": BringupSelection(
        SimulatorBackend.GAZEBO,
        ScenarioProfile.EMPTY,
        PlannerBackend.MOVEIT,
    ),
    "franka_gazebo_moveit_table.launch.py": BringupSelection(
        SimulatorBackend.GAZEBO,
        ScenarioProfile.TABLE,
        PlannerBackend.MOVEIT,
    ),
    "franka_gazebo_moveit_table_camera.launch.py": BringupSelection(
        SimulatorBackend.GAZEBO,
        ScenarioProfile.TABLE_CAMERA,
        PlannerBackend.MOVEIT,
    ),
    "franka_isaac_moveit.launch.py": BringupSelection(
        SimulatorBackend.ISAAC,
        ScenarioProfile.EMPTY,
        PlannerBackend.MOVEIT,
    ),
    "franka_isaac_moveit_table.launch.py": BringupSelection(
        SimulatorBackend.ISAAC,
        ScenarioProfile.TABLE,
        PlannerBackend.MOVEIT,
    ),
    "franka_isaac_moveit_table_camera.launch.py": BringupSelection(
        SimulatorBackend.ISAAC,
        ScenarioProfile.TABLE_CAMERA,
        PlannerBackend.MOVEIT,
    ),
}

SELECTION_BY_LEGACY_LAUNCH: Mapping[str, BringupSelection] = MappingProxyType(
    _MOVEIT_SELECTION_BY_LAUNCH
)

_ROUTE_BY_SELECTION = {
    selection: LegacyLaunchRoute(
        package_name=PRODUCTION_LEAF_PACKAGE,
        launch_file=launch_file,
        planner_backend=PlannerBackend.MOVEIT,
    )
    for launch_file, selection in _MOVEIT_SELECTION_BY_LAUNCH.items()
}
_ROUTE_BY_SELECTION.update(
    {
        BringupSelection(
            selection.simulator,
            selection.scenario,
            PlannerBackend.CUROBO,
        ): LegacyLaunchRoute(
            package_name=PRODUCTION_LEAF_PACKAGE,
            launch_file=launch_file,
            planner_backend=PlannerBackend.CUROBO,
        )
        for launch_file, selection in _MOVEIT_SELECTION_BY_LAUNCH.items()
        if selection.scenario is not ScenarioProfile.TABLE_CAMERA
    }
)
ROUTE_BY_SELECTION: Mapping[BringupSelection, LegacyLaunchRoute] = (
    MappingProxyType(_ROUTE_BY_SELECTION)
)
PUBLIC_COMPATIBILITY_LAUNCHES = tuple(_MOVEIT_SELECTION_BY_LAUNCH)


def route_for_selection(selection: BringupSelection) -> LegacyLaunchRoute:
    """Return the leaf route for a previously validated selection."""
    try:
        return ROUTE_BY_SELECTION[selection]
    except KeyError as error:
        raise InvalidBringupSelection(
            "No launch route exists for "
            f"{selection.simulator.value}/"
            f"{selection.scenario.value}/"
            f"{selection.planner_backend.value}"
        ) from error


def route_for_legacy_launch_name(launch_file: str) -> LegacyLaunchRoute:
    """Return the canonical MoveIt route behind one stable public basename."""
    try:
        selection = SELECTION_BY_LEGACY_LAUNCH[launch_file]
    except KeyError as error:
        allowed_names = ", ".join(PUBLIC_COMPATIBILITY_LAUNCHES)
        raise InvalidBringupSelection(
            f"Unknown compatibility launch {launch_file!r}; expected: "
            f"{allowed_names}"
        ) from error
    return route_for_selection(selection)
