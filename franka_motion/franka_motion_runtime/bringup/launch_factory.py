"""
Launch-description factories for typed routing and legacy forwarders.

Responsibility:
- Declare selector and simulated-time launch arguments.
- Resolve one validated leaf and include it with explicit use_sim_time.

Does not own:
- Nodes, processes, event ordering or backend-specific parameters.
- Leaf implementation or ROS runtime state.
"""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from franka_motion_runtime.bringup.contracts import BringupSelection
from franka_motion_runtime.bringup.legacy_routes import (
    LegacyLaunchRoute,
    route_for_legacy_launch_name,
    route_for_selection,
)


def create_typed_bringup_description() -> LaunchDescription:
    """Create the generic fail-fast simulator/scenario/planner router."""
    simulator_argument = DeclareLaunchArgument(
        "simulator",
        default_value="gazebo",
        description="Simulator backend: gazebo | isaac.",
        choices=["gazebo", "isaac"],
    )
    scenario_argument = DeclareLaunchArgument(
        "scenario",
        default_value="empty",
        description="Scene profile: empty | table | table_camera.",
        choices=["empty", "table", "table_camera"],
    )
    planner_argument = DeclareLaunchArgument(
        "planner_backend",
        default_value="moveit",
        description="MoveIt planning pipeline: moveit | curobo.",
        choices=["moveit", "curobo"],
    )
    use_sim_time_argument = _declare_use_sim_time_argument()
    selected_leaf_action = OpaqueFunction(
        function=_select_leaf_launch,
        kwargs={
            "simulator": LaunchConfiguration("simulator"),
            "scenario": LaunchConfiguration("scenario"),
            "planner_backend": LaunchConfiguration("planner_backend"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        },
    )
    launch_actions = [
        simulator_argument,
        scenario_argument,
        planner_argument,
        use_sim_time_argument,
        selected_leaf_action,
    ]
    launch_description = LaunchDescription(launch_actions)
    return launch_description


def create_compatibility_wrapper_description(
    launch_file: str,
    *,
    forwarded_arguments: Mapping[str, object] | None = None,
) -> LaunchDescription:
    """Create a thin legacy-package forwarder to one canonical leaf."""
    use_sim_time_argument = _declare_use_sim_time_argument()
    use_sim_time = LaunchConfiguration("use_sim_time")
    leaf_route = route_for_legacy_launch_name(launch_file)
    leaf_launch_include = create_leaf_launch_include(
        leaf_route,
        use_sim_time=use_sim_time,
        forwarded_arguments=forwarded_arguments,
    )
    launch_actions = [
        use_sim_time_argument,
        leaf_launch_include,
    ]
    launch_description = LaunchDescription(launch_actions)
    return launch_description


def create_leaf_launch_include(
    route: LegacyLaunchRoute,
    *,
    use_sim_time: LaunchConfiguration,
    forwarded_arguments: Mapping[str, object] | None = None,
) -> IncludeLaunchDescription:
    """Resolve and include a leaf while forwarding the simulation clock policy."""
    leaf_launch_path = _resolve_leaf_launch_path(route)
    resolved_arguments = {
        "use_sim_time": use_sim_time,
        "planner_backend": route.planner_backend.value,
    }
    if forwarded_arguments:
        resolved_arguments.update(forwarded_arguments)
    leaf_launch_source = PythonLaunchDescriptionSource(str(leaf_launch_path))
    leaf_launch_include = IncludeLaunchDescription(
        leaf_launch_source,
        launch_arguments=resolved_arguments.items(),
    )
    return leaf_launch_include


def _declare_use_sim_time_argument() -> DeclareLaunchArgument:
    """Declare the clock policy shared by every simulation entrypoint."""
    use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description=(
            "Use simulator /clock for every ROS node in the selected leaf."
        ),
        choices=["true", "false"],
    )
    return use_sim_time_argument


def _select_leaf_launch(
    context: LaunchContext,
    *,
    simulator: LaunchConfiguration,
    scenario: LaunchConfiguration,
    planner_backend: LaunchConfiguration,
    use_sim_time: LaunchConfiguration,
) -> list[IncludeLaunchDescription]:
    """Validate the complete selector tuple before resolving a leaf path."""
    selection = BringupSelection.from_strings(
        simulator=simulator.perform(context),
        scenario=scenario.perform(context),
        planner_backend=planner_backend.perform(context),
    )
    leaf_route = route_for_selection(selection)
    leaf_launch_include = create_leaf_launch_include(
        leaf_route,
        use_sim_time=use_sim_time,
    )
    return [leaf_launch_include]


def _resolve_leaf_launch_path(route: LegacyLaunchRoute) -> Path:
    """Resolve one installed leaf and report missing migration assets early."""
    try:
        package_share = Path(
            get_package_share_directory(route.package_name)
        )
    except PackageNotFoundError as error:
        raise RuntimeError(
            "Canonical leaf package is unavailable: "
            f"{route.package_name}"
        ) from error

    launch_path = package_share / "launch" / route.launch_file
    if not launch_path.is_file():
        raise FileNotFoundError(
            f"Required leaf launch is unavailable: {launch_path}"
        )
    return launch_path
