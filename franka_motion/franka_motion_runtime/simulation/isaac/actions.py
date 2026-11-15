"""
Isaac Sim launch actions and launch-time configuration parsing.

Responsibility:
- Create managed Isaac Sim process actions from resolved paths and scenario data.
- Provide launch-time helpers for booleans and optional /clock gating.

Does not own:
- Building the Isaac command argument list.
- ROS controller, MoveIt or perception node factories.
"""

from __future__ import annotations

from launch.actions import ExecuteProcess

from .command import create_isaac_command
from .contracts import IsaacLaunchPathsLike
from .environment import create_isaac_process_environment
from .scenarios import IsaacMoveItScenario


def create_managed_isaac_command(
    paths: IsaacLaunchPathsLike,
    isaac_command: list[str],
) -> list[str]:
    """Wrap an Isaac command with graceful shutdown and bounded escalation."""
    return [
        str(paths.managed_process_wrapper),
        "--child-name",
        "isaac_sim",
        "--shutdown-signal",
        "term",
        "--shutdown-timeout-sec",
        "10.0",
        "--kill-timeout-sec",
        "2.0",
        "--",
        *isaac_command,
    ]


def boolean_from_config(value: str) -> bool:
    """Interpret the accepted ROS launch boolean spellings."""
    return value.lower() in {"true", "1", "yes"}


def create_isaac_process(
    *,
    paths: IsaacLaunchPathsLike,
    scenario: IsaacMoveItScenario,
    isaac_python: str,
    isaac_prim: str,
    headless: bool,
    fix_base: bool,
    time_steps: str,
    isaac_experience: str,
    rmw_implementation: str,
) -> ExecuteProcess:
    """Create a managed Isaac process from resolved paths and scenario data."""
    isaac_cmd = create_isaac_command(
        paths=paths,
        scenario=scenario,
        isaac_python=isaac_python,
        isaac_prim=isaac_prim,
        headless=headless,
        fix_base=fix_base,
        time_steps=time_steps,
        isaac_experience=isaac_experience,
    )
    return ExecuteProcess(
        cmd=create_managed_isaac_command(paths, isaac_cmd),
        output="screen",
        additional_env=create_isaac_process_environment(isaac_python, rmw_implementation),
    )


def create_wait_for_clock_action(wait_for_clock_enabled: bool) -> ExecuteProcess | None:
    """Create an optional one-shot clock readiness gate."""
    if not wait_for_clock_enabled:
        return None
    return ExecuteProcess(
        cmd=["ros2", "topic", "echo", "/clock", "--once"],
        output="screen",
    )
