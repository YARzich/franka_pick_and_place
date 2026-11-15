"""
Test launch event sequencing and simulator table-profile constants.

Responsibility: Verify dependency gates, fail-closed exits, and canonical tabletop geometry.
"""

from pathlib import Path
from types import SimpleNamespace
import sys

from launch import LaunchContext
from launch.actions import EmitEvent
from launch.events import Shutdown


_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_PACKAGE_ROOT / 'launch'))

from franka_motion_parts.events import (  # noqa: E402
    actions_after_successful_exit,
    shutdown_after_process_exit,
    shutdown_after_process_failure,
)
from franka_motion_parts.planning_scene import (  # noqa: E402
    GAZEBO_TABLE_COLLISION,
    ISAAC_TABLE_COLLISION,
)


def _process_exit(returncode: int):
    return SimpleNamespace(returncode=returncode)


def _shutdown_event(actions):
    assert len(actions) == 1
    assert isinstance(actions[0], EmitEvent)
    assert isinstance(actions[0].event, Shutdown)
    return actions[0].event


def test_successful_prerequisite_starts_dependents():
    dependent_actions = (object(), object())

    result = actions_after_successful_exit(
        _process_exit(0),
        LaunchContext(),
        actions=dependent_actions,
        prerequisite_name='controller spawner',
    )

    assert result == list(dependent_actions)


def test_failed_prerequisite_stops_launch_without_starting_dependents():
    result = actions_after_successful_exit(
        _process_exit(3),
        LaunchContext(),
        actions=(object(),),
        prerequisite_name='controller spawner',
    )

    shutdown = _shutdown_event(result)
    assert 'controller spawner' in shutdown.reason
    assert '3' in shutdown.reason


def test_simulator_exit_always_stops_launch():
    result = shutdown_after_process_exit(
        _process_exit(0),
        LaunchContext(),
        process_name='Gazebo',
    )

    shutdown = _shutdown_event(result)
    assert 'Gazebo' in shutdown.reason


def test_one_shot_critical_process_only_stops_launch_on_failure():
    assert shutdown_after_process_failure(
        _process_exit(0),
        LaunchContext(),
        process_name='planning scene',
    ) == []

    result = shutdown_after_process_failure(
        _process_exit(1),
        LaunchContext(),
        process_name='planning scene',
    )
    assert 'planning scene' in _shutdown_event(result).reason


def test_process_exit_handlers_are_noops_during_shutdown():
    context = LaunchContext()
    context._set_is_shutdown(True)

    assert actions_after_successful_exit(
        _process_exit(0),
        context,
        actions=(object(),),
        prerequisite_name='controller spawner',
    ) == []
    assert shutdown_after_process_exit(
        _process_exit(0),
        context,
        process_name='Gazebo',
    ) == []
    assert shutdown_after_process_failure(
        _process_exit(1),
        context,
        process_name='planning scene',
    ) == []


def test_table_collision_profiles_match_simulator_tabletops():
    assert GAZEBO_TABLE_COLLISION.frame_id == 'panda_link0'
    assert GAZEBO_TABLE_COLLISION.dimensions == (1.5532, 2.3937, 0.05)
    assert GAZEBO_TABLE_COLLISION.position == (0.6669, 0.0, -0.027)

    assert ISAAC_TABLE_COLLISION.frame_id == 'panda_link0'
    assert ISAAC_TABLE_COLLISION.dimensions == (2.0, 1.0, 0.1)
    assert ISAAC_TABLE_COLLISION.position == (0.0, 0.0, -0.052)
