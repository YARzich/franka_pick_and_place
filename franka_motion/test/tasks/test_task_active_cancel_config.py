"""
Test explicit controller endpoint configuration for active cancellation.

Responsibility:
- Reject missing controller endpoints before the probe can submit a goal.
- Preserve scenario-owned endpoints through launch factory and leaf wiring.
"""

from pathlib import Path
import runpy
from types import SimpleNamespace

import pytest
from rclpy.parameter import Parameter

from franka_motion_runtime.tasks.active_cancel_evidence import (
    load_active_cancel_probe_config,
)
import franka_motion_runtime.tasks.active_cancel_launch as active_cancel_launch
from franka_motion_runtime.tasks.model import MoveObject, Pose


class _ParameterNode:
    """Declare parameters without allocating a ROS node or middleware context."""

    def __init__(self, overrides):
        self.overrides = overrides
        self.declarations = {}

    def declare_parameter(self, name, default):
        self.declarations[name] = default
        value = None if isinstance(default, Parameter.Type) else default
        return SimpleNamespace(value=self.overrides.get(name, value))


@pytest.mark.parametrize('endpoint', [None, '', ' \t ', 42])
def test_probe_rejects_missing_or_invalid_controller_endpoint(endpoint):
    node = _ParameterNode({
        'probe.controller_action_name': endpoint,
        'probe.persistent_world_object_ids': ['table'],
    })

    with pytest.raises(ValueError, match='controller action name'):
        load_active_cancel_probe_config(node)


def test_probe_controller_endpoint_is_required_and_has_no_scenario_default():
    node = _ParameterNode({'probe.persistent_world_object_ids': ['table']})

    with pytest.raises(ValueError, match='controller action name'):
        load_active_cancel_probe_config(node)

    assert node.declarations['probe.controller_action_name'] is Parameter.Type.STRING


def test_probe_preserves_configured_controller_endpoint():
    endpoint = '/custom_arm/follow_joint_trajectory'
    node = _ParameterNode({
        'probe.controller_action_name': endpoint,
        'probe.persistent_world_object_ids': ['table', 'fixture'],
    })

    config = load_active_cancel_probe_config(node)

    assert config.controller_action_name == endpoint
    assert config.persistent_world_object_ids == ('table', 'fixture')


@pytest.mark.parametrize('endpoint', [None, '', ' \t ', 42])
def test_active_cancel_launch_rejects_invalid_controller_endpoint(endpoint):
    with pytest.raises(ValueError, match='controller action name'):
        active_cancel_launch.create_gazebo_active_cancel_description(
            (), controller_action_name=endpoint,
        )


def test_active_cancel_factory_forwards_controller_endpoint_to_probe(monkeypatch):
    captured = {}
    description = object()

    def create_description(requests, **kwargs):
        captured.update(requests=requests, **kwargs)
        return description

    monkeypatch.setattr(
        active_cancel_launch, 'create_simulator_transfer_sequence_description',
        create_description,
    )
    requests = (MoveObject('cube', Pose('panda_link0', (0.5, 0.0, 0.1))),)

    result = active_cancel_launch.create_gazebo_active_cancel_description(
        requests,
        controller_action_name='/custom_arm/follow_joint_trajectory',
        persistent_world_object_ids=('table', 'fixture'),
    )

    assert result is description
    assert captured['requests'] is requests
    assert captured['simulator'] == 'gazebo'
    assert captured['executable'] == 'franka_transfer_sequence_active_cancel_probe'
    assert captured['extra_parameters'] == {
        'probe.controller_action_name': '/custom_arm/follow_joint_trajectory',
        'probe.persistent_world_object_ids': ['table', 'fixture'],
    }


def test_active_cancel_gazebo_leaf_supplies_panda_controller_endpoint(monkeypatch):
    captured = {}
    description = object()

    def create_description(requests, **kwargs):
        captured.update(requests=requests, **kwargs)
        return description

    monkeypatch.setattr(
        active_cancel_launch, 'create_gazebo_active_cancel_description',
        create_description,
    )
    namespace = runpy.run_path(
        Path(__file__).resolve().parents[2] / 'launch' / 'franka_gazebo_active_cancel.launch.py'
    )

    assert namespace['generate_launch_description']() is description
    assert captured['controller_action_name'] == '/panda_arm_controller/follow_joint_trajectory'
    assert captured['persistent_world_object_ids'] == ('table',)
    assert len(captured['requests']) == 1
    assert captured['requests'][0].object_id == 'cube'
