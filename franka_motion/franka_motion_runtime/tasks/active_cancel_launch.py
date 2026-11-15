"""
Build the Gazebo active-cancel request launch from scenario-owned data.

Responsibility:
- Serialize one V2 Gazebo intent and pass persistent fixture identities to the probe.

Does not own:
- Goal geometry, cancellation transport, evidence checks, or simulator bringup.
"""

from __future__ import annotations

from collections.abc import Sequence

from launch import LaunchDescription

from franka_motion_runtime.tasks.launch import (
    create_simulator_transfer_sequence_description,
)
from franka_motion_runtime.tasks.model import MoveObject


def create_gazebo_active_cancel_description(
    requests: Sequence[MoveObject],
    *,
    controller_action_name: str,
    persistent_world_object_ids: Sequence[str] = ("table",),
) -> LaunchDescription:
    """Create one Gazebo request canceled during active approach motion."""
    if not isinstance(controller_action_name, str) or not controller_action_name.strip():
        raise ValueError("active-cancel controller action name must not be empty")
    persistent_ids = tuple(persistent_world_object_ids)
    if not persistent_ids or not all(
        isinstance(value, str) and value for value in persistent_ids
    ):
        raise ValueError("persistent world object IDs must be non-empty strings")

    return create_simulator_transfer_sequence_description(
        requests,
        simulator="gazebo",
        executable="franka_transfer_sequence_active_cancel_probe",
        extra_parameters={
            "probe.controller_action_name": controller_action_name,
            "probe.persistent_world_object_ids": list(persistent_ids),
        },
    )
