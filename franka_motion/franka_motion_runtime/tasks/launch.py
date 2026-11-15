"""
Build concise action request launches for Franka motion scenarios.

Responsibility:
- Serialize scenario target intent, start one client, and propagate use_sim_time.

Does not own:
- Motion execution, scene leases, action-server bringup, or task goal content.
"""

from __future__ import annotations

from functools import partial
from typing import Sequence

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from franka_motion_runtime.tasks.model import MoveObject, TransferObjectIntent
from franka_motion_runtime.tasks.request import transfer_sequence_parameters


_SUPPORTED_SIMULATORS = frozenset({"gazebo", "isaac"})


def _raise_client_failure(context, *, message: str) -> None:
    """Turn an unsuccessful request client into a non-zero launch result."""
    del context
    raise RuntimeError(message)


def _actions_after_client(event, context) -> list[object]:
    """Shutdown cleanly after success and fail closed after any client error."""
    if context.is_shutdown:
        return []
    if event.returncode != 0:
        return [
            OpaqueFunction(
                function=partial(
                    _raise_client_failure,
                    message=(
                        "Transfer sequence client failed with code "
                        f"{event.returncode}"
                    ),
                )
            )
        ]
    return [
        EmitEvent(
            event=Shutdown(reason="Transfer sequence completed successfully")
        )
    ]


def _create_request_description(client_node: Node) -> LaunchDescription:
    """Wrap one request client with stable time and exit propagation policy."""
    use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Stamp transfer requests from the active simulation clock.",
    )
    completion_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=client_node,
            on_exit=_actions_after_client,
        )
    )

    launch_description = LaunchDescription()
    launch_description.add_action(use_sim_time_argument)
    launch_description.add_action(completion_handler)
    launch_description.add_action(client_node)
    return launch_description


def _create_transfer_sequence_node(
    requests: Sequence[MoveObject | TransferObjectIntent],
    *,
    use_sim_time: LaunchConfiguration,
    executable: str = "franka_transfer_sequence_client",
    extra_parameters: dict[str, object] | None = None,
) -> Node:
    """Serialize all target expressions for exactly one V2 action client."""
    if not requests:
        raise ValueError("transfer sequence must contain at least one request")
    if not all(
        isinstance(request, (MoveObject, TransferObjectIntent))
        for request in requests
    ):
        raise TypeError("transfer sequence may contain only transfer intents")
    intents = tuple(
        request.to_intent() if isinstance(request, MoveObject) else request
        for request in requests
    )
    parameters = transfer_sequence_parameters(intents)
    parameters.update(extra_parameters or {})
    return Node(
        package="franka_motion",
        executable=executable,
        name=executable,
        output="screen",
        parameters=[parameters, {"use_sim_time": use_sim_time}],
    )


def create_simulator_transfer_sequence_description(
    requests: Sequence[MoveObject | TransferObjectIntent],
    *,
    simulator: str,
    executable: str = "franka_transfer_sequence_client",
    extra_parameters: dict[str, object] | None = None,
) -> LaunchDescription:
    """Create one simulator request launch for the common V2 client."""
    if simulator not in _SUPPORTED_SIMULATORS:
        supported = ", ".join(sorted(_SUPPORTED_SIMULATORS))
        raise ValueError(
            f"unsupported task simulator '{simulator}'; expected one of: {supported}"
        )

    return create_transfer_sequence_description(
        requests,
        executable=executable,
        extra_parameters=extra_parameters,
    )


def create_transfer_sequence_description(
    requests: Sequence[MoveObject | TransferObjectIntent],
    *,
    executable: str = "franka_transfer_sequence_client",
    extra_parameters: dict[str, object] | None = None,
) -> LaunchDescription:
    """Create one backend-neutral V2 request for an already running graph."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    client_node = _create_transfer_sequence_node(
        requests,
        use_sim_time=use_sim_time,
        executable=executable,
        extra_parameters=extra_parameters,
    )
    return _create_request_description(client_node)


def create_gazebo_transfer_sequence_description(
    requests: Sequence[MoveObject | TransferObjectIntent],
) -> LaunchDescription:
    """Create a Gazebo-table request using the common V2 intent contract."""
    return create_simulator_transfer_sequence_description(
        requests,
        simulator="gazebo",
    )


def create_isaac_transfer_sequence_description(
    requests: Sequence[MoveObject | TransferObjectIntent],
) -> LaunchDescription:
    """Create an Isaac-table request using the common V2 intent contract."""
    return create_simulator_transfer_sequence_description(
        requests,
        simulator="isaac",
    )
