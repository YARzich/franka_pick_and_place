"""
Test canonical one-action task launch construction without a simulator.

Responsibility:
- Preserve the simple public request surface and one-client launch shape.
- Verify launch-level failure propagation and repeated-object acceptance.
"""

import ast
from pathlib import Path
import runpy
from types import SimpleNamespace

from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from manipulation_msgs.msg import TransferObjectStepV2
import pytest

import franka_motion_runtime.tasks.launch as task_launch
import franka_motion_runtime.tasks.active_cancel_launch as active_cancel_launch
from franka_motion_runtime.tasks.model import (
    MoveObject,
    Pose,
    TransferObjectIntent,
)


_PACKAGE_ROOT = Path(__file__).resolve().parents[2]


def _request(*, x: float = 0.555) -> MoveObject:
    """Return one concise public request for task contract tests."""
    return MoveObject(
        object_id="cube",
        target=Pose(
            frame_id="panda_link0",
            position=(x, 0.0, 0.027),
        ),
    )


@pytest.mark.parametrize(
    ("simulator", "factory_name"),
    [
        ("gazebo", "create_gazebo_transfer_sequence_description"),
        ("isaac", "create_isaac_transfer_sequence_description"),
    ],
)
def test_factory_creates_one_package_local_sequence_client(
    simulator: str,
    factory_name: str,
) -> None:
    """Any request count maps to one action client and one public argument."""
    requests = (_request(), _request(x=0.405))
    node = task_launch._create_transfer_sequence_node(
        requests,
        use_sim_time=LaunchConfiguration("use_sim_time"),
    )
    assert node.node_package == "franka_motion"
    assert node.node_executable == "franka_transfer_sequence_client"

    factory = getattr(task_launch, factory_name)
    description = factory(requests)
    entities = list(description.entities)
    assert isinstance(entities[0], DeclareLaunchArgument)
    assert entities[0].name == "use_sim_time"
    assert len([item for item in entities if isinstance(item, RegisterEventHandler)]) == 1
    started_nodes = [item for item in entities if isinstance(item, Node)]
    assert len(started_nodes) == 1
    assert started_nodes[0].node_package == "franka_motion"
    assert started_nodes[0].node_executable == "franka_transfer_sequence_client"


def test_client_exit_is_propagated_to_launch() -> None:
    """A failed client fails the task; success shuts the request launch down."""
    context = SimpleNamespace(is_shutdown=False)
    failed = task_launch._actions_after_client(
        SimpleNamespace(returncode=2),
        context,
    )
    assert len(failed) == 1
    assert isinstance(failed[0], OpaqueFunction)
    with pytest.raises(RuntimeError, match="failed with code 2"):
        failed[0].execute(context)

    completed = task_launch._actions_after_client(
        SimpleNamespace(returncode=0),
        context,
    )
    assert len(completed) == 1
    assert isinstance(completed[0].event, Shutdown)


def test_active_cancel_factory_starts_only_package_local_probe() -> None:
    """The active-cancel leaf changes only the package-local client behavior."""
    description = active_cancel_launch.create_gazebo_active_cancel_description(
        (_request(),),
        controller_action_name="/test_arm/follow_joint_trajectory",
        persistent_world_object_ids=("table",),
    )
    nodes = [item for item in description.entities if isinstance(item, Node)]

    assert len(nodes) == 1
    assert nodes[0].node_executable == (
        "franka_transfer_sequence_active_cancel_probe"
    )


def test_empty_sequence_and_unknown_simulator_fail_before_launch() -> None:
    """No request process starts from an empty or unsupported task request."""
    with pytest.raises(ValueError, match="at least one"):
        task_launch.create_gazebo_transfer_sequence_description(())

    with pytest.raises(ValueError, match="unsupported task simulator"):
        task_launch.create_simulator_transfer_sequence_description(
            (_request(),),
            simulator="unknown",
        )


@pytest.mark.parametrize(
    ("filename", "factory_name"),
    [
        (
            "franka_gazebo_move_cube.launch.py",
            "create_gazebo_transfer_sequence_description",
        ),
        (
            "franka_gazebo_move_cube_round_trip.launch.py",
            "create_gazebo_transfer_sequence_description",
        ),
        (
            "franka_gazebo_active_cancel.launch.py",
            "create_gazebo_active_cancel_description",
        ),
        (
            "franka_isaac_move_cube.launch.py",
            "create_isaac_transfer_sequence_description",
        ),
        (
            "franka_isaac_move_cube_round_trip.launch.py",
            "create_isaac_transfer_sequence_description",
        ),
    ],
)
def test_public_leaf_contains_only_goals_and_factory_call(
    filename: str,
    factory_name: str,
) -> None:
    """Keep user-edited task files free of ROS process and backend construction."""
    path = _PACKAGE_ROOT / "launch" / filename
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    call_names = {
        node.func.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }
    assert "MoveObject" in call_names
    assert "Pose" in call_names
    assert factory_name in call_names
    assert "Node" not in call_names
    assert "ExecuteProcess" not in call_names


@pytest.mark.parametrize("simulator", ["gazebo", "isaac"])
def test_round_trip_leaf_remains_two_target_expressions(simulator: str) -> None:
    """Historical leaves stay usable without loading a client-side catalog."""
    namespace = runpy.run_path(
        _PACKAGE_ROOT
        / "launch"
        / f"franka_{simulator}_move_cube_round_trip.launch.py"
    )
    requests = namespace["REQUESTS"]

    assert len(requests) == 2
    assert isinstance(requests[0], MoveObject)
    assert isinstance(requests[1], TransferObjectIntent)
    assert requests[1].target_mode == TransferObjectStepV2.TARGET_INITIAL_SOURCE


def test_configured_node_parameters_contain_only_v2_intent(monkeypatch) -> None:
    """Launch serialization cannot leak source, grasp, geometry, or timeout."""
    monkeypatch.setattr(
        task_launch,
        "Node",
        lambda **kwargs: SimpleNamespace(**kwargs),
    )
    node = task_launch._create_transfer_sequence_node(
        (_request(),),
        use_sim_time=LaunchConfiguration("use_sim_time"),
    )
    parameters = node.parameters[0]

    assert parameters["steps.0.object_id"] == "cube"
    assert parameters["steps.0.target_mode"] == 0
    assert "steps.0.target_object_pose.position" in parameters
    assert not any(
        fragment in name
        for name in parameters
        for fragment in (
            "timeout",
            "dimensions",
            "source_object_pose",
            "grasp_pose",
            "provider",
            "strategy",
        )
    )


def test_detected_public_leaf_uses_two_v2_intents_and_one_client() -> None:
    """Detected round trip uses the generic V2 intent and client surface."""
    path = _PACKAGE_ROOT / "launch" / "franka_detected_cube_round_trip.launch.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    call_names = {
        node.func.id if isinstance(node.func, ast.Name) else node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        if isinstance(node.func, (ast.Name, ast.Attribute))
    }
    assert {
        "TranslationTarget",
        "translation_from_current_source",
        "initial_source",
        "create_transfer_sequence_description",
    } <= call_names
    assert {
        "DetectedPickPlaceRequest",
        "create_detected_pick_place_description",
        "Node",
        "ExecuteProcess",
    }.isdisjoint(call_names)

    namespace = runpy.run_path(path)
    requests = namespace["REQUESTS"]
    assert len(requests) == 2
    assert all(isinstance(request, TransferObjectIntent) for request in requests)
    assert [request.target_mode for request in requests] == [
        TransferObjectStepV2.TARGET_TRANSLATION_FROM_CURRENT_SOURCE,
        TransferObjectStepV2.TARGET_INITIAL_SOURCE,
    ]

    description = namespace["generate_launch_description"]()
    entities = list(description.entities)
    nodes = [item for item in entities if isinstance(item, Node)]
    assert len(nodes) == 1
    assert nodes[0].node_executable == "franka_transfer_sequence_client"
    assert len(
        [item for item in entities if isinstance(item, RegisterEventHandler)]
    ) == 1
