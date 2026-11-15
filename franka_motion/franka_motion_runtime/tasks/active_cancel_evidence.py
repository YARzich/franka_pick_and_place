"""
Read post-cancel coordinator and PlanningScene evidence through ROS services.

Responsibility:
- Load bounded readback endpoints and waits from probe parameters.
- Poll typed read-only services until the pure cleanup contract converges.

Does not own:
- Action submission, cancellation, scenario geometry, or validation policy.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

from manipulation_msgs.srv import GetPlanningSceneState
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from franka_motion_runtime.tasks.active_cancel_validation import (
    CoordinatorStateSnapshot,
    PlanningSceneSnapshot,
    validate_coordinator_quiescence,
    validate_scene_cleanup,
)
from franka_motion_runtime.tasks.sequence_client_waits import (
    ShutdownRequest,
    wait_future,
)


@dataclass(frozen=True)
class ActiveCancelProbeConfig:
    """Hold probe-only waits and controller/read-only service endpoints."""

    feedback_timeout_sec: float
    cleanup_timeout_sec: float
    service_timeout_sec: float
    controller_action_name: str
    coordinator_state_service: str
    planning_scene_service: str
    persistent_world_object_ids: tuple[str, ...]


def load_active_cancel_probe_config(node: Node) -> ActiveCancelProbeConfig:
    """Declare and validate the probe-specific parameter surface."""
    values = {
        "feedback_timeout_sec": node.declare_parameter(
            "probe.feedback_timeout_sec", 60.0
        ).value,
        "cleanup_timeout_sec": node.declare_parameter(
            "probe.cleanup_timeout_sec", 15.0
        ).value,
        "service_timeout_sec": node.declare_parameter(
            "probe.service_timeout_sec", 2.0
        ).value,
        "controller_action_name": node.declare_parameter(
            "probe.controller_action_name", Parameter.Type.STRING
        ).value,
        "coordinator_state_service": node.declare_parameter(
            "probe.coordinator_state_service",
            "/planning_scene_coordinator/get_state",
        ).value,
        "planning_scene_service": node.declare_parameter(
            "probe.planning_scene_service", "/get_planning_scene"
        ).value,
        "persistent_world_object_ids": node.declare_parameter(
            "probe.persistent_world_object_ids",
            Parameter.Type.STRING_ARRAY,
        ).value,
    }
    waits = (
        float(values["feedback_timeout_sec"]),
        float(values["cleanup_timeout_sec"]),
        float(values["service_timeout_sec"]),
    )
    if not all(math.isfinite(value) and value > 0.0 for value in waits):
        raise ValueError("active-cancel probe waits must be finite and positive")
    controller_action_name = values["controller_action_name"]
    if not isinstance(controller_action_name, str) or not controller_action_name.strip():
        raise ValueError("active-cancel controller action name must not be empty")
    persistent_ids = tuple(values["persistent_world_object_ids"])
    if not persistent_ids or not all(
        isinstance(value, str) and value for value in persistent_ids
    ):
        raise ValueError("persistent world object IDs must be non-empty strings")
    coordinator_service = str(values["coordinator_state_service"])
    scene_service = str(values["planning_scene_service"])
    if not coordinator_service or not scene_service:
        raise ValueError("active-cancel probe service names must not be empty")
    return ActiveCancelProbeConfig(
        feedback_timeout_sec=waits[0],
        cleanup_timeout_sec=waits[1],
        service_timeout_sec=waits[2],
        controller_action_name=controller_action_name,
        coordinator_state_service=coordinator_service,
        planning_scene_service=scene_service,
        persistent_world_object_ids=persistent_ids,
    )


def _call_service(
    node,
    client,
    request,
    *,
    timeout_sec: float,
    shutdown: ShutdownRequest,
):
    """Perform one bounded read-only service call or return no evidence."""
    deadline = time.monotonic() + timeout_sec
    if not client.wait_for_service(timeout_sec=timeout_sec):
        return None
    remaining = deadline - time.monotonic()
    if remaining <= 0.0:
        return None
    future = client.call_async(request)
    if not wait_future(
        node,
        future,
        timeout_sec=remaining,
        shutdown=shutdown,
    ):
        client.remove_pending_request(future)
        return None
    try:
        return future.result()
    except Exception:
        return None


def _coordinator_snapshot(response) -> CoordinatorStateSnapshot | None:
    """Copy a successful coordinator response into pure evidence."""
    if response is None or not response.result.success:
        return None
    state = response.state
    return CoordinatorStateSnapshot(
        active_lease_count=state.active_lease_count,
        operation_in_progress=state.operation_in_progress,
        backend_service_ready=state.backend_service_ready,
        reconciliation_required=state.reconciliation_required,
        unreconciled_lease_expiry_count=(
            state.unreconciled_lease_expiry_count
        ),
        operation_capacity_exhausted=state.operation_capacity_exhausted,
        quarantined_resource_count=state.quarantined_resource_count,
        quarantined_resource_keys=tuple(state.quarantined_resource_keys),
    )


def _scene_snapshot(response) -> PlanningSceneSnapshot | None:
    """Copy MoveIt object identities into pure evidence."""
    if response is None:
        return None
    scene = response.scene
    return PlanningSceneSnapshot(
        world_object_ids=frozenset(
            item.id for item in scene.world.collision_objects
        ),
        attached_object_ids=frozenset(
            item.object.id
            for item in scene.robot_state.attached_collision_objects
        ),
    )


def _scene_request() -> GetPlanningScene.Request:
    """Request only object identities needed by the cleanup contract."""
    request = GetPlanningScene.Request()
    request.components.components = (
        PlanningSceneComponents.WORLD_OBJECT_NAMES
        | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
    )
    return request


def wait_for_cleanup_evidence(
    node: Node,
    *,
    canceled_object_id: str,
    probe_config: ActiveCancelProbeConfig,
    shutdown: ShutdownRequest,
) -> tuple[str, ...]:
    """Poll authoritative state and scene until all cancel cleanup converges."""
    state_client = node.create_client(
        GetPlanningSceneState,
        probe_config.coordinator_state_service,
    )
    scene_client = node.create_client(
        GetPlanningScene,
        probe_config.planning_scene_service,
    )
    deadline = time.monotonic() + probe_config.cleanup_timeout_sec
    last_errors: tuple[str, ...] = ("post-cancel evidence was not sampled",)
    try:
        while not shutdown.requested:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            state_timeout = min(probe_config.service_timeout_sec, remaining)
            state = _coordinator_snapshot(
                _call_service(
                    node,
                    state_client,
                    GetPlanningSceneState.Request(),
                    timeout_sec=state_timeout,
                    shutdown=shutdown,
                )
            )
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                last_errors = (
                    "cleanup deadline expired before PlanningScene readback",
                )
                break
            scene_timeout = min(probe_config.service_timeout_sec, remaining)
            scene = _scene_snapshot(
                _call_service(
                    node,
                    scene_client,
                    _scene_request(),
                    timeout_sec=scene_timeout,
                    shutdown=shutdown,
                )
            )
            errors: list[str] = []
            if state is None:
                errors.append("coordinator state service returned no valid evidence")
            else:
                errors.extend(validate_coordinator_quiescence(state))
            if scene is None:
                errors.append("MoveIt PlanningScene service returned no evidence")
            else:
                errors.extend(
                    validate_scene_cleanup(
                        scene,
                        canceled_object_id=canceled_object_id,
                        persistent_world_object_ids=(
                            probe_config.persistent_world_object_ids
                        ),
                    )
                )
            last_errors = tuple(errors)
            if not last_errors:
                return ()
            rclpy.spin_once(
                node,
                timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())),
            )
        return last_errors
    finally:
        node.destroy_client(scene_client)
        node.destroy_client(state_client)
