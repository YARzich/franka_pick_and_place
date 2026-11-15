"""
OmniGraph specs for Isaac ROS bridge runtime graphs.

Responsibility:
- Build testable graph specs for joint/clock and stereo camera ROS publishers.
- Apply those specs to OmniGraph only inside runtime functions.

Does not own:
- USD prim creation or Isaac World lifecycle.
- ROS 2 launch wiring outside the Isaac process.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from .config import StereoCameraConfig


@dataclass(frozen=True)
class PathListValue:
    """Deferred USD path converted only when applying an OmniGraph spec."""

    path: str


@dataclass(frozen=True)
class GraphEditSpec:
    """Pure, serializable description of one OmniGraph edit."""

    graph_path: str
    create_nodes: list[tuple[str, str]]
    connect: list[tuple[str, str]]
    set_values: list[tuple[str, Any]]
    evaluator_name: str = "execution"


def joint_bridge_graph_spec(
    *,
    articulation_path: str,
    command_topic: str,
    state_topic: str,
    clock_topic: str = "/clock",
) -> GraphEditSpec:
    """Describe the joint command/state and simulation-clock bridge graph."""
    return GraphEditSpec(
        graph_path="/franka_ros_bridge",
        create_nodes=[
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        connect=[
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
            ("Context.outputs:context", "PublishJointState.inputs:context"),
            ("Context.outputs:context", "SubscribeJointState.inputs:context"),
            ("Context.outputs:context", "PublishClock.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            (
                "SubscribeJointState.outputs:positionCommand",
                "ArticulationController.inputs:positionCommand",
            ),
            (
                "SubscribeJointState.outputs:velocityCommand",
                "ArticulationController.inputs:velocityCommand",
            ),
            (
                "SubscribeJointState.outputs:effortCommand",
                "ArticulationController.inputs:effortCommand",
            ),
        ],
        set_values=[
            ("ArticulationController.inputs:robotPath", articulation_path),
            ("PublishJointState.inputs:topicName", state_topic),
            ("SubscribeJointState.inputs:topicName", command_topic),
            ("PublishClock.inputs:topicName", clock_topic),
            ("PublishJointState.inputs:targetPrim", PathListValue(articulation_path)),
        ],
    )


def stereo_camera_graph_spec(
    *,
    stereo: StereoCameraConfig,
    left_camera_path: str,
    right_camera_path: str,
) -> GraphEditSpec:
    """Describe synchronized stereo image and CameraInfo publisher nodes."""
    return GraphEditSpec(
        graph_path="/franka_stereo_bridge",
        create_nodes=[
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("CreateRenderProductLeft", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("CreateRenderProductRight", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ("LeftImagePublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("RightImagePublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ],
        connect=[
            ("OnPlaybackTick.outputs:tick", "CreateRenderProductLeft.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "CreateRenderProductRight.inputs:execIn"),
            ("Context.outputs:context", "CameraInfoPublish.inputs:context"),
            ("Context.outputs:context", "LeftImagePublish.inputs:context"),
            ("Context.outputs:context", "RightImagePublish.inputs:context"),
            ("CreateRenderProductLeft.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
            ("CreateRenderProductRight.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
            (
                "CreateRenderProductLeft.outputs:renderProductPath",
                "CameraInfoPublish.inputs:renderProductPath",
            ),
            (
                "CreateRenderProductRight.outputs:renderProductPath",
                "CameraInfoPublish.inputs:renderProductPathRight",
            ),
            ("CreateRenderProductLeft.outputs:execOut", "LeftImagePublish.inputs:execIn"),
            (
                "CreateRenderProductLeft.outputs:renderProductPath",
                "LeftImagePublish.inputs:renderProductPath",
            ),
            ("CreateRenderProductRight.outputs:execOut", "RightImagePublish.inputs:execIn"),
            (
                "CreateRenderProductRight.outputs:renderProductPath",
                "RightImagePublish.inputs:renderProductPath",
            ),
        ],
        set_values=[
            ("CreateRenderProductLeft.inputs:cameraPrim", PathListValue(left_camera_path)),
            ("CreateRenderProductLeft.inputs:width", int(stereo.width)),
            ("CreateRenderProductLeft.inputs:height", int(stereo.height)),
            ("CreateRenderProductRight.inputs:cameraPrim", PathListValue(right_camera_path)),
            ("CreateRenderProductRight.inputs:width", int(stereo.width)),
            ("CreateRenderProductRight.inputs:height", int(stereo.height)),
            ("CameraInfoPublish.inputs:topicName", stereo.left_camera_info_topic),
            ("CameraInfoPublish.inputs:topicNameRight", stereo.right_camera_info_topic),
            ("CameraInfoPublish.inputs:frameId", stereo.left_frame_id),
            ("CameraInfoPublish.inputs:frameIdRight", stereo.right_frame_id),
            ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
            ("LeftImagePublish.inputs:topicName", stereo.left_image_topic),
            ("LeftImagePublish.inputs:frameId", stereo.left_frame_id),
            ("LeftImagePublish.inputs:type", "rgb"),
            ("LeftImagePublish.inputs:resetSimulationTimeOnStop", True),
            ("RightImagePublish.inputs:topicName", stereo.right_image_topic),
            ("RightImagePublish.inputs:frameId", stereo.right_frame_id),
            ("RightImagePublish.inputs:type", "rgb"),
            ("RightImagePublish.inputs:resetSimulationTimeOnStop", True),
        ],
    )


def apply_graph_spec(og, spec: GraphEditSpec, path_factory: Callable[[str], Any]) -> None:
    """Apply a pure graph specification through the provided OmniGraph API."""
    og.Controller.edit(
        {"graph_path": spec.graph_path, "evaluator_name": spec.evaluator_name},
        {
            og.Controller.Keys.CREATE_NODES: spec.create_nodes,
            og.Controller.Keys.CONNECT: spec.connect,
            og.Controller.Keys.SET_VALUES: [
                (name, _resolve_value(value, path_factory)) for name, value in spec.set_values
            ],
        },
    )


def create_joint_ros_graph(og, path_factory: Callable[[str], Any], *, articulation_path: str,
                           command_topic: str, state_topic: str) -> str:
    """Create the joint bridge graph and return its graph path."""
    spec = joint_bridge_graph_spec(
        articulation_path=articulation_path,
        command_topic=command_topic,
        state_topic=state_topic,
    )
    apply_graph_spec(og, spec, path_factory)
    return spec.graph_path


def create_stereo_ros_graph(og, path_factory: Callable[[str], Any], *, stereo: StereoCameraConfig,
                            left_camera_path: str, right_camera_path: str) -> str:
    """Create the stereo publication graph and return its graph path."""
    spec = stereo_camera_graph_spec(
        stereo=stereo,
        left_camera_path=left_camera_path,
        right_camera_path=right_camera_path,
    )
    apply_graph_spec(og, spec, path_factory)
    return spec.graph_path


def _resolve_value(value: Any, path_factory: Callable[[str], Any]) -> Any:
    if isinstance(value, PathListValue):
        return [path_factory(value.path)]
    return value
