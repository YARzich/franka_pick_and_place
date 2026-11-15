"""
Test the canonical Isaac OmniGraph specifications and stable ROS contracts.

Responsibility: Preserve joint and stereo topics, frames, articulation paths, and value types.
"""

from franka_motion_runtime.simulation.isaac.runtime.config import StereoCameraConfig
from franka_motion_runtime.simulation.isaac.runtime.ros_graph import (
    PathListValue,
    apply_graph_spec,
    joint_bridge_graph_spec,
    stereo_camera_graph_spec,
)


def test_joint_bridge_graph_spec_keeps_topics_and_articulation_path():
    spec = joint_bridge_graph_spec(
        articulation_path='/World/Franka',
        command_topic='/isaac_joint_commands',
        state_topic='/isaac_joint_states',
    )
    values = dict(spec.set_values)

    assert spec.graph_path == '/franka_ros_bridge'
    assert ('PublishClock', 'isaacsim.ros2.bridge.ROS2PublishClock') in spec.create_nodes
    assert values['ArticulationController.inputs:robotPath'] == '/World/Franka'
    assert values['PublishJointState.inputs:topicName'] == '/isaac_joint_states'
    assert values['SubscribeJointState.inputs:topicName'] == '/isaac_joint_commands'
    assert values['PublishClock.inputs:topicName'] == '/clock'
    assert values['PublishJointState.inputs:targetPrim'] == PathListValue('/World/Franka')


def test_stereo_camera_graph_spec_keeps_camera_topics_frames_and_resolution():
    stereo = StereoCameraConfig(
        width=640,
        height=480,
        left_image_topic='/left/image',
        right_image_topic='/right/image',
        left_camera_info_topic='/left/info',
        right_camera_info_topic='/right/info',
        left_frame_id='left_optical',
        right_frame_id='right_optical',
    )

    spec = stereo_camera_graph_spec(
        stereo=stereo,
        left_camera_path='/World/stereo/left',
        right_camera_path='/World/stereo/right',
    )
    values = dict(spec.set_values)

    assert spec.graph_path == '/franka_stereo_bridge'
    assert values['CreateRenderProductLeft.inputs:cameraPrim'] == PathListValue('/World/stereo/left')
    assert values['CreateRenderProductRight.inputs:cameraPrim'] == PathListValue('/World/stereo/right')
    assert values['CreateRenderProductLeft.inputs:width'] == 640
    assert values['CreateRenderProductRight.inputs:height'] == 480
    assert values['CameraInfoPublish.inputs:topicName'] == '/left/info'
    assert values['CameraInfoPublish.inputs:topicNameRight'] == '/right/info'
    assert values['LeftImagePublish.inputs:frameId'] == 'left_optical'
    assert values['RightImagePublish.inputs:topicName'] == '/right/image'


def test_apply_graph_spec_converts_path_list_values():
    spec = joint_bridge_graph_spec(
        articulation_path='/World/Franka',
        command_topic='/cmd',
        state_topic='/state',
    )
    fake_og = _FakeOg()

    apply_graph_spec(fake_og, spec, lambda path: f'PATH:{path}')

    assert fake_og.edits[0][0] == {'graph_path': '/franka_ros_bridge', 'evaluator_name': 'execution'}
    values = dict(fake_og.edits[0][1]['set_values'])
    assert values['PublishJointState.inputs:targetPrim'] == ['PATH:/World/Franka']


class _FakeKeys:
    CREATE_NODES = 'create_nodes'
    CONNECT = 'connect'
    SET_VALUES = 'set_values'


class _FakeController:
    Keys = _FakeKeys

    def __init__(self) -> None:
        self.edits = []

    def edit(self, graph_config, graph_spec) -> None:
        self.edits.append((graph_config, graph_spec))


class _FakeOg:
    Controller = _FakeController()

    @property
    def edits(self):
        return self.Controller.edits
