"""Responsibility: Check actual TF buffers clear old epochs without DDS or a simulator."""

from pathlib import Path
import sys
from types import SimpleNamespace

from geometry_msgs.msg import TransformStamped
import pytest
from rclpy.clock import ROSClock
from rclpy.node import Node
from rclpy.time import Time
from rclpy.time_source import TimeSource
from rosgraph_msgs.msg import Clock

from manipulation_perception.common import stereo_transform_provider


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts' / 'pose'))
import foundationpose_detection_to_pose_node as pose_node  # noqa: E402,I100,I101,I201


@pytest.mark.parametrize('adapter', ['stereo', 'pose'])
def test_tf_buffer_discards_previous_ros_epoch(monkeypatch, adapter):
    clock = ROSClock()
    source = TimeSource()
    source.attach_clock(clock)
    source.ros_time_is_active = True
    source.clock_callback(Clock(clock=Time(seconds=100).to_msg()))
    logger = SimpleNamespace(info=lambda *_args: None)
    if adapter == 'stereo':
        monkeypatch.setattr(stereo_transform_provider, 'TransformListener', lambda *a, **k: None)
        owner = SimpleNamespace(get_clock=lambda: clock, create_service=lambda *a: None)
        provider = stereo_transform_provider.StereoTransformProvider(
            node=owner, allow_latest_fallback=False, logger=logger, frame_index=lambda: 1,
        )
        buffer = provider._buffer
    else:
        values = {
            'input_detections_topic': '/input', 'output_pose_topic': '/pose',
            'output_marker_topic': '/marker', 'output_frame': 'world',
            'marker_namespace': 'object', 'fallback_object_size_m': 0.1,
            'transform_timeout_sec': 0.0,
        }
        monkeypatch.setattr(Node, '__init__', lambda *a, **k: None)
        monkeypatch.setattr(Node, 'declare_parameter', lambda *a: None)
        monkeypatch.setattr(Node, 'get_parameter', lambda self, name: SimpleNamespace(value=values[name]))
        monkeypatch.setattr(Node, 'get_clock', lambda self: clock)
        monkeypatch.setattr(Node, 'get_logger', lambda self: logger)
        for method in ('create_service', 'create_subscription', 'create_publisher'):
            monkeypatch.setattr(Node, method, lambda *a, **k: None)
        monkeypatch.setattr(pose_node.PipelineObserver, 'create', lambda *a: None)
        monkeypatch.setattr(pose_node, 'TransformListener', lambda *a, **k: None)
        owner = pose_node.FoundationPoseDetectionToPoseNode()
        buffer = owner._tf_buffer

    transform = TransformStamped()
    transform.header.frame_id = 'world'
    transform.child_frame_id = 'camera'
    transform.header.stamp.sec = 90
    transform.transform.rotation.w = 1.0
    buffer.set_transform(transform, 'test')
    assert buffer.can_transform_core('world', 'camera', Time(seconds=90))[0]

    source.clock_callback(Clock(clock=Time(seconds=1).to_msg()))

    assert not buffer.can_transform_core('world', 'camera', Time(seconds=90))[0]
    transform.header.stamp.sec = 1
    buffer.set_transform(transform, 'test')
    assert buffer.can_transform_core('world', 'camera', Time(seconds=1))[0]
    buffer.jump_handle.unregister()
