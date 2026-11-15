"""
Test construction and publication policy of stereo ROS outputs.

Responsibility: Verify message frames, encodings, QoS, markers, and publication gating.
"""

from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import MarkerArray


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.common.stereo_ros_outputs import StereoRosOutputs  # noqa: E402,I100,I101,I201


class _Publisher:
    def __init__(self, message_type, topic, qos) -> None:
        self.message_type = message_type
        self.topic = topic
        self.qos = qos
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class _Node:
    def __init__(self) -> None:
        self.publishers = []
        self.destroyed = []

    def create_publisher(self, message_type, topic, qos):
        publisher = _Publisher(message_type, topic, qos)
        self.publishers.append(publisher)
        return publisher

    def destroy_publisher(self, publisher) -> None:
        self.destroyed.append(publisher)


class _FailingNode(_Node):
    def __init__(self, fail_on_call: int) -> None:
        super().__init__()
        self._fail_on_call = fail_on_call
        self._create_calls = 0

    def create_publisher(self, message_type, topic, qos):
        self._create_calls += 1
        if self._create_calls == self._fail_on_call:
            raise RuntimeError('publisher creation failed')
        return super().create_publisher(message_type, topic, qos)


def _parameters(**overrides):
    values = {
        'disparity_topic': '/stereo/disparity',
        'depth_topic': '/stereo/depth',
        'point_cloud_topic': '/stereo/points',
        'octomap_point_cloud_topic': '/stereo/octomap_points',
        'publish_cluster_markers': True,
        'cluster_marker_topic': '/stereo/clusters',
        'publish_table_plane_marker': True,
        'table_plane_marker_topic': '/stereo/table',
        'table_plane_coeffs_topic': '/stereo/table_coeffs',
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_stereo_outputs_create_owned_topics_and_qos():
    node = _Node()

    outputs = StereoRosOutputs.create(node, _parameters())

    by_topic = {publisher.topic: publisher for publisher in node.publishers}
    assert by_topic['/stereo/disparity'].message_type is Image
    assert by_topic['/stereo/depth'].qos.reliability == ReliabilityPolicy.RELIABLE
    assert by_topic['/stereo/points'].message_type is PointCloud2
    assert by_topic['/stereo/octomap_points'].message_type is PointCloud2
    assert (
        by_topic['/stereo/octomap_points'].qos.reliability
        == ReliabilityPolicy.BEST_EFFORT
    )
    assert by_topic['/stereo/clusters'].qos.durability == DurabilityPolicy.VOLATILE
    assert by_topic['/stereo/table'].message_type is MarkerArray
    assert by_topic['/stereo/table'].qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert by_topic['/stereo/table_coeffs'].message_type is Float64MultiArray
    assert by_topic['/stereo/table_coeffs'].qos.depth == 1
    assert outputs.has_table_plane_outputs is True


def test_stereo_outputs_skip_disabled_optional_publishers():
    node = _Node()

    outputs = StereoRosOutputs.create(
        node,
        _parameters(publish_cluster_markers=False, publish_table_plane_marker=False),
    )

    assert outputs.cluster_markers is None
    assert outputs.table_plane_marker is None
    assert outputs.table_plane_coeffs is not None
    assert {publisher.topic for publisher in node.publishers} == {
        '/stereo/disparity',
        '/stereo/depth',
        '/stereo/octomap_points',
        '/stereo/points',
        '/stereo/table_coeffs',
    }


def test_stereo_outputs_publish_results_and_destroy_once():
    node = _Node()
    outputs = StereoRosOutputs.create(node, _parameters())
    disparity = Image()
    depth = Image()
    cloud = PointCloud2()
    clusters = MarkerArray()
    marker = MarkerArray()
    coeffs = Float64MultiArray()

    outputs.publish_disparity(disparity)
    outputs.publish_depth(depth)
    outputs.publish_octomap_cloud(cloud)
    outputs.publish_cloud(cloud, clusters)
    outputs.publish_table_plane(marker, coeffs)

    assert outputs.disparity.messages == [disparity]
    assert outputs.depth.messages == [depth]
    assert outputs.octomap_point_cloud.messages == [cloud]
    assert outputs.point_cloud.messages == [cloud]
    assert outputs.cluster_markers.messages == [clusters]
    assert outputs.table_plane_marker.messages == [marker]
    assert outputs.table_plane_coeffs.messages == [coeffs]

    publishers = tuple(node.publishers)
    outputs.destroy(node)
    outputs.destroy(node)

    assert set(node.destroyed) == set(publishers)
    assert len(node.destroyed) == len(publishers)


def test_stereo_outputs_roll_back_partial_publisher_creation():
    node = _FailingNode(fail_on_call=5)

    with pytest.raises(RuntimeError, match='publisher creation failed'):
        StereoRosOutputs.create(node, _parameters())

    assert node.destroyed == list(reversed(node.publishers))
