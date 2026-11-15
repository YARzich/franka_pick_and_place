"""
Test ROS wiring of the MegaPose detection node.

Responsibility: Require stamped bbox input and delete stale visualization on invalid detections.
"""

from contextlib import contextmanager
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32MultiArray
from vision_msgs.msg import Detection2DArray, Detection3D, Detection3DArray


_SCRIPTS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SCRIPTS_DIR))
sys.path.insert(0, str(_SCRIPTS_DIR / 'scripts' / 'pose'))

import megapose_detection_node  # noqa: E402,I100,I101,I201
import foundationpose_detection_to_pose_node  # noqa: E402,I100,I101,I201


class _FakeStoppableBackend:
    def __init__(self) -> None:
        self.prewarm_calls = 0

    def start_prewarm(self) -> bool:
        self.prewarm_calls += 1
        return True

    def stop(self) -> None:
        pass


class _FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


@contextmanager
def _rclpy_context(args: list[str]):
    if rclpy.ok():
        rclpy.shutdown()
    rclpy.init(args=args)
    try:
        yield
    finally:
        rclpy.try_shutdown()


def _ros_args(parameters: dict[str, object]) -> list[str]:
    args = ['test_perception_node_wiring', '--ros-args']
    for name, value in parameters.items():
        args.extend(['-p', f'{name}:={value}'])
    return args


def _spy_node_wiring(monkeypatch, node_cls):
    subscriptions = []
    publishers = []
    original_create_subscription = Node.create_subscription
    original_create_publisher = Node.create_publisher

    def create_subscription_spy(self, msg_type, topic, callback, qos_profile, *args, **kwargs):
        subscriptions.append((msg_type, topic))
        return original_create_subscription(
            self,
            msg_type,
            topic,
            callback,
            qos_profile,
            *args,
            **kwargs,
        )

    def create_publisher_spy(self, msg_type, topic, qos_profile, *args, **kwargs):
        publishers.append((msg_type, topic))
        return original_create_publisher(
            self,
            msg_type,
            topic,
            qos_profile,
            *args,
            **kwargs,
        )

    monkeypatch.setattr(node_cls, 'create_subscription', create_subscription_spy)
    monkeypatch.setattr(node_cls, 'create_publisher', create_publisher_spy)
    return subscriptions, publishers


def _fake_pose_backend_runners(*_args, **_kwargs):
    return SimpleNamespace(
        megapose=_FakeStoppableBackend(),
        gigapose=_FakeStoppableBackend(),
        cleanup=lambda: None,
    )


@pytest.fixture(autouse=True)
def _shutdown_rclpy_after_test():
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_megapose_node_wires_stamped_bbox_without_legacy_bbox_subscription(
    monkeypatch,
    tmp_path: Path,
):
    mesh_path = tmp_path / 'cube.obj'
    mesh_path.write_text('v 0 0 0\nv 0.05 0.05 0.05\n')
    megapose_data_dir = tmp_path / 'megapose_data'
    megapose_data_dir.mkdir()
    monkeypatch.setattr(
        megapose_detection_node,
        'create_pose_backend_runners',
        _fake_pose_backend_runners,
    )
    subscriptions, publishers = _spy_node_wiring(
        monkeypatch,
        megapose_detection_node.MegaPoseDetectionNode,
    )

    node = None
    with _rclpy_context(
        _ros_args(
            {
                'image_topic': '/wiring/rgb',
                'camera_info_topic': '/wiring/camera_info',
                'depth_topic': '/wiring/depth',
                'detection_bbox_stamped_topic': '/wiring/stamped_bbox',
                'output_detections_topic': '/wiring/pose_output',
                'output_frame_id': 'camera_optical_frame',
                'object_label': 'test_object',
                'mesh_file_path': str(mesh_path),
                'megapose_python': sys.executable,
                'megapose_data_dir': str(megapose_data_dir),
            }
        )
    ):
        try:
            node = megapose_detection_node.MegaPoseDetectionNode()

            assert (Image, '/wiring/rgb') in subscriptions
            assert (CameraInfo, '/wiring/camera_info') in subscriptions
            assert (Image, '/wiring/depth') in subscriptions
            assert (Detection2DArray, '/wiring/stamped_bbox') in subscriptions
            assert (Detection3DArray, '/wiring/pose_output') in publishers
            assert all(msg_type is not Int32MultiArray for msg_type, _ in subscriptions)
            assert node._bbox_topic == '/wiring/stamped_bbox'
        finally:
            if node is not None:
                node.destroy_node()


@pytest.mark.parametrize(
    ('persistent_worker', 'expected_prewarm_calls'),
    [(True, 1), (False, 0)],
)
def test_gigapose_node_only_prewarms_persistent_workers(
    monkeypatch,
    tmp_path: Path,
    persistent_worker: bool,
    expected_prewarm_calls: int,
):
    mesh_path = tmp_path / 'cube.obj'
    mesh_path.write_text('v 0 0 0\nv 0.05 0.05 0.05\n')
    megapose_data_dir = tmp_path / 'megapose_data'
    megapose_data_dir.mkdir()
    gigapose_backend = _FakeStoppableBackend()
    monkeypatch.setattr(
        megapose_detection_node,
        'create_pose_backend_runners',
        lambda *_args, **_kwargs: SimpleNamespace(
            megapose=_FakeStoppableBackend(),
            gigapose=gigapose_backend,
            cleanup=lambda: None,
        ),
    )
    monkeypatch.setattr(
        megapose_detection_node,
        'validate_pose_node_parameters',
        lambda *_args, **_kwargs: None,
    )

    node = None
    with _rclpy_context(
        _ros_args(
            {
                'image_topic': '/wiring/rgb',
                'camera_info_topic': '/wiring/camera_info',
                'depth_topic': '/wiring/depth',
                'detection_bbox_stamped_topic': '/wiring/stamped_bbox',
                'detection_mask_topic': '/wiring/mask',
                'output_detections_topic': '/wiring/pose_output',
                'output_frame_id': 'camera_optical_frame',
                'coarse_estimator': 'gigapose',
                'gigapose_persistent_worker': persistent_worker,
                'object_label': 'test_object',
                'mesh_file_path': str(mesh_path),
                'megapose_python': sys.executable,
                'megapose_data_dir': str(megapose_data_dir),
            }
        )
    ):
        try:
            node = megapose_detection_node.MegaPoseDetectionNode()
            assert gigapose_backend.prewarm_calls == expected_prewarm_calls
        finally:
            if node is not None:
                node.destroy_node()


def test_empty_pose_detection_deletes_stale_markers_in_output_frame():
    node = object.__new__(
        foundationpose_detection_to_pose_node.FoundationPoseDetectionToPoseNode
    )
    node._output_frame = 'world'
    node._marker_namespace = 'test_object'
    node._marker_pub = _FakePublisher()
    node._observer = SimpleNamespace(
        metrics=SimpleNamespace(
            record_input=lambda: None,
            begin_processing=lambda: 0.0,
            record_drop_or_skip=lambda: None,
            record_processed=lambda _started_at: None,
        )
    )
    msg = Detection3DArray()
    msg.header.stamp.sec = 9
    msg.header.frame_id = 'table_stereo_left_optical_frame'

    node._on_detections(msg)

    markers = node._marker_pub.messages[0].markers
    assert [marker.id for marker in markers] == [0, 1]
    assert all(marker.action == marker.DELETE for marker in markers)
    assert all(marker.header.stamp.sec == 9 for marker in markers)
    assert all(marker.header.frame_id == 'world' for marker in markers)


def test_invalid_pose_detection_is_not_published_or_reported_as_success():
    node = object.__new__(
        foundationpose_detection_to_pose_node.FoundationPoseDetectionToPoseNode
    )
    node._output_frame = 'world'
    node._marker_namespace = 'test_object'
    node._marker_pub = _FakePublisher()
    node._pose_pub = _FakePublisher()
    successful_headers = []
    drops = []
    node._observer = SimpleNamespace(
        record_output_header=successful_headers.append,
        metrics=SimpleNamespace(
            record_input=lambda: None,
            begin_processing=lambda: 0.0,
            record_drop_or_skip=lambda: drops.append(True),
            record_processed=lambda _started_at: None,
        ),
    )
    msg = Detection3DArray()
    msg.header.stamp.sec = 9
    msg.header.frame_id = 'world'
    detection = Detection3D()
    detection.bbox.center.position.x = float('nan')
    msg.detections.append(detection)

    node._on_detections(msg)

    assert node._pose_pub.messages == []
    assert successful_headers == []
    assert drops == [True]
    assert all(
        marker.action == marker.DELETE
        for marker in node._marker_pub.messages[0].markers
    )
