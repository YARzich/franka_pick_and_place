"""
Test ROS wiring of the stereo depth and point-cloud node.

Responsibility: Verify exact-time subscriptions, callbacks, parameters, publishers, and shutdown.
"""

from contextlib import contextmanager
from pathlib import Path
import sys
from types import SimpleNamespace

from message_filters import TimeSynchronizer
import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Float64MultiArray, Int32MultiArray
from stereo_msgs.msg import DisparityImage
from vision_msgs.msg import Detection2DArray


_SCRIPTS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SCRIPTS_DIR))
sys.path.insert(0, str(_SCRIPTS_DIR / 'scripts' / 'common'))

import stereo_sgbm_pointcloud_node  # noqa: E402,I100,I101,I201
from manipulation_perception.common.stereo_input_processor import StereoInputProcessor  # noqa: E402,I100,I101,I201


class _FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


class _FakeLogger:
    def __init__(self) -> None:
        self.errors = []

    def error(self, message: str, **_kwargs) -> None:
        self.errors.append(message)


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


def _required_stereo_parameters() -> dict[str, object]:
    return {
        'left_image_topic': '/camera/left/image',
        'right_image_topic': '/camera/right/image',
        'left_camera_info_topic': '/camera/left/camera_info',
        'left_image_frame_id': 'left_camera_optical_frame',
        'right_image_frame_id': 'right_camera_optical_frame',
        'local_stereo_frame_id': 'left_camera_optical_frame',
        'disparity_topic': '/stereo/disparity',
        'depth_topic': '/stereo/depth',
        'point_cloud_topic': '/stereo/points',
        'octomap_point_cloud_topic': '/stereo/octomap_points',
        'point_cloud_frame': 'world',
        'workspace_frame': 'world',
        'robot_base_frame': 'robot_base',
        'baseline_m': 0.06,
        'robot_reach_m': 1.0,
        'table_plane_coeffs_topic': '/stereo/plane_coeffs',
    }


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


def _spy_node_timers(monkeypatch, node_cls):
    timers = []
    original_create_timer = Node.create_timer

    def create_timer_spy(self, timer_period_sec, callback, *args, **kwargs):
        timers.append((timer_period_sec, callback))
        return original_create_timer(
            self,
            timer_period_sec,
            callback,
            *args,
            **kwargs,
        )

    monkeypatch.setattr(node_cls, 'create_timer', create_timer_spy)
    return timers


def _image_msg(
    height: int = 10,
    width: int = 20,
    frame_id: str = 'table_stereo_left_image_frame',
) -> Image:
    msg = Image()
    msg.height = height
    msg.width = width
    msg.header.stamp.sec = 7
    msg.header.stamp.nanosec = 11
    msg.header.frame_id = frame_id
    return msg


def _external_disparity_msg(*, focal_length: float, baseline: float) -> DisparityImage:
    msg = DisparityImage()
    msg.header.stamp.sec = 7
    msg.header.stamp.nanosec = 11
    msg.header.frame_id = 'table_stereo_left_optical_frame'
    msg.image.header = msg.header
    msg.image.height = 1
    msg.image.width = 2
    msg.image.encoding = '32FC1'
    msg.image.step = 8
    msg.image.data = np.array([2.0, 4.0], dtype='<f4').tobytes()
    msg.f = focal_length
    msg.t = baseline
    return msg


def _matching_camera_info(*, focal_length: float) -> CameraInfo:
    msg = CameraInfo()
    msg.header.stamp.sec = 7
    msg.header.stamp.nanosec = 11
    msg.header.frame_id = 'table_stereo_left_optical_frame'
    msg.height = 1
    msg.width = 2
    msg.k = [focal_length, 0.0, 0.5, 0.0, 120.0, 0.0, 0.0, 0.0, 1.0]
    return msg


def _local_camera_info(image: Image, *, focal_length: float = 100.0) -> CameraInfo:
    msg = CameraInfo()
    msg.header = image.header
    msg.height = image.height
    msg.width = image.width
    msg.k = [focal_length, 0.0, 1.0, 0.0, focal_length, 1.0, 0.0, 0.0, 1.0]
    return msg


@pytest.fixture(autouse=True)
def _shutdown_rclpy_after_test():
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_stereo_node_wires_detection_roi_to_stamped_bbox(monkeypatch):
    subscriptions, publishers = _spy_node_wiring(
        monkeypatch,
        stereo_sgbm_pointcloud_node.StereoSgbmPointCloudNode,
    )
    timers = _spy_node_timers(
        monkeypatch,
        stereo_sgbm_pointcloud_node.StereoSgbmPointCloudNode,
    )

    node = None
    with _rclpy_context(
        _ros_args(
            _required_stereo_parameters() | {
                'use_detection_bbox_roi': 'true',
                'detection_bbox_stamped_topic': '/wiring/stamped_bbox',
                'publish_table_plane_marker': 'false',
                'publish_cluster_markers': 'false',
            }
        )
    ):
        try:
            node = stereo_sgbm_pointcloud_node.StereoSgbmPointCloudNode()

            assert (Detection2DArray, '/wiring/stamped_bbox') in subscriptions
            assert all(msg_type is not Int32MultiArray for msg_type, _ in subscriptions)
            assert any(topic == '/stereo/depth' for msg_type, topic in publishers if msg_type is Image)
            assert any(topic == '/stereo/points' for msg_type, topic in publishers if msg_type is PointCloud2)
            assert any(
                topic == '/stereo/octomap_points'
                for msg_type, topic in publishers
                if msg_type is PointCloud2
            )
            assert any(
                topic == '/stereo/plane_coeffs'
                for msg_type, topic in publishers
                if msg_type is Float64MultiArray
            )
            assert any(
                callback.__name__ == 'process_pending_depth_frames'
                for _, callback in timers
            )
            assert isinstance(node._transport._synchronizer, TimeSynchronizer)
        finally:
            if node is not None:
                node.destroy_node()


def test_external_disparity_wires_exact_camera_info_synchronizer(monkeypatch):
    subscriptions, _publishers = _spy_node_wiring(
        monkeypatch,
        stereo_sgbm_pointcloud_node.StereoSgbmPointCloudNode,
    )

    node = None
    with _rclpy_context(
        _ros_args(
            _required_stereo_parameters() | {
                'use_external_disparity': 'true',
                'left_camera_info_topic': '/wiring/camera_info',
                'external_disparity_topic': '/wiring/disparity',
                'publish_table_plane_marker': 'false',
                'publish_cluster_markers': 'false',
            }
        )
    ):
        try:
            node = stereo_sgbm_pointcloud_node.StereoSgbmPointCloudNode()

            assert (CameraInfo, '/wiring/camera_info') in subscriptions
            assert (DisparityImage, '/wiring/disparity') in subscriptions
            assert isinstance(node._transport._synchronizer, TimeSynchronizer)
        finally:
            if node is not None:
                node.destroy_node()


def test_local_stereo_rejects_output_frame_different_from_left_sensor_frame():
    parameters = _required_stereo_parameters()
    parameters['local_stereo_frame_id'] = 'different_optical_frame'
    parameters['publish_table_plane_marker'] = 'false'
    parameters['publish_cluster_markers'] = 'false'

    with _rclpy_context(_ros_args(parameters)):
        with pytest.raises(ValueError, match='must equal left_image_frame_id'):
            stereo_sgbm_pointcloud_node.StereoSgbmPointCloudNode()


def test_local_stereo_output_uses_configured_optical_frame():
    bridge = SimpleNamespace(
        imgmsg_to_cv2=lambda *_args, **_kwargs: SimpleNamespace(),
        cv2_to_imgmsg=lambda *_args, **_kwargs: Image(),
    )
    disparity_pub = _FakePublisher()
    processed = []
    depth_coordinator = SimpleNamespace(
        process_depth=lambda depth, header, *, started_at, intrinsics=None: processed.append(
            (depth, header, started_at, intrinsics)
        ),
    )
    processor = StereoInputProcessor(
        parameters=SimpleNamespace(
            local_stereo_frame_id='custom_left_optical_frame',
            left_image_frame_id='custom_left_optical_frame',
            right_image_frame_id='custom_right_optical_frame',
            baseline_m=0.06,
            min_valid_disparity_px=0.1,
            min_depth_m=0.1,
            max_depth_m=5.0,
            use_external_disparity=True,
        ),
        bridge=bridge,
        outputs=SimpleNamespace(publish_disparity=disparity_pub.publish),
        depth_coordinator=depth_coordinator,
        logger=_FakeLogger(),
        record_processed=lambda _started_at: None,
        record_error=lambda: None,
    )
    processor._sgbm_estimator = SimpleNamespace(
        compute_disparity=lambda *_args, **_kwargs: (
            np.ones((2, 3), dtype=np.float32),
            None,
        ),
        ecc_fail_count=0,
    )

    left_msg = _image_msg(height=2, width=3, frame_id='custom_left_optical_frame')
    right_msg = _image_msg(height=2, width=3, frame_id='custom_right_optical_frame')
    camera_info = _local_camera_info(left_msg)
    processor.process_stereo_pair(
        left_msg,
        right_msg,
        camera_info,
        frame_index=1,
        started_at=10.0,
    )

    assert left_msg.header.frame_id == 'custom_left_optical_frame'
    assert disparity_pub.messages[0].header.stamp == left_msg.header.stamp
    assert disparity_pub.messages[0].header.frame_id == 'custom_left_optical_frame'
    assert processed[0][1].stamp == left_msg.header.stamp
    assert processed[0][1].frame_id == 'custom_left_optical_frame'
    assert processed[0][2] == 10.0
    assert processed[0][3] == (100.0, 100.0, 1.0, 1.0)


def test_external_disparity_uses_sample_geometry_for_depth_and_projection_intrinsics():
    processed = []
    processor = StereoInputProcessor(
        parameters=SimpleNamespace(
            local_stereo_frame_id='table_stereo_left_optical_frame',
            baseline_m=0.06,
            min_valid_disparity_px=0.1,
            min_depth_m=0.1,
            max_depth_m=20.0,
            use_external_disparity=True,
        ),
        bridge=SimpleNamespace(),
        outputs=SimpleNamespace(),
        depth_coordinator=SimpleNamespace(
            focal_length_x=100.0,
            process_depth=lambda depth, header, **kwargs: processed.append(
                (depth, header, kwargs)
            ),
        ),
        logger=_FakeLogger(),
        record_processed=lambda _started_at: None,
        record_error=lambda: None,
    )

    processor.process_external_disparity(
        _external_disparity_msg(focal_length=240.0, baseline=0.08),
        _matching_camera_info(focal_length=100.0),
        started_at=10.0,
    )

    assert len(processed) == 1
    depth, header, kwargs = processed[0]
    np.testing.assert_allclose(depth, np.array([[9.6, 4.8]], dtype=np.float32))
    assert header.frame_id == 'table_stereo_left_optical_frame'
    assert kwargs['intrinsics'] == (240.0, 120.0, 0.5, 0.0)
    assert kwargs['started_at'] == 10.0
