"""
Test ROS wiring and failure containment of the YOLO detection node.

Responsibility: Verify stamped outputs, shutdown guards, frame overrides, and empty detections.
"""

from contextlib import contextmanager
from pathlib import Path
import signal
import sys
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from vision_msgs.msg import Detection2DArray


_SCRIPTS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SCRIPTS_DIR))
sys.path.insert(0, str(_SCRIPTS_DIR / 'scripts' / 'yolo'))

import yolo_visualizer_node  # noqa: E402,I100,I101,I201
from manipulation_perception.common.detection_candidate import (  # noqa: E402,I100,I101,I201
    DetectionCandidate,
)
from manipulation_perception.yolo.yolo_frame_processor import (  # noqa: E402,I100,I101,I201
    YoloFrameProcessor,
)
from manipulation_perception.yolo.yolo_output_adapter import (  # noqa: E402,I100,I101,I201
    YoloOutputAdapter,
)


class _FakeYoloDetector:
    has_sahi_model = False

    def __init__(self, **_kwargs) -> None:
        pass


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


class _FailingBridge:
    def __init__(self, message: str) -> None:
        self._message = message

    def imgmsg_to_cv2(self, *_args, **_kwargs):
        raise RuntimeError(self._message)


class _FrameBridge:
    def __init__(self, frame) -> None:
        self._frame = frame

    def imgmsg_to_cv2(self, *_args, **_kwargs):
        return self._frame


class _FailingDetector:
    def __init__(self, message: str) -> None:
        self._message = message

    def detect(self, *_args, **_kwargs):
        raise RuntimeError(self._message)


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


def _fake_yolo_runtime(*_args, **_kwargs):
    return SimpleNamespace(
        model=SimpleNamespace(names={0: 'cube'}),
        mask_model=None,
        sahi_model=None,
        get_sliced_prediction=None,
        mask_model_available=False,
    )


def _minimal_yolo_callback_node(monkeypatch, *, bridge, detector):
    monkeypatch.setattr(yolo_visualizer_node.rclpy, 'ok', lambda: True)
    node = object.__new__(yolo_visualizer_node.YoloVisualizerNode)
    logger = _FakeLogger()
    node.get_logger = lambda: logger
    node._shutting_down = False
    node._sync_runtime_controls = lambda: None
    node._runtime_controller = SimpleNamespace(
        state=SimpleNamespace(run_every_n_frames=1),
        build_detector_config=lambda: SimpleNamespace(),
    )
    node._bridge = bridge
    node._detector = detector
    node._bbox_pub = _FakePublisher()
    node._bbox_stamped_pub = _FakePublisher()
    node._mask_pub = None
    node._outputs = YoloOutputAdapter(
        bridge=bridge,
        bbox_publisher=node._bbox_pub,
        stamped_bbox_publisher=node._bbox_stamped_pub,
        mask_publisher=None,
        bbox_frame_id='table_stereo_left_optical_frame',
        mask_fill_value=255,
    )
    node._debug_session = SimpleNamespace(
        sync_controls=lambda *_args: node._sync_runtime_controls(),
        render=lambda *_args: None,
    )
    node._frame_processor = YoloFrameProcessor(
        bridge=bridge,
        detector=detector,
        runtime_controller=node._runtime_controller,
        outputs=node._outputs,
        debug_session=node._debug_session,
        logger=logger,
        record_input=lambda: None,
        record_drop_or_skip=lambda: None,
        begin_processing=lambda: 0.0,
        record_processed=lambda _started_at: None,
        record_error=lambda: None,
    )
    return node, logger


def _image_msg(height: int = 10, width: int = 20) -> Image:
    msg = Image()
    msg.height = height
    msg.width = width
    msg.header.stamp.sec = 7
    msg.header.stamp.nanosec = 11
    msg.header.frame_id = 'table_stereo_left_image_frame'
    return msg


def _raise_runtime_controls_error() -> None:
    raise RuntimeError('controls failed')


@pytest.fixture(autouse=True)
def _shutdown_rclpy_after_test():
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_yolo_node_wires_legacy_and_stamped_bbox_publishers(monkeypatch, tmp_path):
    monkeypatch.setattr(yolo_visualizer_node, 'load_yolo_runtime', _fake_yolo_runtime)
    monkeypatch.setattr(yolo_visualizer_node, 'HeadlessYoloDetector', _FakeYoloDetector)
    subscriptions, publishers = _spy_node_wiring(
        monkeypatch,
        yolo_visualizer_node.YoloVisualizerNode,
    )
    model_path = tmp_path / 'detector.pt'
    model_path.write_text('test model')

    node = None
    with _rclpy_context(
        _ros_args(
            {
                'image_topic': '/wiring/rgb',
                'model_path': str(model_path),
                'detection_bbox_topic': '/wiring/legacy_bbox',
                'detection_bbox_stamped_topic': '/wiring/stamped_bbox',
                'publish_detection_bbox': 'true',
                'publish_detection_mask': 'true',
                'detection_mask_topic': '/wiring/mask',
                'enable_window': 'false',
                'enable_runtime_controls': 'false',
            }
        )
    ):
        try:
            node = yolo_visualizer_node.YoloVisualizerNode()

            assert (Image, '/wiring/rgb') in subscriptions
            assert (Int32MultiArray, '/wiring/legacy_bbox') in publishers
            assert (Detection2DArray, '/wiring/stamped_bbox') in publishers
            assert (Image, '/wiring/mask') in publishers
            assert any(topic == '/diagnostics' for _, topic in publishers)
            assert node._observer.metrics.snapshot().input_count == 0
        finally:
            if node is not None:
                node.destroy_node()


def test_yolo_request_shutdown_sets_callback_guard():
    node = object.__new__(yolo_visualizer_node.YoloVisualizerNode)
    node._shutting_down = False

    node.request_shutdown()

    assert node._shutting_down is True


def test_yolo_signal_handler_requests_shutdown_and_stops_rclpy(monkeypatch):
    handlers = {}
    requested = []

    monkeypatch.setattr(
        yolo_visualizer_node.signal,
        'signal',
        lambda signum, handler: handlers.setdefault(signum, handler),
    )
    monkeypatch.setattr(
        yolo_visualizer_node.rclpy,
        'try_shutdown',
        lambda: requested.append('rclpy_shutdown'),
    )
    node = SimpleNamespace(request_shutdown=lambda: requested.append('node_shutdown'))

    yolo_visualizer_node._install_shutdown_signal_handlers(node)

    assert signal.SIGINT in handlers
    assert signal.SIGTERM in handlers
    with pytest.raises(KeyboardInterrupt):
        handlers[signal.SIGTERM](signal.SIGTERM, None)
    assert requested == ['node_shutdown', 'rclpy_shutdown']


def test_yolo_publish_best_bbox_preserves_stamp_and_overrides_frame_id():
    legacy_pub = _FakePublisher()
    stamped_pub = _FakePublisher()
    source_msg = Image()
    source_msg.header.stamp.sec = 7
    source_msg.header.stamp.nanosec = 11
    source_msg.header.frame_id = 'table_stereo_left_image_frame'

    outputs = YoloOutputAdapter(
        bridge=SimpleNamespace(),
        bbox_publisher=legacy_pub,
        stamped_bbox_publisher=stamped_pub,
        mask_publisher=None,
        bbox_frame_id='table_stereo_left_optical_frame',
        mask_fill_value=255,
    )
    outputs.publish(
        [DetectionCandidate((1.0, 2.0, 5.0, 6.0), 0.8, 3)],
        (10, 20, 3),
        source_msg,
    )

    assert len(legacy_pub.messages) == 1
    assert list(legacy_pub.messages[0].data) == [1, 2, 5, 6, 20, 10, 3, 800]
    assert len(stamped_pub.messages) == 1
    stamped_msg = stamped_pub.messages[0]
    assert stamped_msg.header.stamp.sec == 7
    assert stamped_msg.header.stamp.nanosec == 11
    assert stamped_msg.header.frame_id == 'table_stereo_left_optical_frame'
    assert stamped_msg.detections[0].header.frame_id == 'table_stereo_left_optical_frame'


def test_yolo_image_callback_publishes_empty_bbox_when_image_decode_fails(monkeypatch):
    node, logger = _minimal_yolo_callback_node(
        monkeypatch,
        bridge=_FailingBridge('bad image'),
        detector=SimpleNamespace(detect=lambda *_args, **_kwargs: []),
    )

    node._image_callback(_image_msg())

    assert any('Не удалось декодировать RGB для YOLO' in error for error in logger.errors)
    assert len(node._bbox_pub.messages) == 1
    assert list(node._bbox_pub.messages[0].data) == []
    assert len(node._bbox_stamped_pub.messages) == 1
    stamped_msg = node._bbox_stamped_pub.messages[0]
    assert stamped_msg.header.stamp.sec == 7
    assert stamped_msg.header.stamp.nanosec == 11
    assert stamped_msg.header.frame_id == 'table_stereo_left_optical_frame'
    assert list(stamped_msg.detections) == []


def test_yolo_image_callback_publishes_empty_bbox_when_detector_fails(monkeypatch):
    frame = SimpleNamespace(shape=(10, 20, 3))
    node, logger = _minimal_yolo_callback_node(
        monkeypatch,
        bridge=_FrameBridge(frame),
        detector=_FailingDetector('model failed'),
    )

    node._image_callback(_image_msg())

    assert any('Ошибка YOLO inference' in error for error in logger.errors)
    assert len(node._bbox_pub.messages) == 1
    assert list(node._bbox_pub.messages[0].data) == []
    assert len(node._bbox_stamped_pub.messages) == 1
    assert list(node._bbox_stamped_pub.messages[0].detections) == []


def test_yolo_image_callback_keeps_detecting_when_runtime_controls_fail(monkeypatch):
    frame = SimpleNamespace(shape=(10, 20, 3))
    node, logger = _minimal_yolo_callback_node(
        monkeypatch,
        bridge=_FrameBridge(frame),
        detector=SimpleNamespace(detect=lambda *_args, **_kwargs: []),
    )
    node._sync_runtime_controls = _raise_runtime_controls_error

    node._image_callback(_image_msg())

    assert any('Ошибка runtime controls YOLO' in error for error in logger.errors)
    assert len(node._bbox_pub.messages) == 1
    assert list(node._bbox_pub.messages[0].data) == []
