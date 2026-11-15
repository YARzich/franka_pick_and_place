"""
Provide stamped message builders and fakes for pose input-buffer tests.

Responsibility: Keep pose cache, context, and snapshot tests independent
from a ROS clock and cv_bridge image transport.
"""

from types import SimpleNamespace

from std_msgs.msg import Header
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from manipulation_perception.pose.pose_input_buffer import PoseInputBuffer


class FakeDuration:
    def __init__(self, nanoseconds: int) -> None:
        self.nanoseconds = nanoseconds


class FakeTime:
    def __init__(self, nanoseconds: int) -> None:
        self.nanoseconds = nanoseconds

    def __sub__(self, other):  # noqa: D105
        return FakeDuration(self.nanoseconds - other.nanoseconds)


class FakeClock:
    def __init__(self) -> None:
        self.nanoseconds = 0

    def now(self) -> FakeTime:
        return FakeTime(self.nanoseconds)

    def advance(self, seconds: float) -> None:
        self.nanoseconds += int(seconds * 1.0e9)


class FakeBridge:
    def __init__(self) -> None:
        self.images = {}
        self.fail_for = set()

    def add(self, msg, image):
        self.images[id(msg)] = image

    def fail(self, msg) -> None:
        self.fail_for.add(id(msg))

    def imgmsg_to_cv2(self, msg, desired_encoding='passthrough'):
        del desired_encoding
        if id(msg) in self.fail_for:
            raise RuntimeError('decode failed')
        return self.images[id(msg)]


def make_header(sec=1, nanosec=0, frame_id='table_stereo_left_optical_frame'):
    header = Header()
    header.stamp.sec = sec
    header.stamp.nanosec = nanosec
    header.frame_id = frame_id
    return header


def make_image_message(height=4, width=6, header=None):
    header = header if header is not None else make_header()
    return SimpleNamespace(height=height, width=width, header=header)


def make_camera_info(header=None):
    return SimpleNamespace(
        header=header if header is not None else make_header(),
        k=[1.0] * 9,
    )


def make_bbox_message(header=None, xyxy=(1, 1, 4, 3), frame_shape=(4, 6, 3)):
    del frame_shape
    x1, y1, x2, y2 = xyxy
    message = Detection2DArray()
    message.header = header if header is not None else make_header()
    detection = Detection2D()
    detection.header = message.header
    detection.bbox.center.position.x = 0.5 * float(x1 + x2)
    detection.bbox.center.position.y = 0.5 * float(y1 + y2)
    detection.bbox.size_x = float(x2 - x1)
    detection.bbox.size_y = float(y2 - y1)
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = '1'
    hypothesis.hypothesis.score = 0.9
    detection.results.append(hypothesis)
    message.detections.append(detection)
    return message


def make_empty_bbox_message(header=None):
    message = Detection2DArray()
    message.header = header if header is not None else make_header()
    return message


def make_buffer(
    clock,
    bridge,
    *,
    use_mask=True,
    depth_enabled=True,
    warnings=None,
    errors=None,
):
    return PoseInputBuffer(
        bridge=bridge,
        clock=clock,
        bbox_timeout_sec=0.5,
        use_mask=use_mask,
        depth_enabled=depth_enabled,
        log_warn=(warnings if warnings is not None else []).append,
        log_error=(errors if errors is not None else []).append,
    )
