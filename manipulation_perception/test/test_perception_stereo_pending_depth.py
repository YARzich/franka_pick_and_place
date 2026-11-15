"""
Test bounded depth-frame waiting for exact stamped detector responses.

Responsibility: Verify ROI correlation, timeout release, duplicate refresh,
backpressure, and disabled/already-matched bypass behavior.
"""

from types import MethodType, SimpleNamespace

import numpy as np
from std_msgs.msg import Header

from manipulation_perception.common.header_sync import header_key
from manipulation_perception.common.pending_depth_frames import PendingDepthFrames
from manipulation_perception.common.stereo_depth_coordinator import StereoDepthCoordinator


class _FakeClock:
    def __init__(self, nanoseconds: int = 50_000_000_000) -> None:
        self.nanoseconds = nanoseconds

    def now(self) -> SimpleNamespace:
        return SimpleNamespace(nanoseconds=self.nanoseconds)


class _FakeLogger:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def warn(self, message: str, **_kwargs) -> None:
        self.warnings.append(message)


class _FakeRoiFilter:
    enabled = True

    def __init__(self) -> None:
        self.ready = False

    def has_fresh_response_for_header(self, source_header, now_sec: float) -> bool:
        return self.ready


def _header(sec=2, nanosec=3, frame_id='table_stereo_left_optical_frame'):
    header = Header()
    header.stamp.sec = sec
    header.stamp.nanosec = nanosec
    header.frame_id = frame_id
    return header


def test_stereo_depth_frame_waits_for_matching_bbox_response_before_publish():
    coordinator = object.__new__(StereoDepthCoordinator)
    roi_filter = _FakeRoiFilter()
    logger = _FakeLogger()
    header = _header()
    depth = np.ones((2, 3), dtype=np.float32)
    intrinsics = (200.0, 210.0, 1.0, 0.5)
    published = []

    coordinator._roi_filter = roi_filter
    coordinator._pending_frames = PendingDepthFrames(
        response_lookup=roi_filter,
        timeout_sec=1.0,
    )
    coordinator._shutting_down = False
    coordinator._clock = _FakeClock()
    coordinator._logger = logger
    coordinator._on_drop_or_skip = lambda _count: None
    coordinator._record_processed = lambda _started_at: None
    coordinator._on_error = lambda: None
    coordinator._publish_depth = MethodType(
        lambda _self, depth_msg, source_header, sample_intrinsics: published.append(
            (depth_msg, source_header, sample_intrinsics)
        ),
        coordinator,
    )

    assert coordinator._queue_until_matching_detection(depth, header, 0.0, intrinsics) is True
    assert published == []
    assert coordinator._pending_frames.keys() == (header_key(header),)

    roi_filter.ready = True
    coordinator.process_pending_depth_frames()

    assert published == [(depth, header, intrinsics)]
    assert len(coordinator._pending_frames) == 0


def test_stereo_pending_depth_timeout_publishes_without_new_bbox_callback():
    coordinator = object.__new__(StereoDepthCoordinator)
    roi_filter = _FakeRoiFilter()
    logger = _FakeLogger()
    header = _header()
    depth = np.ones((2, 3), dtype=np.float32)
    published = []

    coordinator._roi_filter = roi_filter
    coordinator._pending_frames = PendingDepthFrames(
        response_lookup=roi_filter,
        timeout_sec=0.1,
    )
    coordinator._pending_frames.add_if_waiting(
        depth,
        header,
        now_ros_sec=50.0,
        received_at_monotonic=-1.0,
    )
    coordinator._shutting_down = False
    coordinator._clock = _FakeClock()
    coordinator._logger = logger
    coordinator._record_processed = lambda _started_at: None
    coordinator._on_error = lambda: None
    coordinator._publish_depth = MethodType(
        lambda _self, depth_msg, source_header, intrinsics: published.append(
            (depth_msg, source_header, intrinsics)
        ),
        coordinator,
    )

    coordinator.process_pending_depth_frames()

    assert published == [(depth, header, (None, None, None, None))]
    assert len(coordinator._pending_frames) == 0


def test_pending_depth_frames_release_matching_and_expired_frames_in_order():
    roi_filter = _FakeRoiFilter()
    pending = PendingDepthFrames(response_lookup=roi_filter, timeout_sec=0.5)
    matching_header = _header(sec=2)
    waiting_header = _header(sec=3)
    expired_header = _header(sec=4)
    matching_depth = np.full((1, 1), 2.0, dtype=np.float32)
    waiting_depth = np.full((1, 1), 3.0, dtype=np.float32)
    expired_depth = np.full((1, 1), 4.0, dtype=np.float32)

    pending.add_if_waiting(
        matching_depth,
        matching_header,
        now_ros_sec=50.0,
        received_at_monotonic=9.9,
    )
    pending.add_if_waiting(
        waiting_depth,
        waiting_header,
        now_ros_sec=50.0,
        received_at_monotonic=9.9,
    )
    pending.add_if_waiting(
        expired_depth,
        expired_header,
        now_ros_sec=50.0,
        received_at_monotonic=9.0,
    )
    roi_filter.has_fresh_response_for_header = (
        lambda source_header, _now_sec: source_header is matching_header
    )

    ready = pending.release_ready(
        now_ros_sec=50.0,
        now_monotonic=10.0,
    )

    assert [frame.source_header for frame in ready] == [matching_header, expired_header]
    assert ready[0].depth is matching_depth
    assert ready[1].depth is expired_depth
    assert pending.keys() == (header_key(waiting_header),)


def test_pending_depth_frames_apply_backpressure_and_reject_invalid_headers():
    roi_filter = _FakeRoiFilter()
    pending = PendingDepthFrames(
        response_lookup=roi_filter,
        timeout_sec=1.0,
        max_size=2,
    )
    invalid_header = _header(sec=0, nanosec=0)

    invalid_result = pending.add_if_waiting(
        np.zeros((1, 1), dtype=np.float32),
        invalid_header,
        now_ros_sec=1.0,
        received_at_monotonic=1.0,
    )
    assert invalid_result.queued is False
    assert invalid_result.dropped == ()

    first_header = _header(sec=1)
    second_header = _header(sec=2)
    third_header = _header(sec=3)
    first_depth = np.full((1, 1), 1.0, dtype=np.float32)
    pending.add_if_waiting(
        first_depth,
        first_header,
        now_ros_sec=1.0,
        received_at_monotonic=1.0,
    )
    pending.add_if_waiting(
        np.full((1, 1), 2.0),
        second_header,
        now_ros_sec=1.1,
        received_at_monotonic=1.1,
    )
    result = pending.add_if_waiting(
        np.full((1, 1), 3.0),
        third_header,
        now_ros_sec=1.2,
        received_at_monotonic=1.2,
    )

    assert result.queued is True
    assert len(result.dropped) == 1
    assert result.dropped[0].source_header is first_header
    assert result.dropped[0].depth is first_depth
    assert pending.keys() == (header_key(second_header), header_key(third_header))


def test_pending_depth_frames_refresh_duplicate_before_applying_backpressure():
    roi_filter = _FakeRoiFilter()
    pending = PendingDepthFrames(
        response_lookup=roi_filter,
        timeout_sec=1.0,
        max_size=2,
    )
    first_header = _header(sec=1)
    second_header = _header(sec=2)
    third_header = _header(sec=3)
    refreshed_depth = np.full((1, 1), 10.0, dtype=np.float32)

    pending.add_if_waiting(
        np.full((1, 1), 1.0),
        first_header,
        now_ros_sec=1.0,
        received_at_monotonic=1.0,
    )
    pending.add_if_waiting(
        np.full((1, 1), 2.0),
        second_header,
        now_ros_sec=1.1,
        received_at_monotonic=1.1,
    )
    pending.add_if_waiting(
        refreshed_depth,
        first_header,
        now_ros_sec=1.2,
        received_at_monotonic=1.2,
    )
    result = pending.add_if_waiting(
        np.full((1, 1), 3.0),
        third_header,
        now_ros_sec=1.3,
        received_at_monotonic=1.3,
    )

    assert [frame.source_header for frame in result.dropped] == [second_header]
    assert pending.keys() == (header_key(first_header), header_key(third_header))
    ready = pending.release_ready(now_ros_sec=1.3, now_monotonic=2.1)
    assert len(ready) == 0


def test_pending_depth_frames_bypass_disabled_or_already_matched_frames():
    roi_filter = _FakeRoiFilter()
    pending = PendingDepthFrames(response_lookup=roi_filter, timeout_sec=1.0)
    header = _header()
    depth = np.ones((1, 1), dtype=np.float32)

    roi_filter.enabled = False
    disabled = pending.add_if_waiting(
        depth,
        header,
        now_ros_sec=1.0,
        received_at_monotonic=1.0,
    )
    roi_filter.enabled = True
    roi_filter.ready = True
    matched = pending.add_if_waiting(
        depth,
        header,
        now_ros_sec=1.0,
        received_at_monotonic=1.0,
    )

    assert disabled.queued is False
    assert matched.queued is False
    assert len(pending) == 0
