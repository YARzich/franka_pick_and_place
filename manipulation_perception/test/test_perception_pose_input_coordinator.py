"""
Test pending-frame coordination for stamped pose inference inputs.

Responsibility: Verify readiness, bounded backpressure, timeout behavior,
empty detections, and exact bbox-to-image matching in PoseInputCoordinator.
"""

import time
from types import SimpleNamespace

from perception_pose_lifecycle_test_support import (
    FakeInputBuffer,
    KeyedFakeInputBuffer,
    make_coordinator,
    make_header,
    make_image_message,
)

from manipulation_perception.common.header_sync import header_key


def test_pending_image_waits_until_matching_inputs_are_ready():
    input_buffer = FakeInputBuffer()
    image = make_image_message()
    coordinator, runner, _logger = make_coordinator(
        input_buffer,
        (image, time.monotonic()),
    )

    coordinator._try_start_ready_inference()
    assert runner.started == []
    input_buffer.ready = True
    coordinator._try_start_ready_inference()

    assert runner.started[0][0].bbox == (1, 1, 2, 2)
    assert len(coordinator._pending_frames) == 0


def test_empty_detection_response_publishes_invalidation():
    input_buffer = FakeInputBuffer()
    input_buffer.empty_bbox = True
    image = make_image_message()
    coordinator, runner, logger = make_coordinator(
        input_buffer,
        (image, time.monotonic()),
    )

    coordinator._try_start_ready_inference()

    assert runner.invalidated == [(image.header, 0)]
    assert len(coordinator._pending_frames) == 0
    assert logger.warnings == []


def test_pending_timeout_drops_without_false_invalidation():
    input_buffer = FakeInputBuffer()
    image = make_image_message(make_header(sec=12, frame_id='camera'))
    coordinator, runner, logger = make_coordinator(
        input_buffer,
        (image, time.monotonic() - 2.0),
    )

    coordinator._try_start_ready_inference()

    assert runner.invalidated == []
    assert len(coordinator._pending_frames) == 0
    assert any('timeout' in message for message in logger.warnings)


def test_ready_context_suppresses_individual_timeout_warning():
    expired = make_image_message(make_header(sec=12, frame_id='camera'))
    ready = make_image_message(make_header(sec=13, frame_id='camera'))
    input_buffer = FakeInputBuffer()
    input_buffer.context_for_image = lambda message, **_kwargs: (
        SimpleNamespace(camera_info='camera', bbox=(1, 1, 2, 2))
        if message is ready
        else None
    )
    coordinator, runner, logger = make_coordinator(
        input_buffer,
        (expired, time.monotonic() - 2.0),
        (ready, time.monotonic()),
    )

    coordinator._try_start_ready_inference()

    assert runner.started[0][0].bbox == (1, 1, 2, 2)
    assert logger.warnings == []


def test_pending_image_eviction_is_bounded_without_invalidation():
    input_buffer = KeyedFakeInputBuffer()
    coordinator, runner, _logger = make_coordinator(input_buffer)
    coordinator._try_start_ready_inference = lambda: None

    for sec in range(1, 22):
        coordinator.on_image(make_image_message(make_header(sec=sec, frame_id='camera')))

    assert len(coordinator._pending_frames) == 20
    assert [key.stamp_ns for key in coordinator._pending_frames.keys()] == [
        sec * 1_000_000_000 for sec in range(2, 22)
    ]
    assert runner.invalidated == []


def test_busy_worker_does_not_delay_newer_empty_invalidation():
    ready = make_image_message(make_header(sec=10, frame_id='camera'))
    empty = make_image_message(make_header(sec=11, frame_id='camera'))
    input_buffer = FakeInputBuffer()
    input_buffer.has_empty_bbox_response_for_image = lambda message: message is empty
    input_buffer.context_for_image = lambda message, **_kwargs: (
        SimpleNamespace(camera_info='camera', bbox=(1, 1, 2, 2))
        if message is ready
        else None
    )
    coordinator, runner, _logger = make_coordinator(
        input_buffer,
        (ready, time.monotonic()),
        (empty, time.monotonic()),
    )
    assert runner.gate.try_start(time.monotonic())

    coordinator._try_start_ready_inference()

    assert runner.invalidated == [(empty.header, 0)]
    assert len(coordinator._pending_frames) == 0


def test_busy_worker_backpressure_drops_without_false_invalidation():
    ready = make_image_message(make_header(sec=10, frame_id='camera'))
    expired = make_image_message(make_header(sec=11, frame_id='camera'))
    input_buffer = FakeInputBuffer()
    input_buffer.context_for_image = lambda message, **_kwargs: (
        SimpleNamespace(camera_info='camera', bbox=(1, 1, 2, 2))
        if message is ready
        else None
    )
    coordinator, runner, logger = make_coordinator(
        input_buffer,
        (ready, time.monotonic()),
        (expired, time.monotonic() - 2.0),
    )
    assert runner.gate.try_start(time.monotonic())

    coordinator._try_start_ready_inference()

    assert runner.invalidated == []
    assert len(coordinator._pending_frames) == 0
    assert logger.warnings == []


def test_inference_starts_when_matching_bbox_arrives_after_rgb():
    input_buffer = KeyedFakeInputBuffer()
    coordinator, runner, _logger = make_coordinator(input_buffer)
    image = make_image_message(make_header(sec=4, frame_id='camera'))
    bbox = SimpleNamespace(header=make_header(sec=4, frame_id='camera'))

    coordinator.on_image(image)
    assert runner.started == []
    coordinator.on_bbox(bbox)

    assert runner.started[0][0].bbox == (1, 1, 2, 2)
    assert len(coordinator._pending_frames) == 0


def test_inference_does_not_start_for_mismatched_bbox_frame():
    input_buffer = KeyedFakeInputBuffer()
    coordinator, runner, _logger = make_coordinator(input_buffer)
    image = make_image_message(make_header(sec=4, frame_id='camera'))
    bbox = SimpleNamespace(header=make_header(sec=4, frame_id='other_camera'))

    coordinator.on_image(image)
    coordinator.on_bbox(bbox)

    assert runner.started == []
    assert list(coordinator._pending_frames.keys()) == [header_key(image.header)]
