"""
Test exact stamped context assembly for pose inference.

Responsibility: Verify that RGB receives only matching camera calibration,
detections, depth, and masks, with explicit missing-input behavior.
"""

from types import SimpleNamespace

import numpy as np

from perception_pose_input_test_support import (
    FakeBridge,
    FakeClock,
    make_bbox_message,
    make_buffer,
    make_camera_info,
    make_empty_bbox_message,
    make_header,
    make_image_message,
)


def test_context_uses_stamped_bbox_in_current_image_coordinates():
    clock = FakeClock()
    bridge = FakeBridge()
    warnings = []
    buffer = make_buffer(clock, bridge, warnings=warnings)

    header = make_header()
    image_msg = make_image_message(height=200, width=300, header=header)
    buffer.update_camera_info(make_camera_info(header))
    buffer.update_bbox_detections(
        make_bbox_message(header, xyxy=(10, 20, 30, 40), frame_shape=(100, 100, 3))
    )

    context = buffer.context_for_image(image_msg)

    assert context.camera_info.header == header
    assert context.bbox == (10, 20, 30, 40)
    assert warnings == []


def test_context_rejects_missing_camera_info_or_stale_bbox():
    clock = FakeClock()
    bridge = FakeBridge()
    warnings = []
    buffer = make_buffer(clock, bridge, warnings=warnings)

    image_msg = make_image_message(height=10, width=10)

    assert buffer.context_for_image(image_msg) is None
    assert warnings[-1] == "MegaPose ожидает CameraInfo с тем же stamp/frame_id, что и RGB."

    buffer.update_camera_info(make_camera_info(image_msg.header))
    buffer.update_bbox_detections(
        make_bbox_message(image_msg.header, xyxy=(1, 1, 4, 4), frame_shape=(10, 10, 3))
    )
    clock.advance(0.6)

    assert buffer.context_for_image(image_msg) is None
    assert warnings[-1] == "MegaPose ожидает свежий stamped bbox от детектора для текущего RGB."


def test_context_rejects_camera_info_bbox_stamp_or_frame_mismatch():
    clock = FakeClock()
    bridge = FakeBridge()
    warnings = []
    buffer = make_buffer(clock, bridge, warnings=warnings)
    image_msg = make_image_message(header=make_header(sec=10, frame_id="camera"))

    buffer.update_camera_info(make_camera_info(make_header(sec=11, frame_id="camera")))
    buffer.update_bbox_detections(make_bbox_message(image_msg.header))

    assert buffer.context_for_image(image_msg) is None
    assert warnings[-1] == "MegaPose ожидает CameraInfo с тем же stamp/frame_id, что и RGB."

    buffer = make_buffer(clock, bridge, warnings=warnings)
    buffer.update_camera_info(make_camera_info(image_msg.header))
    buffer.update_bbox_detections(make_bbox_message(make_header(sec=10, frame_id="other_camera")))

    assert buffer.context_for_image(image_msg) is None
    assert warnings[-1] == "MegaPose ожидает свежий stamped bbox от детектора для текущего RGB."


def test_context_rejects_empty_bbox_response_without_waiting_for_timeout():
    clock = FakeClock()
    bridge = FakeBridge()
    warnings = []
    buffer = make_buffer(clock, bridge, warnings=warnings)
    image_msg = make_image_message()

    buffer.update_camera_info(make_camera_info(image_msg.header))
    buffer.update_bbox_detections(make_empty_bbox_message(image_msg.header))

    assert buffer.has_empty_bbox_response_for_image(image_msg) is True
    assert buffer.context_for_image(image_msg) is None
    assert warnings[-1] == "MegaPose ожидает свежий stamped bbox от детектора для текущего RGB."


def test_context_can_require_matching_depth_before_inference():
    clock = FakeClock()
    bridge = FakeBridge()
    warnings = []
    buffer = make_buffer(clock, bridge, warnings=warnings)
    image_msg = make_image_message()

    buffer.update_camera_info(make_camera_info(image_msg.header))
    buffer.update_bbox_detections(make_bbox_message(image_msg.header))

    assert buffer.context_for_image(image_msg, require_depth=True) is None
    assert warnings[-1] == "MegaPose ожидает depth с тем же stamp/frame_id, что и RGB."

    depth_msg = make_image_message(header=image_msg.header)
    buffer.update_depth(depth_msg)

    assert buffer.context_for_image(image_msg, require_depth=True) is not None


def test_context_uses_matching_frame_state_even_after_newer_messages():
    clock = FakeClock()
    bridge = FakeBridge()
    buffer = make_buffer(clock, bridge)
    header_a = make_header(sec=1, frame_id="camera")
    header_b = make_header(sec=2, frame_id="camera")
    rgb_a = make_image_message(header=header_a)
    depth_a = make_image_message(header=header_a)
    mask_a = make_image_message(header=header_a)
    depth_b = make_image_message(header=header_b)
    mask_b = make_image_message(header=header_b)
    camera_info_a = SimpleNamespace(header=header_a, k=[1.0] * 9)
    camera_info_b = SimpleNamespace(header=header_b, k=[2.0] * 9)
    depth_array_a = np.full((4, 6), 1.0, dtype=np.float32)
    depth_array_b = np.full((4, 6), 2.0, dtype=np.float32)
    mask_array_a = np.zeros((4, 6), dtype=np.uint8)
    mask_array_b = np.ones((4, 6), dtype=np.uint8) * 255
    bridge.add(rgb_a, np.zeros((4, 6, 3), dtype=np.uint8))
    bridge.add(depth_a, depth_array_a)
    bridge.add(depth_b, depth_array_b)
    bridge.add(mask_a, mask_array_a)
    bridge.add(mask_b, mask_array_b)

    buffer.update_camera_info(camera_info_a)
    buffer.update_bbox_detections(make_bbox_message(header_a, xyxy=(1, 1, 4, 3)))
    buffer.update_depth(depth_a)
    buffer.update_mask(mask_a)
    buffer.update_camera_info(camera_info_b)
    buffer.update_bbox_detections(make_bbox_message(header_b, xyxy=(0, 0, 2, 2)))
    buffer.update_depth(depth_b)
    buffer.update_mask(mask_b)

    context = buffer.context_for_image(rgb_a, require_depth=True)
    snapshot = buffer.decode_snapshot(rgb_a, context)

    assert context.camera_info is camera_info_a
    assert context.bbox == (1, 1, 4, 3)
    assert snapshot.depth is depth_array_a
    np.testing.assert_array_equal(snapshot.mask, np.zeros((4, 6), dtype=np.uint8))


def test_context_rejects_unstamped_or_empty_frame_bbox():
    clock = FakeClock()
    bridge = FakeBridge()
    warnings = []
    buffer = make_buffer(clock, bridge, warnings=warnings)
    image_msg = make_image_message(header=make_header(sec=10, frame_id="camera"))

    buffer.update_camera_info(make_camera_info(image_msg.header))
    buffer.update_bbox_detections(make_bbox_message(make_header(sec=10, frame_id="")))

    assert buffer.context_for_image(image_msg) is None
    assert warnings[-1] == "MegaPose ожидает свежий stamped bbox от детектора для текущего RGB."

    buffer.update_bbox_detections(
        SimpleNamespace(header=SimpleNamespace(frame_id="camera"), detections=[])
    )

    assert buffer.context_for_image(image_msg) is None
    assert warnings[-1] == "MegaPose ожидает свежий stamped bbox от детектора для текущего RGB."


def test_context_rejects_zero_stamp_without_latest_bbox_fallback():
    clock = FakeClock()
    bridge = FakeBridge()
    warnings = []
    buffer = make_buffer(clock, bridge, warnings=warnings)
    valid_header = make_header(sec=10, frame_id="camera")
    zero_header = make_header(sec=0, nanosec=0, frame_id="camera")
    image_msg = make_image_message(header=valid_header)

    buffer.update_camera_info(make_camera_info(valid_header))
    buffer.update_bbox_detections(make_bbox_message(zero_header, xyxy=(1, 1, 4, 4)))

    assert buffer.context_for_image(image_msg) is None
    assert warnings[-1] == "MegaPose ожидает свежий stamped bbox от детектора для текущего RGB."

    zero_image_msg = make_image_message(header=zero_header)
    buffer.update_camera_info(make_camera_info(zero_header))
    buffer.update_bbox_detections(make_bbox_message(zero_header, xyxy=(1, 1, 4, 4)))

    assert buffer.context_for_image(zero_image_msg) is None
    assert warnings[-1] == "MegaPose ожидает RGB с непустыми stamp/frame_id."
