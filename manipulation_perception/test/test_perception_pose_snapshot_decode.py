"""
Test conversion of synchronized pose inputs into inference snapshots.

Responsibility: Verify RGB/depth/mask decoding, optional-input fallbacks,
error reporting, and synchronized-cache reset behavior.
"""

import numpy as np

from perception_pose_input_test_support import (
    FakeBridge,
    FakeClock,
    make_bbox_message,
    make_buffer,
    make_camera_info,
    make_header,
    make_image_message,
)


def test_decode_snapshot_reads_rgb_depth_and_binary_mask():
    clock = FakeClock()
    bridge = FakeBridge()
    buffer = make_buffer(clock, bridge)
    header = make_header()
    rgb_msg = make_image_message(header=header)
    depth_msg = make_image_message(header=header)
    mask_msg = make_image_message(header=header)
    rgb = np.zeros((4, 6, 3), dtype=np.uint8)
    depth = np.ones((4, 6), dtype=np.float32)
    mask = np.zeros((2, 3), dtype=np.uint8)
    mask[0, 1] = 255
    bridge.add(rgb_msg, rgb)
    bridge.add(depth_msg, depth)
    bridge.add(mask_msg, mask)

    buffer.update_camera_info(make_camera_info(header))
    buffer.update_bbox_detections(make_bbox_message(header))
    buffer.update_depth(depth_msg)
    buffer.update_mask(mask_msg)
    context = buffer.context_for_image(rgb_msg)

    snapshot = buffer.decode_snapshot(rgb_msg, context)

    assert snapshot.rgb is rgb
    assert snapshot.depth is depth
    assert snapshot.mask.shape == (4, 6)
    assert snapshot.mask.dtype == np.uint8
    assert snapshot.camera_info.header == header
    assert snapshot.image_header == header
    assert snapshot.bbox == (1, 1, 4, 3)


def test_decode_snapshot_falls_back_when_optional_inputs_fail_mismatch_or_are_disabled():
    clock = FakeClock()
    bridge = FakeBridge()
    warnings = []
    buffer = make_buffer(clock, bridge, use_mask=True, depth_enabled=True, warnings=warnings)
    header = make_header(sec=1)
    rgb_msg = make_image_message(header=header)
    stale_header = make_header(sec=2)
    depth_msg = make_image_message(header=stale_header)
    mask_msg = make_image_message(header=stale_header)
    bridge.add(rgb_msg, np.zeros((4, 6, 3), dtype=np.uint8))
    bridge.add(mask_msg, np.ones((4, 6), dtype=np.uint8))
    bridge.fail(depth_msg)

    buffer.update_camera_info(make_camera_info(header))
    buffer.update_bbox_detections(make_bbox_message(header))
    buffer.update_depth(depth_msg)
    buffer.update_mask(mask_msg)
    context = buffer.context_for_image(rgb_msg)

    snapshot = buffer.decode_snapshot(rgb_msg, context)

    assert snapshot.depth is None
    assert snapshot.mask is None
    assert warnings == []

    matching_depth_msg = make_image_message(header=header)
    bridge.fail(matching_depth_msg)
    buffer.update_depth(matching_depth_msg)

    snapshot = buffer.decode_snapshot(rgb_msg, context)

    assert snapshot.depth is None
    assert "depth" in warnings[-1]


def test_decode_snapshot_reports_rgb_decode_failure():
    clock = FakeClock()
    bridge = FakeBridge()
    errors = []
    buffer = make_buffer(clock, bridge, errors=errors)
    rgb_msg = make_image_message()
    bridge.fail(rgb_msg)

    buffer.update_camera_info(make_camera_info(rgb_msg.header))
    buffer.update_bbox_detections(make_bbox_message(rgb_msg.header))
    context = buffer.context_for_image(rgb_msg)

    assert buffer.decode_snapshot(rgb_msg, context) is None
    assert "RGB" in errors[-1]


def test_clear_removes_all_synchronized_input_caches():
    clock = FakeClock()
    bridge = FakeBridge()
    buffer = make_buffer(clock, bridge)
    header = make_header(sec=5, frame_id="camera")
    rgb_msg = make_image_message(header=header)
    depth_msg = make_image_message(header=header)
    mask_msg = make_image_message(header=header)
    bridge.add(rgb_msg, np.zeros((4, 6, 3), dtype=np.uint8))
    bridge.add(depth_msg, np.ones((4, 6), dtype=np.float32))
    bridge.add(mask_msg, np.ones((4, 6), dtype=np.uint8) * 255)

    buffer.update_camera_info(make_camera_info(header))
    buffer.update_bbox_detections(make_bbox_message(header))
    buffer.update_depth(depth_msg)
    buffer.update_mask(mask_msg)
    context = buffer.context_for_image(rgb_msg, require_depth=True)
    assert context is not None
    assert buffer.has_depth_for_image(rgb_msg) is True
    assert buffer.decode_snapshot(rgb_msg, context).mask is not None

    buffer.clear()

    assert buffer.context_for_image(rgb_msg, require_depth=True, report_missing=False) is None
    assert buffer.has_depth_for_image(rgb_msg) is False
    assert buffer.has_empty_bbox_response_for_image(rgb_msg) is False
    cleared_snapshot = buffer.decode_snapshot(rgb_msg, context)
    assert cleared_snapshot.depth is None
    assert cleared_snapshot.mask is None
