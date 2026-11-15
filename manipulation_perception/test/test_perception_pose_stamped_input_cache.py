"""
Test bounded exact-key storage for pose inference inputs.

Responsibility: Verify cache matching and per-input eviction independently
from cv_bridge decoding and higher-level warning behavior.
"""

from perception_pose_input_test_support import (
    FakeClock,
    make_bbox_message,
    make_camera_info,
    make_header,
    make_image_message,
)

from manipulation_perception.pose.pose_stamped_input_cache import (
    PoseContextStatus,
    PoseStampedInputCache,
)


def test_stamped_cache_matches_inputs_without_cv_bridge_dependency():
    clock = FakeClock()
    cache = PoseStampedInputCache(
        clock=clock,
        bbox_timeout_sec=0.5,
        use_mask=False,
        depth_enabled=True,
    )
    header = make_header(sec=4, frame_id="camera")
    image = make_image_message(header=header)
    depth = make_image_message(header=header)

    cache.update_camera_info(make_camera_info(header))
    cache.update_bbox_detections(make_bbox_message(header))
    cache.update_depth(depth)
    lookup = cache.lookup_context(image, require_depth=True)

    assert lookup.status is PoseContextStatus.READY
    assert lookup.context.bbox == (1, 1, 4, 3)
    assert cache.active_depth(cache.message_key(image)).message is depth


def test_stamped_cache_bounds_state_per_input_kind():
    clock = FakeClock()
    cache = PoseStampedInputCache(
        clock=clock,
        bbox_timeout_sec=0.5,
        use_mask=False,
        depth_enabled=False,
        max_size=1,
    )
    old_header = make_header(sec=1, frame_id="camera")
    new_header = make_header(sec=2, frame_id="camera")
    old_image = make_image_message(header=old_header)
    new_image = make_image_message(header=new_header)

    cache.update_camera_info(make_camera_info(old_header))
    cache.update_bbox_detections(make_bbox_message(old_header))
    cache.update_camera_info(make_camera_info(new_header))
    cache.update_bbox_detections(make_bbox_message(new_header))

    old_lookup = cache.lookup_context(old_image, require_depth=False)
    new_lookup = cache.lookup_context(new_image, require_depth=False)

    assert old_lookup.status is PoseContextStatus.MISSING_CAMERA_INFO
    assert new_lookup.status is PoseContextStatus.READY
