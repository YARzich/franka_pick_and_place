"""
Coordinate stamped pose inputs before backend inference starts.

Responsibility:
    Associate RGB, CameraInfo, bbox, and optional depth by exact sensor identity,
    enforce freshness, and emit one immutable inference snapshot.

Does not own:
    ROS subscriptions, image decoding, or backend execution.
"""

from __future__ import annotations

import time
from typing import Any, Callable

from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray

from manipulation_perception.pose.pose_input_buffer import PoseInputBuffer
from manipulation_perception.pose.pose_node_parameters import PoseNodeParameters
from manipulation_perception.pose.pose_pending_frames import PendingPoseFrames


class PoseInputCoordinator:
    """
    Match sensor messages by exact `(stamp, frame_id)` and start inference.

    Methods are callback-serialized by the owning ROS node. Header association
    and cached-input age follow ROS time and are cleared on clock jumps. Pending
    queue age and inference intervals use steady `time.monotonic()` so a paused
    simulation clock cannot retain pending work forever.
    """

    def __init__(
        self,
        *,
        parameters: PoseNodeParameters,
        bridge: Any,
        clock: Any,
        runner: Any,
        logger: Any,
        on_input: Callable[[], None] | None = None,
        on_drop_or_skip: Callable[[int], None] | None = None,
        on_error: Callable[[], None] | None = None,
    ) -> None:
        common = parameters.common
        self._runner = runner
        self._logger = logger
        self._on_input = on_input or (lambda: None)
        self._on_drop_or_skip = on_drop_or_skip or (lambda _count=1: None)
        self._on_error = on_error or (lambda: None)
        self._input_buffer = PoseInputBuffer(
            bridge=bridge,
            clock=clock,
            bbox_timeout_sec=common.bbox_timeout_sec,
            use_mask=common.uses_gigapose_coarse,
            depth_enabled=not common.is_depth_disabled,
            log_warn=lambda message: logger.warn(
                message, throttle_duration_sec=3.0
            ),
            log_error=logger.error,
        )
        self._pending_frames = PendingPoseFrames(
            input_lookup=self._input_buffer,
            timeout_sec=common.bbox_timeout_sec,
        )

    def on_camera_info(self, message: CameraInfo) -> None:
        """Store calibration and retry the oldest ready RGB frame."""
        if self._runner.shutdown_requested():
            return
        self._input_buffer.update_camera_info(message)
        self._try_start_ready_inference()

    def on_depth(self, message: Image) -> None:
        """Store depth and retry pending RGB frames with the same header key."""
        if self._runner.shutdown_requested():
            return
        self._input_buffer.update_depth(message)
        self._try_start_ready_inference()

    def on_mask(self, message: Image) -> None:
        """Store a segmentation mask and retry the matching pending RGB frame."""
        if self._runner.shutdown_requested():
            return
        self._input_buffer.update_mask(message)
        self._try_start_ready_inference()

    def on_bbox(self, message: Detection2DArray) -> None:
        """Store stamped detections and retry the matching pending RGB frame."""
        if self._runner.shutdown_requested():
            return
        self._input_buffer.update_bbox_detections(message)
        self._try_start_ready_inference()

    def on_image(self, message: Image) -> None:
        """Queue an accepted RGB frame under its exact stamped frame key."""
        if self._runner.shutdown_requested():
            return
        self._on_input()
        if not self._runner.accepts_frame():
            self._on_drop_or_skip(1)
            return
        image_key = self._input_buffer.message_key(message)
        if image_key is None:
            self._on_drop_or_skip(1)
            self._logger.warn(
                "MegaPose пропускает RGB без непустых stamp/frame_id.",
                throttle_duration_sec=3.0,
            )
            return
        queue_size = len(self._pending_frames)
        self._pending_frames.add(
            image_key,
            message,
            time.monotonic(),
            self._runner.current_generation(),
        )
        if queue_size == len(self._pending_frames) and queue_size > 0:
            self._on_drop_or_skip(1)
        self._try_start_ready_inference()

    def on_clock_jump(self) -> None:
        """Clear all stamped state and invalidate the previous output generation."""
        self._runner.reset_for_clock_jump()
        self._pending_frames.clear()
        self._input_buffer.clear()
        self._logger.warn(
            "Обнаружен сброс ROS clock: очищено синхронизированное состояние MegaPose."
        )

    def _try_start_ready_inference(self) -> None:
        if self._runner.shutdown_requested():
            return
        now_monotonic = time.monotonic()
        pending_result = self._pending_frames.evaluate(
            now_monotonic=now_monotonic,
            require_depth=self._runner.requires_depth_before_inference(),
            worker_running=self._runner.is_running(),
        )
        for frame in pending_result.no_detection_frames:
            self._runner.publish_invalidation(frame.image.header, frame.generation)
        dropped_count = (
            len(pending_result.expired_frames)
            + len(pending_result.no_detection_frames)
            + pending_result.backpressured_count
        )
        if dropped_count:
            self._on_drop_or_skip(dropped_count)
        if pending_result.should_warn_sync_timeout:
            self._logger.warn(
                "MegaPose пропустил RGB: не дождался matching CameraInfo/bbox/depth до timeout.",
                throttle_duration_sec=3.0,
            )

        ready = pending_result.ready
        if (
            self._runner.is_running()
            or ready is None
            or not self._runner.try_start(now_monotonic)
        ):
            return
        self._pending_frames.remove(ready.key)
        snapshot = self._input_buffer.decode_snapshot(ready.image, ready.context)
        if snapshot is None:
            self._on_error()
            self._runner.publish_invalidation(ready.image.header, ready.generation)
            # A failed decode consumed the reserved gate but started no worker.
            self._runner.mark_start_failed()
            return
        self._runner.start(snapshot, ready.generation)


__all__ = ["PoseInputCoordinator"]
