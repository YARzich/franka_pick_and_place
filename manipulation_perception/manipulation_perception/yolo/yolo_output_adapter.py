"""
Adapt YOLO detections to the stamped ROS output contract.

Responsibility:
    Convert backend-neutral detections and masks into ROS messages that retain
    the exact source image header, then publish them through borrowed handles.

Does not own:
    Publisher lifetime, inference, or detection tracking.

The adapter borrows publishers and `CvBridge`; the ROS node owns and destroys
those resources. All bbox and mask outputs retain the source image timestamp.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from sensor_msgs.msg import Image

from manipulation_perception.common.detection_candidate import DetectionCandidate
from manipulation_perception.yolo.yolo_outputs import (
    detection_bbox_array_message,
    detection_bbox_message,
    detection_mask_array,
)


class YoloOutputAdapter:
    """Publish legacy bbox, stamped detection and mask from one result list."""

    def __init__(
        self,
        *,
        bridge: Any,
        bbox_publisher: Any | None,
        stamped_bbox_publisher: Any | None,
        mask_publisher: Any | None,
        bbox_frame_id: str,
        mask_fill_value: int,
        on_output: Callable[[Any, bool], None] | None = None,
    ) -> None:
        self._bridge = bridge
        self._bbox_publisher = bbox_publisher
        self._stamped_bbox_publisher = stamped_bbox_publisher
        self._mask_publisher = mask_publisher
        self._bbox_frame_id = bbox_frame_id
        self._mask_fill_value = mask_fill_value
        self._on_output = on_output or (lambda _header, _successful: None)

    def publish(
        self,
        detections: list[DetectionCandidate],
        frame_shape: tuple[int, int, int],
        source_message: Image,
        *,
        successful: bool = True,
    ) -> None:
        """Publish every enabled output for the source sensor sample."""
        if self._bbox_publisher is not None:
            self._bbox_publisher.publish(
                detection_bbox_message(detections, frame_shape)
            )

        if self._stamped_bbox_publisher is not None:
            bbox_header = source_message.header
            if self._bbox_frame_id:
                bbox_header = type(source_message.header)()
                bbox_header.stamp = source_message.header.stamp
                bbox_header.frame_id = self._bbox_frame_id
            self._stamped_bbox_publisher.publish(
                detection_bbox_array_message(detections, frame_shape, bbox_header)
            )

        if self._mask_publisher is not None:
            mask = detection_mask_array(
                detections,
                frame_shape,
                self._mask_fill_value,
            )
            mask_message = self._bridge.cv2_to_imgmsg(mask, encoding="mono8")
            mask_message.header = source_message.header
            self._mask_publisher.publish(mask_message)
        self._on_output(source_message.header, successful)

    def publish_empty(self, source_message: Image) -> None:
        """Invalidate all enabled outputs after decode or inference failure."""
        frame_shape = (
            max(int(source_message.height), 1),
            max(int(source_message.width), 1),
            3,
        )
        self.publish([], frame_shape, source_message, successful=False)

    def publish_empty_safely(
        self,
        source_message: Image,
        *,
        frame_shape: tuple[int, int, int] | None,
        log_error: Any,
    ) -> None:
        """Best-effort invalidation used while handling another pipeline error."""
        try:
            if frame_shape is None:
                self.publish_empty(source_message)
            else:
                self.publish([], frame_shape, source_message, successful=False)
        except Exception as error:  # noqa: BLE001 - recovery must not mask root failure
            log_error(
                f"Не удалось опубликовать пустой YOLO bbox/mask после ошибки: {error}",
                throttle_duration_sec=3.0,
            )


__all__ = ["YoloOutputAdapter"]
