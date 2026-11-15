"""
Compatibility facade for synchronized pose inputs and snapshot decoding.

Responsibility:
    Preserve the historical buffer API while decoding synchronized snapshots
    from the canonical stamped cache into backend-ready arrays.

Does not own:
    ROS subscriptions, backend inference, or cache clock-jump policy.

Stamped message ownership and freshness live in `PoseStampedInputCache`.
This facade owns CvBridge conversion, decode fallback and user-facing warnings;
it creates no subscriptions and is callback-serialized by its ROS node owner.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import numpy as np

from manipulation_perception.common.header_sync import HeaderKey, headers_match
from manipulation_perception.pose.pose_input_state import active_binary_mask
from manipulation_perception.pose.pose_stamped_input_cache import (
    ActiveStampedMessage,
    DEFAULT_MAX_CACHE_SIZE,
    PoseContextStatus,
    PoseFrameContext,
    PoseStampedInputCache,
)


LogCallback = Callable[[str], None]
# Historical module constant kept for downstream imports; state owns enforcement.
MAX_CACHE_SIZE = DEFAULT_MAX_CACHE_SIZE

_MISSING_CONTEXT_MESSAGES = {
    PoseContextStatus.INVALID_IMAGE_HEADER: (
        "MegaPose ожидает RGB с непустыми stamp/frame_id."
    ),
    PoseContextStatus.MISSING_CAMERA_INFO: (
        "MegaPose ожидает CameraInfo с тем же stamp/frame_id, что и RGB."
    ),
    PoseContextStatus.MISSING_BBOX: (
        "MegaPose ожидает свежий stamped bbox от детектора для текущего RGB."
    ),
    PoseContextStatus.MISSING_DEPTH: (
        "MegaPose ожидает depth с тем же stamp/frame_id, что и RGB."
    ),
}


@dataclass(frozen=True)
class PoseFrameSnapshot:
    """Hold decoded inputs until the runner copies them across its thread boundary."""

    rgb: np.ndarray
    mask: np.ndarray | None
    depth: np.ndarray | None
    camera_info: Any
    image_header: Any
    bbox: tuple[int, int, int, int]


class PoseInputBuffer:
    """Preserve the node-facing API while delegating stamped state ownership."""

    def __init__(
        self,
        *,
        bridge: Any,
        clock: Any,
        bbox_timeout_sec: float,
        use_mask: bool,
        depth_enabled: bool,
        log_warn: LogCallback | None = None,
        log_error: LogCallback | None = None,
    ) -> None:
        self._bridge = bridge
        self._bbox_timeout_sec = bbox_timeout_sec
        self._log_warn = log_warn
        self._log_error = log_error
        self._cache = PoseStampedInputCache(
            clock=clock,
            bbox_timeout_sec=bbox_timeout_sec,
            use_mask=use_mask,
            depth_enabled=depth_enabled,
        )

    def clear(self) -> None:
        self._cache.clear()

    def update_camera_info(self, message: Any) -> None:
        self._cache.update_camera_info(message)

    def update_depth(self, message: Any) -> None:
        self._cache.update_depth(message)

    def update_mask(self, message: Any) -> None:
        self._cache.update_mask(message)

    def update_bbox_detections(self, message: Any) -> None:
        self._cache.update_bbox_detections(message)

    def context_for_image(
        self,
        image_msg: Any,
        *,
        require_depth: bool = False,
        report_missing: bool = True,
    ) -> PoseFrameContext | None:
        """Return exact stamped context and optionally report why it is incomplete."""
        lookup = self._cache.lookup_context(
            image_msg,
            require_depth=require_depth,
        )
        if lookup.context is None:
            self._warn_if(report_missing, _MISSING_CONTEXT_MESSAGES[lookup.status])
        return lookup.context

    def decode_snapshot(
        self,
        image_msg: Any,
        context: PoseFrameContext,
    ) -> PoseFrameSnapshot | None:
        """Decode one ready frame while treating depth and mask as optional inputs."""
        image_key = self.message_key(image_msg)
        if image_key is None:
            self._warn("MegaPose ожидает RGB с непустыми stamp/frame_id.")
            return None

        try:
            rgb = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding="rgb8")
        except Exception as exc:  # pragma: no cover - runtime dependency guard
            self._error(f"Не удалось декодировать RGB для MegaPose: {exc}")
            return None

        return PoseFrameSnapshot(
            rgb=rgb,
            mask=self._decode_mask(image_msg, image_key),
            depth=self._decode_depth(image_key),
            camera_info=context.camera_info,
            image_header=image_msg.header,
            bbox=context.bbox,
        )

    def has_depth_for_image(self, image_msg: Any) -> bool:
        key = self.message_key(image_msg)
        return key is not None and self._cache.active_depth(key) is not None

    def has_empty_bbox_response_for_image(self, image_msg: Any) -> bool:
        key = self.message_key(image_msg)
        return key is not None and self._cache.has_empty_bbox_response(key)

    @staticmethod
    def message_key(message: Any) -> HeaderKey | None:
        return PoseStampedInputCache.message_key(message)

    @staticmethod
    def headers_match(first_header: Any, second_header: Any) -> bool:
        return headers_match(first_header, second_header)

    def _decode_mask(
        self,
        image_msg: Any,
        image_key: HeaderKey,
    ) -> np.ndarray | None:
        active_mask = self._cache.active_mask(image_key)
        if active_mask is None:
            return None
        try:
            mask = self._bridge.imgmsg_to_cv2(
                active_mask.message,
                desired_encoding="passthrough",
            )
        except Exception as exc:  # pragma: no cover - runtime dependency guard
            self._warn(
                "Не удалось декодировать mask для GigaPose, "
                f"использую bbox-only: {exc}"
            )
            return None
        return active_binary_mask(
            mask,
            enabled=True,
            age_sec=active_mask.age_sec,
            timeout_sec=self._bbox_timeout_sec,
            frame_height=image_msg.height,
            frame_width=image_msg.width,
        )

    def _decode_depth(self, image_key: HeaderKey) -> np.ndarray | None:
        active_depth = self._cache.active_depth(image_key)
        if active_depth is None:
            return None
        return self._decode_optional_image(
            active_depth,
            "Не удалось декодировать depth для MegaPose, запускаю RGB-only",
        )

    def _decode_optional_image(
        self,
        active_message: ActiveStampedMessage,
        error_prefix: str,
    ) -> np.ndarray | None:
        try:
            return self._bridge.imgmsg_to_cv2(
                active_message.message,
                desired_encoding="passthrough",
            )
        except Exception as exc:  # pragma: no cover - runtime dependency guard
            self._warn(f"{error_prefix}: {exc}")
            return None

    def _warn_if(self, enabled: bool, message: str) -> None:
        if enabled:
            self._warn(message)

    def _warn(self, message: str) -> None:
        if self._log_warn is not None:
            self._log_warn(message)

    def _error(self, message: str) -> None:
        if self._log_error is not None:
            self._log_error(message)


__all__ = [
    "MAX_CACHE_SIZE",
    "PoseFrameContext",
    "PoseFrameSnapshot",
    "PoseInputBuffer",
]
