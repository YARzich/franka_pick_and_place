"""
Stamped TF lookup policy for stereo point-cloud processing.

Responsibility:
    Own the TF2 buffer/listener and resolve `T_target_source` at each sensor
    stamp under the configured timeout and latest-transform fallback policy.

Does not own:
    Point transformation, sensor synchronization, or cloud publication.
"""

from __future__ import annotations

from typing import Any, Callable

import numpy as np
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from manipulation_perception.common.transform_geometry import quat_to_rot_matrix


class StereoTransformProvider:
    """
    Own TF2 handles and return `T_target_source` matrices.

    Message stamps use ROS time. The optional latest-TF fallback is intended
    only for a static table camera and is reported once. The owning ROS node
    calls this object from its serialized callback group and releases it during
    shutdown before destroying the node.
    """

    def __init__(
        self,
        *,
        node: Any,
        allow_latest_fallback: bool,
        logger: Any,
        frame_index: Callable[[], int],
    ) -> None:
        self._allow_latest_fallback = allow_latest_fallback
        self._logger = logger
        self._frame_index = frame_index
        self._warned_about_tf = False
        self._warned_about_latest_fallback = False
        self._buffer = Buffer(cache_time=Duration(seconds=5.0), node=node)
        self._listener = TransformListener(self._buffer, node, spin_thread=False)

    def lookup(
        self,
        target_frame: str,
        source_frame: str,
        stamp: Any,
    ) -> np.ndarray | None:
        """Return `T_target_source` at `stamp`, or `None` when unavailable."""
        if not target_frame or not source_frame:
            return None
        if target_frame == source_frame:
            return np.eye(4, dtype=np.float32)

        try:
            transform = self._buffer.lookup_transform(
                target_frame,
                source_frame,
                Time.from_msg(stamp),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as stamped_error:
            if not self._allow_latest_fallback:
                if self._should_warn_tf():
                    self._logger.warn(
                        f"Нет stamped TF {target_frame} <- {source_frame}, пропускаю кадр; "
                        "allow_latest_tf_fallback=false. "
                        f"Детали: {stamped_error}"
                    )
                return None
            try:
                transform = self._buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
            except TransformException as latest_error:
                if self._should_warn_tf():
                    self._logger.warn(
                        f"Нет TF {target_frame} <- {source_frame}, пропускаю кадр. "
                        f"Stamped: {stamped_error}; latest: {latest_error}"
                    )
                return None
            if not self._warned_about_latest_fallback:
                self._warned_about_latest_fallback = True
                self._logger.warn(
                    f"Для статического сенсора использован latest TF {target_frame} <- "
                    f"{source_frame}: stamped TF недоступен."
                )

        self._warned_about_tf = False
        rotation = transform.transform.rotation
        translation = transform.transform.translation
        transform_matrix = np.eye(4, dtype=np.float32)
        transform_matrix[:3, :3] = quat_to_rot_matrix(
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        transform_matrix[:3, 3] = np.array(
            [translation.x, translation.y, translation.z],
            dtype=np.float32,
        )
        return transform_matrix

    def close(self) -> None:
        """Drop TF handles before the owning ROS node is destroyed."""
        if self._buffer is not None:
            self._buffer.jump_handle.unregister()
        self._listener = None
        self._buffer = None

    def _should_warn_tf(self) -> bool:
        should_warn = not self._warned_about_tf or self._frame_index() % 120 == 1
        self._warned_about_tf = True
        return should_warn


__all__ = ["StereoTransformProvider"]
