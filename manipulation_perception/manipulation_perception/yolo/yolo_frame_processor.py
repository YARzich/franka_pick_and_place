"""
Orchestrate one decimated YOLO frame with observable stage boundaries.

Responsibility:
    Apply frame decimation, detector invocation, post-processing, stamped output,
    metrics, and optional debug rendering to one source image.

Does not own:
    ROS subscription lifecycle, model loading, or runtime control storage.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from sensor_msgs.msg import Image

from manipulation_perception.common.detection_candidate import DetectionCandidate


class YoloFrameProcessor:
    """
    Run decode, detection, publication and optional rendering in order.

    The processor borrows all backend/ROS adapters. Metrics are reported through
    callbacks, keeping this orchestration independent of diagnostic transport.
    """

    def __init__(
        self,
        *,
        bridge: Any,
        detector: Any,
        runtime_controller: Any,
        outputs: Any,
        debug_session: Any,
        logger: Any,
        record_input: Callable[[], None],
        record_drop_or_skip: Callable[[], None],
        begin_processing: Callable[[], float],
        record_processed: Callable[[float], None],
        record_error: Callable[[], None],
    ) -> None:
        self._bridge = bridge
        self._detector = detector
        self._runtime_controller = runtime_controller
        self._outputs = outputs
        self._debug_session = debug_session
        self._logger = logger
        self._record_input = record_input
        self._record_drop_or_skip = record_drop_or_skip
        self._begin_processing = begin_processing
        self._record_processed = record_processed
        self._record_error = record_error
        self._frame_counter = 0

    def process(self, message: Image) -> None:
        """Process one ROS image while preserving its header on every output."""
        self._record_input()
        self._sync_runtime_controls()
        self._frame_counter += 1
        if self._frame_counter % self._runtime_controller.state.run_every_n_frames != 0:
            self._record_drop_or_skip()
            return

        started_at = self._begin_processing()
        try:
            frame_bgr = self._decode(message)
            if frame_bgr is None:
                return
            detections = self._detect(frame_bgr, message)
            if detections is None:
                return
            try:
                self._outputs.publish(detections, frame_bgr.shape, message)
            except Exception as error:  # noqa: BLE001 - ROS publisher/runtime boundary
                self._record_error()
                self._logger.error(
                    f"Ошибка публикации YOLO outputs: {error}",
                    throttle_duration_sec=3.0,
                )
                return
            self._render(frame_bgr, detections)
        finally:
            self._record_processed(started_at)

    def _sync_runtime_controls(self) -> None:
        try:
            self._debug_session.sync_controls(self._detector, self._logger)
        except Exception as error:  # noqa: BLE001 - optional UI must not stop inference
            self._record_error()
            self._logger.error(
                f"Ошибка runtime controls YOLO: {error}",
                throttle_duration_sec=3.0,
            )

    def _decode(self, message: Image) -> Any | None:
        try:
            return self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:  # noqa: BLE001 - CvBridge boundary
            self._record_error()
            self._logger.error(
                f"Не удалось декодировать RGB для YOLO: {error}",
                throttle_duration_sec=3.0,
            )
            self._outputs.publish_empty_safely(
                message,
                frame_shape=None,
                log_error=self._logger.error,
            )
            return None

    def _detect(
        self,
        frame_bgr: Any,
        message: Image,
    ) -> list[DetectionCandidate] | None:
        try:
            return self._detector.detect(
                frame_bgr,
                self._runtime_controller.build_detector_config(),
            )
        except Exception as error:  # noqa: BLE001 - replaceable ML backend boundary
            self._record_error()
            self._logger.error(
                f"Ошибка YOLO inference: {error}",
                throttle_duration_sec=3.0,
            )
            self._outputs.publish_empty_safely(
                message,
                frame_shape=frame_bgr.shape,
                log_error=self._logger.error,
            )
            return None

    def _render(self, frame_bgr: Any, detections: list[DetectionCandidate]) -> None:
        try:
            self._debug_session.render(frame_bgr, detections)
        except Exception as error:  # noqa: BLE001 - optional UI is not pipeline output
            self._record_error()
            self._logger.error(
                f"Ошибка debug rendering YOLO: {error}",
                throttle_duration_sec=3.0,
            )


__all__ = ["YoloFrameProcessor"]
