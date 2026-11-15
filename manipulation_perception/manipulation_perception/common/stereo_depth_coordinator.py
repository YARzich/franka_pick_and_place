"""
Coordinate one decoded depth frame through ROI, table and cloud outputs.

Responsibility:
    Apply stamped detection ROI, TF, table-plane, and cloud-output stages to one
    decoded depth sample while recording stage metrics and drop reasons.

Does not own:
    ROS subscriptions, image decoding, or publishers.
"""

from __future__ import annotations

import time
from typing import Any, Callable

import numpy as np

from manipulation_perception.common.pending_depth_frames import PendingDepthFrames
from manipulation_perception.common.stereo_cloud_output_path import StereoCloudOutputPath
from manipulation_perception.common.stereo_cloud_pipeline import (
    SourceDepthProjection,
    StereoCloudPipeline,
)
from manipulation_perception.common.stereo_detection_roi import DetectionRoiFilter
from manipulation_perception.common.stereo_table_plane_pipeline import StereoTablePlanePipeline


Intrinsics = tuple[float | None, float | None, float | None, float | None]
_MISSING_INTRINSICS: Intrinsics = (None, None, None, None)


class StereoDepthCoordinator:
    """
    Apply exact stamped ROI gating and publish derived depth/cloud products.

    The coordinator borrows ROS publishers, `CvBridge`, clock and TF callbacks;
    it owns only bounded pending-frame state and pure processing pipelines. All
    methods are callback-serialized by the owning node. Observer callbacks mark
    terminal work/output events but never influence processing decisions.
    """

    def __init__(
        self,
        *,
        parameters: Any,
        bridge: Any,
        outputs: Any,
        transform_lookup: Callable[[str, str, Any], np.ndarray | None],
        clock: Any,
        logger: Any,
        context_ok: Callable[[], bool],
        frame_index: Callable[[], int],
        on_output: Callable[[Any, bool], None] | None = None,
        on_drop_or_skip: Callable[[int], None] | None = None,
        begin_processing: Callable[[], float] | None = None,
        record_processed: Callable[[float], None] | None = None,
        on_error: Callable[[], None] | None = None,
    ) -> None:
        self._bridge = bridge
        self._outputs = outputs
        self._transform_lookup = transform_lookup
        self._clock = clock
        self._logger = logger
        self._context_ok = context_ok
        self._frame_index = frame_index
        self._on_output = on_output or (lambda _header, _successful: None)
        self._on_drop_or_skip = on_drop_or_skip or (lambda _count: None)
        self._begin_processing = begin_processing or time.monotonic
        self._record_processed = record_processed or (lambda _started_at: None)
        self._on_error = on_error or (lambda: None)
        self._shutting_down = False
        self._warned_about_missing_bbox = False
        self._warned_about_plane = False

        self._roi_filter = DetectionRoiFilter(
            parameters.use_detection_bbox_roi,
            parameters.detection_bbox_timeout_sec,
        )
        self._pending_frames = PendingDepthFrames(
            response_lookup=self._roi_filter,
            timeout_sec=parameters.detection_bbox_timeout_sec,
        )
        self._cloud_pipeline = StereoCloudPipeline(
            parameters.workspace_config,
            parameters.cluster_marker_config,
            parameters.point_cloud_stride,
        )
        self._cloud_output_path = StereoCloudOutputPath(
            cloud_pipeline=self._cloud_pipeline,
            outputs=outputs,
            transform_lookup=transform_lookup,
            minimum_sensor_points=parameters.octomap_min_points,
            logger=logger,
            context_ok=lambda: not self._shutting_down and self._context_ok(),
            frame_index=frame_index,
        )
        self._table_plane_pipeline = StereoTablePlanePipeline(
            parameters.table_plane_config,
            self._cloud_pipeline,
        )

    @property
    def roi_enabled(self) -> bool:
        """Return whether depth waits for an exact stamped detection."""
        return self._roi_filter.enabled

    def update_detection(self, message: Any) -> None:
        """Store stamped detection data and release any matching depth frame."""
        if self._shutting_down or not self._context_ok():
            return
        now_ros_sec = self._clock.now().nanoseconds * 1.0e-9
        self._roi_filter.update_detection_array(message, now_ros_sec)
        self.process_pending_depth_frames()

    def process_depth(
        self,
        depth: np.ndarray,
        source_header: Any,
        *,
        started_at: float | None = None,
        intrinsics: Intrinsics | None = None,
    ) -> None:
        """Publish one depth sample with an immutable calibration snapshot."""
        processing_started_at = self._begin_processing() if started_at is None else float(started_at)
        sample_intrinsics = intrinsics if intrinsics is not None else _MISSING_INTRINSICS
        queued = False
        try:
            queued = self._queue_until_matching_detection(
                depth,
                source_header,
                processing_started_at,
                sample_intrinsics,
            )
            if not queued:
                self._publish_depth(depth, source_header, sample_intrinsics)
        except Exception:
            self._on_error()
            raise
        finally:
            if not queued:
                self._record_processed(processing_started_at)

    def process_pending_depth_frames(self) -> None:
        """Release matching/expired bounded ROI queue entries."""
        if self._shutting_down or not self._roi_filter.enabled:
            return
        ready_frames = self._pending_frames.release_ready(
            now_ros_sec=self._clock.now().nanoseconds * 1.0e-9,
            now_monotonic=time.monotonic(),
        )
        for frame in ready_frames:
            try:
                self._publish_depth(frame.depth, frame.source_header, frame.intrinsics)
            except Exception:
                self._on_error()
                raise
            finally:
                self._record_processed(frame.received_at_monotonic)

    def close(self) -> None:
        """Reject new work and clear all borrowed sensor messages."""
        self._shutting_down = True
        self._pending_frames.clear()

    def _queue_until_matching_detection(
        self,
        depth: np.ndarray,
        source_header: Any,
        processing_started_at: float,
        intrinsics: Intrinsics,
    ) -> bool:
        add_result = self._pending_frames.add_if_waiting(
            depth,
            source_header,
            now_ros_sec=self._clock.now().nanoseconds * 1.0e-9,
            received_at_monotonic=processing_started_at,
            intrinsics=intrinsics,
        )
        for dropped_frame in add_result.dropped:
            expired_header = dropped_frame.source_header
            self._logger.warn(
                "Depth/point cloud ROI queue переполнена; пропускаю самый старый depth кадр "
                f"stamp={expired_header.stamp.sec}.{expired_header.stamp.nanosec:09d}.",
                throttle_duration_sec=3.0,
            )
        if add_result.dropped:
            self._on_drop_or_skip(len(add_result.dropped))
            for dropped_frame in add_result.dropped:
                self._record_processed(dropped_frame.received_at_monotonic)
        return add_result.queued

    def _publish_depth(
        self,
        depth: np.ndarray,
        source_header: Any,
        intrinsics: Intrinsics,
    ) -> None:
        source_projection = self._cloud_output_path.prepare_projection(depth, intrinsics)
        self._publish_table_plane(depth, source_header, intrinsics, source_projection)
        sensor_cloud_published = self._cloud_output_path.publish_sensor_frame_cloud(
            depth,
            source_header,
            intrinsics,
            projection=source_projection,
        )
        output_depth, successful = self._apply_roi(depth, source_header)
        depth_message = self._bridge.cv2_to_imgmsg(output_depth, encoding="32FC1")
        depth_message.header = source_header
        self._outputs.publish_depth(depth_message)

        # A disabled ROI returns the same array, so the raw projection is valid
        # for both clouds. An active ROI changes sample membership and must be
        # projected independently while still reusing the same pipeline code.
        public_projection = source_projection if output_depth is depth else None
        public_cloud_published = self._cloud_output_path.publish_public_cloud(
            output_depth,
            source_header,
            intrinsics,
            projection=public_projection,
        )
        self._on_output(
            source_header,
            successful and sensor_cloud_published and public_cloud_published,
        )

    def _apply_roi(self, depth: np.ndarray, source_header: Any) -> tuple[np.ndarray, bool]:
        if self._shutting_down or not self._context_ok():
            return np.full_like(depth, np.nan, dtype=np.float32), False
        roi_result = self._roi_filter.apply(
            depth,
            self._clock.now().nanoseconds * 1.0e-9,
            source_header,
        )
        if roi_result.missing_required_bbox:
            if not self._warned_about_missing_bbox or self._frame_index() % 120 == 1:
                self._warned_about_missing_bbox = True
                self._logger.warn(
                    "Нет актуального stamped bbox для текущего depth кадра: "
                    "depth/point cloud временно пустые."
                )
            return roi_result.depth, False
        self._warned_about_missing_bbox = False
        return roi_result.depth, True

    def _publish_table_plane(
        self,
        depth: np.ndarray,
        source_header: Any,
        intrinsics: Intrinsics,
        projection: SourceDepthProjection,
    ) -> None:
        if not self._outputs.has_table_plane_outputs:
            return
        result = self._table_plane_pipeline.process(
            depth,
            fx=intrinsics[0],
            fy=intrinsics[1],
            cx=intrinsics[2],
            cy=intrinsics[3],
            source_frame=source_header.frame_id,
            stamp=source_header.stamp,
            transform_lookup=self._transform_lookup,
            include_marker=self._outputs.table_plane_marker is not None,
            include_coeffs=self._outputs.table_plane_coeffs is not None,
            projection=projection,
        )
        if result.detection_failure == "no_plane" and (
            not self._warned_about_plane or self._frame_index() % 120 == 1
        ):
            self._warned_about_plane = True
            self._logger.warn(
                "Не удалось найти плоскость стола по RANSAC до обрезки по bbox."
            )
        if result.estimate_found:
            self._warned_about_plane = False
        self._outputs.publish_table_plane(result.marker, result.coeffs)
        if result.just_locked:
            self._logger.info(
                "Плоскость стола зафиксирована один раз после прогрева симуляции."
            )


__all__ = ["StereoDepthCoordinator"]
