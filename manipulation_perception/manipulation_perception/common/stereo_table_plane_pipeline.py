"""
Stateful table-plane output pipeline for stereo depth frames.

Responsibility:
- Consume a shared depth projection and crop points for table-plane detection.
- Own RANSAC detection, warmup samples and one-shot lock state.
- Build table-plane MarkerArray and coefficient messages.

Does not own:
- ROS publishers, parameters, TF buffers or logging.
- Point-cloud/cluster output construction.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import MarkerArray

from manipulation_perception.common.stereo_cloud_pipeline import (
    SourceDepthProjection,
    StereoCloudPipeline,
    TransformLookup,
)
from manipulation_perception.common.stereo_config import TablePlaneConfig
from manipulation_perception.common.stereo_messages import (
    table_plane_coeffs_message,
    TablePlaneMarkerBuilder,
)
from manipulation_perception.common.stereo_table_plane import TablePlaneDetector
from manipulation_perception.common.table_plane_tracker import TablePlaneOutput, TablePlaneTracker


@dataclass(frozen=True)
class StereoTablePlaneOutput:
    """Optional stamped ROS outputs and state transition from one depth sample."""

    marker: MarkerArray | None = None
    coeffs: Float64MultiArray | None = None
    detection_failure: str | None = None
    estimate_found: bool = False
    just_locked: bool = False


class StereoTablePlanePipeline:
    """Detect and freeze the tabletop plane without owning ROS transport."""

    def __init__(
        self,
        config: TablePlaneConfig,
        cloud_pipeline: StereoCloudPipeline,
    ) -> None:
        self._config = config
        self._cloud_pipeline = cloud_pipeline
        self._up_vector = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        self._detector = TablePlaneDetector(
            config,
            self._up_vector,
            np.random.default_rng(42),
        )
        self._tracker = TablePlaneTracker(config, self._up_vector)
        self._marker_builder = TablePlaneMarkerBuilder(config)

    @staticmethod
    def _stamp_seconds(stamp: Any) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    def _messages_for_output(
        self,
        output: TablePlaneOutput,
        stamp: Any,
        *,
        include_marker: bool,
        include_coeffs: bool,
    ) -> StereoTablePlaneOutput:
        """Build enabled ROS messages from one accepted tracker output."""
        marker = None
        if include_marker:
            marker = self._marker_builder.build(
                output.bounds,
                stamp,
                self._cloud_pipeline.workspace_frame,
            )
        coeffs = table_plane_coeffs_message(output.coeffs) if include_coeffs else None
        return StereoTablePlaneOutput(
            marker=marker,
            coeffs=coeffs,
            estimate_found=True,
            just_locked=output.just_locked,
        )

    def _clear_marker(
        self,
        stamp: Any,
        include_marker: bool,
    ) -> MarkerArray | None:
        if not include_marker:
            return None
        return TablePlaneMarkerBuilder.clear(
            stamp,
            self._cloud_pipeline.workspace_frame,
        )

    def process(
        self,
        depth: np.ndarray,
        *,
        fx: float | None,
        fy: float | None,
        cx: float | None,
        cy: float | None,
        source_frame: str,
        stamp: Any,
        transform_lookup: TransformLookup,
        include_marker: bool,
        include_coeffs: bool,
        projection: SourceDepthProjection | None = None,
    ) -> StereoTablePlaneOutput:
        """
        Update the table estimate using TF at the depth sample timestamp.

        ``transform_lookup`` must return exact ``T_target_source`` transforms;
        absent TF produces no estimate and may emit a marker delete. When
        ``detect_once`` locks, later calls reuse coefficients but restamp output
        messages for the current sample.
        """
        locked_output = self._tracker.locked_output()
        if self._config.detect_once and self._tracker.locked:
            if locked_output is None:
                return StereoTablePlaneOutput()
            return self._messages_for_output(
                locked_output,
                stamp,
                include_marker=include_marker,
                include_coeffs=include_coeffs,
            )

        if not self._tracker.ready_for_detection(self._stamp_seconds(stamp)):
            return StereoTablePlaneOutput()

        if projection is None:
            projection = self._cloud_pipeline.prepare_projection(depth, fx, fy, cx, cy)
        points_source = projection.points_source
        if points_source is None or not source_frame:
            return StereoTablePlaneOutput()

        source_to_workspace = transform_lookup(
            self._cloud_pipeline.workspace_frame,
            source_frame,
            stamp,
        )
        if source_to_workspace is None:
            return StereoTablePlaneOutput(
                marker=self._clear_marker(stamp, include_marker),
            )
        base_to_workspace = transform_lookup(
            self._cloud_pipeline.workspace_frame,
            self._cloud_pipeline.robot_base_frame,
            stamp,
        )
        if base_to_workspace is None:
            return StereoTablePlaneOutput(
                marker=self._clear_marker(stamp, include_marker),
            )

        workspace_points = self._cloud_pipeline.crop_to_workspace(
            points_source,
            source_to_workspace,
            base_to_workspace,
        )
        detection = self._detector.detect(workspace_points)
        if detection.estimate is None:
            clear_marker = detection.failure in {'not_enough_points', 'no_plane'}
            return StereoTablePlaneOutput(
                marker=self._clear_marker(stamp, include_marker) if clear_marker else None,
                detection_failure=detection.failure,
            )

        estimate = detection.estimate
        output = self._tracker.add_sample(estimate.bounds, estimate.coeffs)
        if output is None:
            return StereoTablePlaneOutput(estimate_found=True)
        return self._messages_for_output(
            output,
            stamp,
            include_marker=include_marker,
            include_coeffs=include_coeffs,
        )
