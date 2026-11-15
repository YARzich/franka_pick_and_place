"""
Coordinate the public world cloud and internal sensor-frame Octomap cloud.

Responsibility:
- Publish both cloud contracts through the owning ROS output adapter.
- Keep cloud-specific warnings outside the depth/state coordinator.

Does not own:
- Publishers, subscriptions, TF buffers or depth/ROI state.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import numpy as np

from manipulation_perception.common.stereo_cloud_pipeline import (
    CloudPipelineFailure,
    SourceDepthProjection,
    StereoCloudPipeline,
    TransformLookup,
)
from manipulation_perception.common.stereo_sensor_cloud import build_sensor_frame_cloud

Intrinsics = tuple[float | None, float | None, float | None, float | None]


class StereoCloudOutputPath:
    """Publish full public and workspace-cropped Octomap cloud contracts."""

    def __init__(
        self,
        *,
        cloud_pipeline: StereoCloudPipeline,
        outputs: Any,
        transform_lookup: TransformLookup,
        minimum_sensor_points: int,
        logger: Any,
        context_ok: Callable[[], bool],
        frame_index: Callable[[], int],
    ) -> None:
        self._cloud_pipeline = cloud_pipeline
        self._outputs = outputs
        self._transform_lookup = transform_lookup
        self._minimum_sensor_points = max(1, int(minimum_sensor_points))
        self._logger = logger
        self._context_ok = context_ok
        self._frame_index = frame_index
        self._warned_about_intrinsics = False
        self._warned_about_clustering = False
        self._warned_about_sparse_sensor_cloud = False

    def prepare_projection(
        self,
        depth: np.ndarray,
        intrinsics: Intrinsics,
    ) -> SourceDepthProjection:
        """Project a depth frame once for its synchronous cloud consumers."""
        return self._cloud_pipeline.prepare_projection(depth, *intrinsics)

    def publish_sensor_frame_cloud(
        self,
        depth: np.ndarray,
        source_header: Any,
        intrinsics: Intrinsics,
        projection: SourceDepthProjection | None = None,
    ) -> bool:
        """Publish the pre-ROI optical-frame cloud and report completion."""
        if not self._context_ok():
            return False
        result = build_sensor_frame_cloud(
            depth,
            intrinsics=intrinsics,
            source_frame=source_header.frame_id,
            stamp=source_header.stamp,
            minimum_points=self._minimum_sensor_points,
            cloud_pipeline=self._cloud_pipeline,
            transform_lookup=self._transform_lookup,
            projection=projection,
        )
        if result.point_cloud is not None:
            self._warned_about_sparse_sensor_cloud = False
            self._outputs.publish_octomap_cloud(result.point_cloud)
            return True
        if result.failure == CloudPipelineFailure.INSUFFICIENT_POINTS and (
            not self._warned_about_sparse_sensor_cloud
            or self._frame_index() % 120 == 1
        ):
            self._warned_about_sparse_sensor_cloud = True
            self._logger.warn(
                "Octomap cloud не публикуется: после валидации/workspace осталось "
                f"{result.point_count} точек, требуется минимум "
                f"{self._minimum_sensor_points}."
            )
        return False

    def publish_public_cloud(
        self,
        depth: np.ndarray,
        source_header: Any,
        intrinsics: Intrinsics,
        projection: SourceDepthProjection | None = None,
    ) -> bool:
        """Publish the configurable-frame cloud and report completion."""
        if not self._context_ok():
            return False
        result = self._cloud_pipeline.build(
            depth,
            fx=intrinsics[0],
            fy=intrinsics[1],
            cx=intrinsics[2],
            cy=intrinsics[3],
            source_frame=source_header.frame_id,
            stamp=source_header.stamp,
            transform_lookup=self._transform_lookup,
            include_cluster_markers=self._outputs.cluster_markers is not None,
            projection=projection,
        )
        if result.point_cloud is not None:
            self._outputs.publish_cloud(result.point_cloud, result.cluster_markers)
        elif result.failure == CloudPipelineFailure.MISSING_INTRINSICS:
            self._warn_missing_intrinsics_once()

        if result.failure == CloudPipelineFailure.MISSING_SOURCE_FRAME:
            self._logger.warn(
                "У входного depth/disparity пустой frame_id, "
                "пропускаю кадр.",
                throttle_duration_sec=3.0,
            )
        if result.clusters_filtered_by_size and (
            not self._warned_about_clustering or self._frame_index() % 120 == 1
        ):
            self._warned_about_clustering = True
            self._logger.warn(
                "Кластеры найдены, но отфильтрованы "
                "по размеру. Подстройте cluster_* параметры."
            )
        return result.point_cloud is not None and result.point_cloud.width > 0

    def _warn_missing_intrinsics_once(self) -> None:
        if self._warned_about_intrinsics:
            return
        self._warned_about_intrinsics = True
        self._logger.warn(
            "Калибровка камеры ещё не получена, "
            "point cloud временно не публикуется."
        )


__all__ = ["StereoCloudOutputPath"]
