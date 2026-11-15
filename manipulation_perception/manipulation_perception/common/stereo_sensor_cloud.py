"""
Build the workspace-cropped optical-frame cloud used for Octomap rays.

Responsibility:
- Preserve the stamped sensor frame while applying the shared workspace mask.

Does not own:
- ROS publishers, TF buffers, detection ROI or the public visualization cloud.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from sensor_msgs.msg import PointCloud2

from manipulation_perception.common.stereo_cloud_pipeline import (
    CloudPipelineFailure,
    SourceDepthProjection,
    StereoCloudPipeline,
    TransformLookup,
)
from manipulation_perception.common.stereo_messages import PointCloud2Builder

# Stereo depth produces sparse "flying pixels" along object silhouettes. A
# single such point becomes a persistent occupied Octomap voxel floating in
# free air and later blocks motion planning, while real surfaces project tens
# of points per voxel. Requiring a few co-located points per voxel removes
# that noise without adding a tunable task parameter.
_NOISE_VOXEL_SIZE_M = 0.03
_NOISE_MINIMUM_VOXEL_POINTS = 3


def _remove_isolated_points(points: np.ndarray) -> np.ndarray:
    """Drop points whose voxel holds fewer than the required neighbors."""
    if points.shape[0] == 0:
        return points
    voxel_keys = np.floor(points / _NOISE_VOXEL_SIZE_M).astype(np.int64)
    _, inverse_indices, counts = np.unique(
        voxel_keys, axis=0, return_inverse=True, return_counts=True
    )
    return points[counts[inverse_indices] >= _NOISE_MINIMUM_VOXEL_POINTS]


@dataclass(frozen=True)
class SensorFrameCloudOutput:
    """One source-frame cloud or the validation/TF failure that suppressed it."""

    point_cloud: PointCloud2 | None
    failure: CloudPipelineFailure | None = None
    point_count: int = 0


def build_sensor_frame_cloud(
    depth: np.ndarray,
    *,
    intrinsics: tuple[float | None, float | None, float | None, float | None],
    source_frame: str,
    stamp: Any,
    minimum_points: int,
    cloud_pipeline: StereoCloudPipeline,
    transform_lookup: TransformLookup,
    projection: SourceDepthProjection | None = None,
) -> SensorFrameCloudOutput:
    """
    Build an Octomap input without losing the physical sensor origin.

    MoveIt's PointCloudOctomapUpdater derives the ray origin from the cloud
    header frame.  Feeding it the public cloud after that cloud is transformed
    to ``world`` would therefore make every ray start at the world origin.
    """
    if projection is None:
        projection = cloud_pipeline.prepare_projection(depth, *intrinsics)
    points_source = projection.points_source
    if points_source is None:
        return SensorFrameCloudOutput(None, CloudPipelineFailure.MISSING_INTRINSICS)
    required_points = max(1, int(minimum_points))
    source_point_count = int(points_source.shape[0])
    if source_point_count < required_points:
        return SensorFrameCloudOutput(
            None,
            CloudPipelineFailure.INSUFFICIENT_POINTS,
            source_point_count,
        )
    if not source_frame:
        return SensorFrameCloudOutput(
            None,
            CloudPipelineFailure.MISSING_SOURCE_FRAME,
            source_point_count,
        )

    source_to_workspace = transform_lookup(
        cloud_pipeline.workspace_frame,
        source_frame,
        stamp,
    )
    if source_to_workspace is None:
        return SensorFrameCloudOutput(
            None,
            CloudPipelineFailure.MISSING_TRANSFORM,
            source_point_count,
        )
    base_to_workspace = transform_lookup(
        cloud_pipeline.workspace_frame,
        cloud_pipeline.robot_base_frame,
        stamp,
    )
    if base_to_workspace is None:
        return SensorFrameCloudOutput(
            None,
            CloudPipelineFailure.MISSING_TRANSFORM,
            source_point_count,
        )

    cropped_source_points = cloud_pipeline.crop_sensor_frame_to_workspace(
        points_source,
        source_to_workspace,
        base_to_workspace,
    )
    cropped_source_points = _remove_isolated_points(cropped_source_points)
    cropped_point_count = int(cropped_source_points.shape[0])
    if cropped_point_count < required_points:
        return SensorFrameCloudOutput(
            None,
            CloudPipelineFailure.INSUFFICIENT_POINTS,
            cropped_point_count,
        )
    point_cloud = PointCloud2Builder().build(
        cropped_source_points,
        stamp,
        source_frame,
    )
    return SensorFrameCloudOutput(point_cloud, point_count=cropped_point_count)


__all__ = ["SensorFrameCloudOutput", "build_sensor_frame_cloud"]
