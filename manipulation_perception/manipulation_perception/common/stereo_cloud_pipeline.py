"""
Point-cloud projection pipeline for the stereo perception node.

Responsibility:
- Project depth samples once into a callback-scoped camera-frame value.
- Transform the public cloud into its configured output frame.
- Expose workspace cropping for the Octomap and table-plane paths.
- Cluster output points and build PointCloud2/MarkerArray messages.

Does not own:
- ROS parameters, publishers, subscriptions or TF buffers.
- Runtime logging or shutdown state.
- Table-plane detection.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from typing import Any

import numpy as np
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray

from manipulation_perception.common.depth_geometry import project_depth_to_points
from manipulation_perception.common.stereo_config import ClusterMarkerConfig, WorkspaceConfig
from manipulation_perception.common.stereo_messages import ClusterMarkerBuilder, PointCloud2Builder
from manipulation_perception.common.stereo_workspace import WorkspaceProjector
from manipulation_perception.common.voxel_clustering import cluster_points_voxel


TransformLookup = Callable[[str, str, Any], np.ndarray | None]


class CloudPipelineFailure(Enum):
    """Typed reason why a stamped depth frame produced no public cloud."""

    MISSING_INTRINSICS = 'missing_intrinsics'
    MISSING_SOURCE_FRAME = 'missing_source_frame'
    MISSING_TRANSFORM = 'missing_transform'
    INSUFFICIENT_POINTS = 'insufficient_points'


@dataclass(frozen=True)
class SourceDepthProjection:
    """
    Borrow one projected XYZ array across outputs from the same depth frame.

    `points_source=None` means camera intrinsics were unavailable. The array is
    pipeline-local and consumers must treat it as read-only; retaining it only
    for the synchronous processing of one stamped frame avoids repeated high-rate
    projection without introducing a cache or cross-callback state.
    """

    points_source: np.ndarray | None

    @property
    def point_count(self) -> int:
        """Return the finite projected sample count for metrics and health policy."""
        return 0 if self.points_source is None else int(self.points_source.shape[0])


@dataclass(frozen=True)
class StereoCloudOutput:
    """
    Messages and typed failure produced from one stamped depth sample.

    The point cloud and markers, when present, share the requested ROS stamp
    and output frame.  ``failure`` is set only when no cloud can be emitted.
    """

    point_cloud: PointCloud2 | None
    cluster_markers: MarkerArray | None
    failure: CloudPipelineFailure | None = None
    clusters_filtered_by_size: bool = False


class StereoCloudPipeline:
    """Pure geometry/message pipeline with externally supplied TF lookup."""

    def __init__(
        self,
        workspace_config: WorkspaceConfig,
        cluster_marker_config: ClusterMarkerConfig,
        point_cloud_stride: int,
    ) -> None:
        self._point_cloud_stride = max(1, int(point_cloud_stride))
        self._cluster_marker_config = cluster_marker_config
        self._workspace_projector = WorkspaceProjector(workspace_config)
        self._cloud_builder = PointCloud2Builder()
        self._cluster_marker_builder = ClusterMarkerBuilder(cluster_marker_config)

    @property
    def workspace_frame(self) -> str:
        return self._workspace_projector.workspace_frame

    @property
    def robot_base_frame(self) -> str:
        return self._workspace_projector.robot_base_frame

    @property
    def point_cloud_frame(self) -> str:
        return self._workspace_projector.point_cloud_frame

    def project_depth(
        self,
        depth: np.ndarray,
        fx: float | None,
        fy: float | None,
        cx: float | None,
        cy: float | None,
    ) -> np.ndarray | None:
        """Project metric depth into XYZ points expressed in the camera frame."""
        return project_depth_to_points(
            depth,
            fx,
            fy,
            cx,
            cy,
            self._point_cloud_stride,
        )

    def prepare_projection(
        self,
        depth: np.ndarray,
        fx: float | None,
        fy: float | None,
        cx: float | None,
        cy: float | None,
    ) -> SourceDepthProjection:
        """Project one depth frame once for synchronous downstream consumers."""
        return SourceDepthProjection(self.project_depth(depth, fx, fy, cx, cy))

    def crop_to_workspace(
        self,
        points_source: np.ndarray,
        source_to_workspace: np.ndarray,
        base_to_workspace: np.ndarray,
    ) -> np.ndarray:
        """
        Transform source points into the workspace frame and crop by reach.

        Both transforms are homogeneous matrices named as
        ``T_workspace_source`` and ``T_workspace_base``. Returned XYZ points
        are expressed in ``workspace_frame`` and measured in metres.
        """
        return self._workspace_projector.crop_source_points_to_workspace(
            points_source,
            source_to_workspace,
            base_to_workspace,
        )

    def crop_sensor_frame_to_workspace(
        self,
        points_source: np.ndarray,
        source_to_workspace: np.ndarray,
        base_to_workspace: np.ndarray,
    ) -> np.ndarray:
        """Crop by workspace without changing the source-frame coordinates."""
        return self._workspace_projector.crop_sensor_frame_points_to_workspace(
            points_source,
            source_to_workspace,
            base_to_workspace,
        )

    def build(
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
        include_cluster_markers: bool,
        projection: SourceDepthProjection | None = None,
    ) -> StereoCloudOutput:
        """
        Build cloud outputs using TF at the depth sample's ROS timestamp.

        ``source_frame`` is the camera frame of the projected depth.  The
        supplied lookup must return ``T_target_source`` for the exact ``stamp``;
        this pipeline never substitutes latest TF.  All returned coordinates
        are metric and stamped with the original sample time.
        """
        if projection is None:
            projection = self.prepare_projection(depth, fx, fy, cx, cy)
        points_source = projection.points_source
        if points_source is None:
            return StereoCloudOutput(None, None, CloudPipelineFailure.MISSING_INTRINSICS)
        if not source_frame:
            return StereoCloudOutput(None, None, CloudPipelineFailure.MISSING_SOURCE_FRAME)

        source_to_workspace = transform_lookup(self.workspace_frame, source_frame, stamp)
        if source_to_workspace is None:
            return StereoCloudOutput(None, None, CloudPipelineFailure.MISSING_TRANSFORM)
        workspace_to_output = None
        if self._workspace_projector.needs_output_transform:
            workspace_to_output = transform_lookup(
                self.point_cloud_frame,
                self.workspace_frame,
                stamp,
            )
            if workspace_to_output is None:
                return StereoCloudOutput(None, None, CloudPipelineFailure.MISSING_TRANSFORM)

        output_projection = self._workspace_projector.project_source_to_output(
            points_source,
            source_to_workspace,
            workspace_to_output,
        )
        point_cloud = self._cloud_builder.build(
            output_projection.points,
            stamp,
            output_projection.frame_id,
        )
        if not include_cluster_markers:
            return StereoCloudOutput(point_cloud, None)

        clusters = cluster_points_voxel(
            output_projection.points,
            self._cluster_marker_config.voxel_size_m,
            self._cluster_marker_config.min_points,
            self._cluster_marker_config.max_points,
        )
        markers = self._cluster_marker_builder.build(
            output_projection.points,
            clusters,
            stamp,
            output_projection.frame_id,
        )
        clusters_filtered_by_size = bool(clusters) and len(markers.markers) == 1
        return StereoCloudOutput(
            point_cloud,
            markers,
            clusters_filtered_by_size=clusters_filtered_by_size,
        )


__all__ = [
    "CloudPipelineFailure",
    "SourceDepthProjection",
    "StereoCloudOutput",
    "StereoCloudPipeline",
    "TransformLookup",
]
