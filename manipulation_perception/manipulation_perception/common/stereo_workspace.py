"""
Workspace projection helpers for stereo point clouds.

Responsibility:
- Transform source-frame points into the configured workspace frame.
- Crop points by robot reach around the robot base.
- Transform full or workspace-cropped points into the configured output cloud frame.

Does not own:
- TF lookup or ROS frame resolution.
- Depth projection from camera intrinsics.
- PointCloud2 or MarkerArray construction.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from manipulation_perception.common.stereo_config import WorkspaceConfig
from manipulation_perception.common.transform_geometry import apply_transform


@dataclass(frozen=True)
class WorkspaceProjection:
    """Metric XYZ array paired with the frame in which it is expressed."""

    points: np.ndarray
    frame_id: str


class WorkspaceProjector:
    """
    Apply explicit homogeneous transforms and radial workspace cropping.

    Transform arguments follow ``T_target_source`` naming and points are row
    vectors in metres. The class is immutable after construction and safe to
    reuse from a serialized pipeline.
    """

    def __init__(self, config: WorkspaceConfig) -> None:
        self._config = config

    @property
    def workspace_frame(self) -> str:
        return self._config.workspace_frame

    @property
    def robot_base_frame(self) -> str:
        return self._config.robot_base_frame

    @property
    def point_cloud_frame(self) -> str:
        return self._config.point_cloud_frame

    @property
    def needs_output_transform(self) -> bool:
        return self.point_cloud_frame != self.workspace_frame

    def crop_source_points_to_workspace(
        self,
        points_source: np.ndarray,
        source_to_workspace: np.ndarray,
        base_to_workspace: np.ndarray,
    ) -> np.ndarray:
        """Return source points transformed to workspace and inside robot reach."""
        points_workspace = apply_transform(points_source, source_to_workspace)
        return points_workspace[
            self._workspace_mask(points_workspace, base_to_workspace)
        ]

    def crop_sensor_frame_points_to_workspace(
        self,
        points_source: np.ndarray,
        source_to_workspace: np.ndarray,
        base_to_workspace: np.ndarray,
    ) -> np.ndarray:
        """Apply the workspace mask while preserving sensor-frame coordinates."""
        points_workspace = apply_transform(points_source, source_to_workspace)
        return points_source[
            self._workspace_mask(points_workspace, base_to_workspace)
        ]

    def project_source_to_output(
        self,
        points_source: np.ndarray,
        source_to_workspace: np.ndarray,
        workspace_to_output: np.ndarray | None = None,
    ) -> WorkspaceProjection:
        """Transform the full source cloud without applying the workspace mask."""
        points_workspace = apply_transform(points_source, source_to_workspace)
        return self.project_output(points_workspace, workspace_to_output)

    def _workspace_mask(
        self,
        points_workspace: np.ndarray,
        base_to_workspace: np.ndarray,
    ) -> np.ndarray:
        base_position = base_to_workspace[:3, 3]
        workspace_dist = np.linalg.norm(points_workspace - base_position, axis=1)
        return workspace_dist <= self._config.workspace_radius_m

    def project_output(
        self,
        workspace_points: np.ndarray,
        workspace_to_output: np.ndarray | None = None,
    ) -> WorkspaceProjection:
        """
        Express workspace points in the configured public cloud frame.

        ``workspace_to_output`` is ``T_output_workspace`` and is mandatory when
        the configured frames differ.
        """
        if not self.needs_output_transform:
            return WorkspaceProjection(workspace_points, self.workspace_frame)
        if workspace_to_output is None:
            raise ValueError("workspace_to_output is required when output frame differs")
        return WorkspaceProjection(
            apply_transform(workspace_points, workspace_to_output),
            self.point_cloud_frame,
        )
