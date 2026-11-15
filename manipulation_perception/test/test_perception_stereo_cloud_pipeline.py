"""
Test the pure stereo point-cloud processing pipeline.

Responsibility: Verify projection ownership, TF order, output frames, workspace, and clusters.
"""

from pathlib import Path
import sys

from builtin_interfaces.msg import Time
import numpy as np


_SCRIPTS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SCRIPTS_DIR))

from manipulation_perception.common.stereo_cloud_pipeline import (  # noqa: E402,I100,I101,I201
    CloudPipelineFailure,
    SourceDepthProjection,
    StereoCloudPipeline,
)
from manipulation_perception.common.stereo_config import (  # noqa: E402,I100,I101,I201
    ClusterMarkerConfig,
    WorkspaceConfig,
)
from visualization_msgs.msg import Marker  # noqa: E402,I100,I101,I201


def _workspace_config(point_cloud_frame: str = 'world') -> WorkspaceConfig:
    return WorkspaceConfig.normalized(
        workspace_frame='world',
        robot_base_frame='panda_link0',
        point_cloud_frame=point_cloud_frame,
        robot_reach_m=5.0,
        workspace_extra_ratio=0.0,
    )


def _cluster_config(*, max_diagonal_m: float = 1.0) -> ClusterMarkerConfig:
    return ClusterMarkerConfig.normalized(
        voxel_size_m=0.02,
        min_points=10,
        max_points=100,
        min_diagonal_m=0.001,
        max_diagonal_m=max_diagonal_m,
        marker_min_edge_m=0.002,
        marker_alpha=0.5,
    )


def test_cloud_pipeline_reports_missing_intrinsics_without_tf_lookup():
    pipeline = StereoCloudPipeline(_workspace_config(), _cluster_config(), 1)
    lookup_calls = []

    result = pipeline.build(
        np.ones((2, 2), dtype=np.float32),
        fx=None,
        fy=None,
        cx=None,
        cy=None,
        source_frame='camera',
        stamp=Time(sec=1),
        transform_lookup=lambda *args: lookup_calls.append(args),
        include_cluster_markers=False,
    )

    assert result.failure == CloudPipelineFailure.MISSING_INTRINSICS
    assert result.point_cloud is None
    assert result.cluster_markers is None
    assert lookup_calls == []


def test_cloud_pipeline_preserves_output_frame_and_transform_order():
    pipeline = StereoCloudPipeline(
        _workspace_config(point_cloud_frame='camera_debug'),
        _cluster_config(),
        1,
    )
    lookup_calls = []

    def lookup(target_frame, source_frame, stamp):
        lookup_calls.append((target_frame, source_frame, stamp.sec))
        transform = np.eye(4, dtype=np.float32)
        if target_frame == 'camera_debug':
            transform[0, 3] = 1.0
        return transform

    result = pipeline.build(
        np.ones((2, 2), dtype=np.float32),
        fx=1.0,
        fy=1.0,
        cx=0.0,
        cy=0.0,
        source_frame='camera',
        stamp=Time(sec=2),
        transform_lookup=lookup,
        include_cluster_markers=True,
    )

    assert result.failure is None
    assert result.point_cloud is not None
    assert result.point_cloud.header.frame_id == 'camera_debug'
    assert result.point_cloud.width == 4
    assert result.cluster_markers is not None
    assert result.cluster_markers.markers[0].action == Marker.DELETEALL
    assert result.cluster_markers.markers[0].header.frame_id == 'camera_debug'
    assert lookup_calls == [
        ('world', 'camera', 2),
        ('camera_debug', 'world', 2),
    ]


def test_public_cloud_keeps_points_outside_robot_workspace():
    workspace = WorkspaceConfig.normalized(
        workspace_frame='world',
        robot_base_frame='panda_link0',
        point_cloud_frame='world',
        robot_reach_m=0.1,
        workspace_extra_ratio=0.0,
    )
    pipeline = StereoCloudPipeline(workspace, _cluster_config(), 1)
    projection = SourceDepthProjection(
        np.array(
            [
                [0.0, 0.0, 0.05],
                [2.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        ),
    )
    lookup_calls = []

    def lookup(target_frame, source_frame, stamp):
        lookup_calls.append((target_frame, source_frame, stamp.sec))
        return np.eye(4, dtype=np.float32)

    result = pipeline.build(
        np.full((1, 1), np.nan, dtype=np.float32),
        fx=None,
        fy=None,
        cx=None,
        cy=None,
        source_frame='camera',
        stamp=Time(sec=2),
        transform_lookup=lookup,
        include_cluster_markers=False,
        projection=projection,
    )

    assert result.failure is None
    assert result.point_cloud is not None
    assert result.point_cloud.width == 2
    assert lookup_calls == [('world', 'camera', 2)]


def test_cloud_pipeline_consumes_prepared_projection_without_reprojecting():
    pipeline = StereoCloudPipeline(_workspace_config(), _cluster_config(), 1)
    projection = SourceDepthProjection(
        np.array([[0.0, 0.0, 1.0]], dtype=np.float32),
    )

    def unexpected_projection(*_args):
        raise AssertionError('projection must be reused')

    pipeline.project_depth = unexpected_projection  # type: ignore[method-assign]

    result = pipeline.build(
        np.full((1, 1), np.nan, dtype=np.float32),
        fx=None,
        fy=None,
        cx=None,
        cy=None,
        source_frame='camera',
        stamp=Time(sec=2),
        transform_lookup=lambda *_args: np.eye(4, dtype=np.float32),
        include_cluster_markers=False,
        projection=projection,
    )

    assert result.failure is None
    assert result.point_cloud is not None
    assert result.point_cloud.width == 1


def test_cloud_pipeline_reports_clusters_filtered_by_marker_size():
    pipeline = StereoCloudPipeline(
        _workspace_config(),
        _cluster_config(max_diagonal_m=0.02),
        1,
    )

    result = pipeline.build(
        np.ones((1, 10), dtype=np.float32),
        fx=100.0,
        fy=100.0,
        cx=0.0,
        cy=0.0,
        source_frame='camera',
        stamp=Time(sec=3),
        transform_lookup=lambda *_args: np.eye(4, dtype=np.float32),
        include_cluster_markers=True,
    )

    assert result.failure is None
    assert result.cluster_markers is not None
    assert len(result.cluster_markers.markers) == 1
    assert result.clusters_filtered_by_size is True
