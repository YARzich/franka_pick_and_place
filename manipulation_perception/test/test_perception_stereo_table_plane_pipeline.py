"""
Test table-plane filtering and obstacle-cloud extraction.

Responsibility: Verify warmup, cached-plane use, workspace filtering, and diagnostic outcomes.
"""

from pathlib import Path
import sys

from builtin_interfaces.msg import Time
import numpy as np


_SCRIPTS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SCRIPTS_DIR))

from manipulation_perception.common.stereo_cloud_pipeline import (  # noqa: E402,I100,I101,I201
    SourceDepthProjection,
    StereoCloudPipeline,
)
from manipulation_perception.common.stereo_config import (  # noqa: E402,I100,I101,I201
    ClusterMarkerConfig,
    TablePlaneConfig,
    WorkspaceConfig,
)
from manipulation_perception.common.stereo_table_plane_pipeline import (  # noqa: E402,I100,I101,I201
    StereoTablePlanePipeline,
)
from visualization_msgs.msg import Marker  # noqa: E402,I100,I101,I201


def _cloud_pipeline() -> StereoCloudPipeline:
    workspace = WorkspaceConfig.normalized(
        workspace_frame='world',
        robot_base_frame='panda_link0',
        point_cloud_frame='world',
        robot_reach_m=5.0,
        workspace_extra_ratio=0.0,
    )
    clusters = ClusterMarkerConfig.normalized(
        voxel_size_m=0.02,
        min_points=10,
        max_points=100,
        min_diagonal_m=0.001,
        max_diagonal_m=1.0,
        marker_min_edge_m=0.002,
        marker_alpha=0.5,
    )
    return StereoCloudPipeline(workspace, clusters, point_cloud_stride=1)


def _table_config() -> TablePlaneConfig:
    return TablePlaneConfig.normalized(
        detect_once=True,
        init_delay_sec=0.0,
        init_required_frames=1,
        ransac_iterations=20,
        distance_threshold_m=0.005,
        normal_max_tilt_deg=20.0,
        marker_thickness_m=0.004,
        marker_alpha=0.35,
    )


def _process(pipeline, depth, stamp, transform_lookup, projection=None):
    return pipeline.process(
        depth,
        fx=10.0,
        fy=10.0,
        cx=1.5,
        cy=1.5,
        source_frame='camera',
        stamp=stamp,
        transform_lookup=transform_lookup,
        include_marker=True,
        include_coeffs=True,
        projection=projection,
    )


def test_table_plane_pipeline_warms_up_then_republishes_locked_output():
    pipeline = StereoTablePlanePipeline(_table_config(), _cloud_pipeline())
    depth = np.ones((4, 4), dtype=np.float32)

    def identity_lookup(*_args):
        return np.eye(4, dtype=np.float32)

    warmup = _process(pipeline, depth, Time(sec=1), identity_lookup)
    locked = _process(pipeline, depth, Time(sec=2), identity_lookup)

    assert warmup.marker is None
    assert warmup.coeffs is None
    assert locked.estimate_found is True
    assert locked.just_locked is True
    assert locked.marker is not None
    assert locked.marker.markers[0].action == Marker.ADD
    assert locked.marker.markers[0].header.frame_id == 'world'
    assert locked.coeffs is not None
    np.testing.assert_allclose(locked.coeffs.data, [0.0, 0.0, 1.0, -1.0], atol=1.0e-5)

    def unexpected_lookup(*_args):
        raise AssertionError('locked output must not query TF again')

    repeated = _process(
        pipeline,
        np.full((4, 4), np.nan, dtype=np.float32),
        Time(sec=3),
        unexpected_lookup,
    )

    assert repeated.just_locked is False
    assert repeated.marker is not None
    assert repeated.coeffs is not None
    np.testing.assert_allclose(repeated.coeffs.data, locked.coeffs.data)


def test_table_plane_pipeline_clears_marker_when_tf_is_missing():
    pipeline = StereoTablePlanePipeline(_table_config(), _cloud_pipeline())
    depth = np.ones((4, 4), dtype=np.float32)
    _process(
        pipeline,
        depth,
        Time(sec=1),
        lambda *_args: np.eye(4, dtype=np.float32),
    )

    result = _process(pipeline, depth, Time(sec=2), lambda *_args: None)

    assert result.marker is not None
    assert result.marker.markers[0].action == Marker.DELETE
    assert result.marker.markers[0].header.frame_id == 'world'
    assert result.coeffs is None
    assert result.estimate_found is False


def test_table_plane_pipeline_reuses_prepared_source_projection():
    cloud_pipeline = _cloud_pipeline()
    pipeline = StereoTablePlanePipeline(_table_config(), cloud_pipeline)

    def identity_lookup(*_args):
        return np.eye(4, dtype=np.float32)

    _process(pipeline, np.ones((4, 4), dtype=np.float32), Time(sec=1), identity_lookup)
    xs, ys = np.meshgrid(np.linspace(-0.1, 0.1, 4), np.linspace(-0.1, 0.1, 4))
    projection = SourceDepthProjection(
        np.column_stack((xs.ravel(), ys.ravel(), np.ones(xs.size))).astype(np.float32)
    )

    def unexpected_projection(*_args):
        raise AssertionError('prepared table projection must be reused')

    cloud_pipeline.project_depth = unexpected_projection  # type: ignore[method-assign]
    result = _process(
        pipeline,
        np.full((4, 4), np.nan, dtype=np.float32),
        Time(sec=2),
        identity_lookup,
        projection=projection,
    )

    assert result.estimate_found is True
