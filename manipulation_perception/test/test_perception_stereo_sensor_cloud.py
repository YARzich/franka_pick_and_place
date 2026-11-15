"""
Test sensor-cloud preparation before TF and table filtering.

Responsibility: Verify stamped projection, ROI handling, downsampling, and invalid-input reports.
"""

from pathlib import Path
import sys
from types import SimpleNamespace

from builtin_interfaces.msg import Time
import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.common.stereo_cloud_pipeline import (  # noqa: E402,I100,I101,I201
    CloudPipelineFailure,
    StereoCloudPipeline,
)
from manipulation_perception.common.stereo_cloud_output_path import (  # noqa: E402,I100,I101,I201
    StereoCloudOutputPath,
)
from manipulation_perception.common.stereo_config import (  # noqa: E402,I100,I101,I201
    ClusterMarkerConfig,
    WorkspaceConfig,
)
from manipulation_perception.common.stereo_depth_coordinator import (  # noqa: E402,I100,I101,I201
    StereoDepthCoordinator,
)
from manipulation_perception.common.stereo_sensor_cloud import (  # noqa: E402,I100,I101,I201
    build_sensor_frame_cloud,
)


def _cloud_pipeline() -> StereoCloudPipeline:
    workspace = WorkspaceConfig.normalized(
        workspace_frame='world',
        robot_base_frame='panda_link0',
        point_cloud_frame='world',
        robot_reach_m=2.0,
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


def test_octomap_cloud_keeps_optical_frame_stamp_and_sensor_coordinates():
    stamp = Time(sec=7, nanosec=8)
    lookup_calls = []

    def lookup(target_frame, source_frame, message_stamp):
        lookup_calls.append((target_frame, source_frame, message_stamp.sec))
        transform = np.eye(4, dtype=np.float32)
        transform[0, 3] = 10.0
        return transform

    # fx=fy=1000 keeps this 3x3 patch inside one denoise voxel.
    result = build_sensor_frame_cloud(
        np.ones((3, 3), dtype=np.float32),
        intrinsics=(1000.0, 1000.0, 0.0, 0.0),
        source_frame='table_stereo_left_optical_frame',
        stamp=stamp,
        minimum_points=1,
        cloud_pipeline=_cloud_pipeline(),
        transform_lookup=lookup,
    )

    assert result.failure is None
    assert result.point_cloud is not None
    assert result.point_cloud.header.frame_id == 'table_stereo_left_optical_frame'
    assert result.point_cloud.header.stamp == stamp
    assert result.point_cloud.width == 9
    xyz = np.frombuffer(result.point_cloud.data, dtype=np.float32).reshape(-1, 4)[:, :3]
    np.testing.assert_allclose(xyz[0], [0.0, 0.0, 1.0])
    assert lookup_calls == [
        ('world', 'table_stereo_left_optical_frame', 7),
        ('world', 'panda_link0', 7),
    ]


def test_octomap_cloud_drops_isolated_flying_pixels_but_keeps_surfaces():
    """A lone airborne stereo artifact must not become an occupied voxel."""
    depth = np.full((3, 4), np.nan, dtype=np.float32)
    depth[:, :3] = 1.0
    # One valid but isolated pixel far from the dense 3x3 surface patch.
    depth[1, 3] = 2.0

    result = build_sensor_frame_cloud(
        depth,
        intrinsics=(1000.0, 1000.0, 0.0, 0.0),
        source_frame='table_stereo_left_optical_frame',
        stamp=Time(sec=1),
        minimum_points=1,
        cloud_pipeline=_cloud_pipeline(),
        transform_lookup=lambda *_args: np.eye(4, dtype=np.float32),
    )

    assert result.failure is None
    assert result.point_cloud is not None
    assert result.point_cloud.width == 9
    xyz = np.frombuffer(result.point_cloud.data, dtype=np.float32).reshape(-1, 4)[:, :3]
    assert not np.any(xyz[:, 2] > 1.5)


def test_octomap_cloud_rejects_empty_depth_before_tf_lookup():
    lookup_calls = []
    result = build_sensor_frame_cloud(
        np.full((2, 2), np.nan, dtype=np.float32),
        intrinsics=(1.0, 1.0, 0.0, 0.0),
        source_frame='table_stereo_left_optical_frame',
        stamp=Time(sec=1),
        minimum_points=1,
        cloud_pipeline=_cloud_pipeline(),
        transform_lookup=lambda *args: lookup_calls.append(args),
    )

    assert result.point_cloud is None
    assert result.failure == CloudPipelineFailure.INSUFFICIENT_POINTS
    assert result.point_count == 0
    assert lookup_calls == []


def test_octomap_cloud_applies_minimum_after_workspace_crop():
    def lookup(_target_frame, source_frame, _stamp):
        transform = np.eye(4, dtype=np.float32)
        if source_frame == 'camera':
            transform[0, 3] = 10.0
        return transform

    result = build_sensor_frame_cloud(
        np.ones((2, 2), dtype=np.float32),
        intrinsics=(1.0, 1.0, 0.0, 0.0),
        source_frame='camera',
        stamp=Time(sec=1),
        minimum_points=1,
        cloud_pipeline=_cloud_pipeline(),
        transform_lookup=lookup,
    )

    assert result.point_cloud is None
    assert result.failure == CloudPipelineFailure.INSUFFICIENT_POINTS
    assert result.point_count == 0


def test_cloud_output_path_reports_missing_sensor_transform():
    outputs = SimpleNamespace(
        cluster_markers=None,
        publish_octomap_cloud=lambda _cloud: pytest.fail('unexpected publication'),
    )
    path = StereoCloudOutputPath(
        cloud_pipeline=_cloud_pipeline(),
        outputs=outputs,
        transform_lookup=lambda _target, _source, _stamp: None,
        minimum_sensor_points=1,
        logger=SimpleNamespace(warn=lambda *_args, **_kwargs: None),
        context_ok=lambda: True,
        frame_index=lambda: 1,
    )

    published = path.publish_sensor_frame_cloud(
        np.ones((1, 1), dtype=np.float32),
        SimpleNamespace(frame_id='camera', stamp=Time(sec=1)),
        (1.0, 1.0, 0.0, 0.0),
    )

    assert published is False


def test_cloud_output_path_does_not_report_empty_public_cloud_as_healthy():
    published_clouds = []
    outputs = SimpleNamespace(
        cluster_markers=None,
        publish_cloud=lambda cloud, _markers: published_clouds.append(cloud),
    )
    path = StereoCloudOutputPath(
        cloud_pipeline=_cloud_pipeline(),
        outputs=outputs,
        transform_lookup=lambda *_args: np.eye(4, dtype=np.float32),
        minimum_sensor_points=1,
        logger=SimpleNamespace(warn=lambda *_args, **_kwargs: None),
        context_ok=lambda: True,
        frame_index=lambda: 1,
    )

    successful = path.publish_public_cloud(
        np.full((2, 2), np.nan, dtype=np.float32),
        SimpleNamespace(frame_id='camera', stamp=Time(sec=1)),
        (1.0, 1.0, 0.0, 0.0),
    )

    assert successful is False
    assert len(published_clouds) == 1
    assert published_clouds[0].width == 0


class _CloudOutputPath:
    def __init__(self, calls, *, sensor_published=True, public_published=True):
        self.calls = calls
        self.sensor_depth = None
        self.public_depth = None
        self.sensor_published = sensor_published
        self.public_published = public_published
        self.projection = SimpleNamespace(points_source=np.ones((1, 3), dtype=np.float32))
        self.projection_intrinsics = None
        self.sensor_projection = None
        self.public_projection = None

    def prepare_projection(self, _depth, intrinsics):
        self.calls.append('projection')
        self.projection_intrinsics = intrinsics
        return self.projection

    def publish_sensor_frame_cloud(self, depth, _header, _intrinsics, *, projection=None):
        self.calls.append('sensor_cloud')
        self.sensor_depth = depth.copy()
        self.sensor_projection = projection
        return self.sensor_published

    def publish_public_cloud(self, depth, _header, _intrinsics, *, projection=None):
        self.calls.append('public_cloud')
        self.public_depth = depth.copy()
        self.public_projection = projection
        return self.public_published


def test_depth_coordinator_builds_octomap_cloud_before_detection_roi():
    calls = []
    cloud_path = _CloudOutputPath(calls)
    coordinator = object.__new__(StereoDepthCoordinator)
    coordinator._intrinsics = (1.0, 1.0, 0.0, 0.0)
    coordinator._cloud_output_path = cloud_path
    coordinator._publish_table_plane = (
        lambda _depth, _header, _intrinsics, _projection: calls.append('table')
    )

    def apply_roi(_depth, _header):
        calls.append('roi')
        return np.full((1, 1), np.nan, dtype=np.float32), False

    coordinator._apply_roi = apply_roi
    coordinator._bridge = SimpleNamespace(
        cv2_to_imgmsg=lambda _depth, encoding: SimpleNamespace(
            header=None,
            encoding=encoding,
        )
    )
    coordinator._outputs = SimpleNamespace(
        publish_depth=lambda _message: calls.append('depth')
    )
    output_status = []

    def record_output(_header, successful):
        calls.append('output')
        output_status.append(successful)

    coordinator._on_output = record_output

    source_depth = np.ones((1, 1), dtype=np.float32)
    sample_intrinsics = (240.0, 220.0, 0.5, 0.5)
    coordinator._publish_depth(
        source_depth,
        SimpleNamespace(frame_id='camera', stamp=Time(sec=1)),
        sample_intrinsics,
    )

    assert calls == [
        'projection',
        'table',
        'sensor_cloud',
        'roi',
        'depth',
        'public_cloud',
        'output',
    ]
    assert output_status == [False]
    np.testing.assert_allclose(cloud_path.sensor_depth, source_depth)
    assert np.isnan(cloud_path.public_depth).all()
    assert cloud_path.sensor_projection is cloud_path.projection
    assert cloud_path.public_projection is None
    assert cloud_path.projection_intrinsics == sample_intrinsics


def test_depth_coordinator_marks_output_unsuccessful_when_a_cloud_is_missing():
    calls = []
    coordinator = object.__new__(StereoDepthCoordinator)
    coordinator._intrinsics = (1.0, 1.0, 0.0, 0.0)
    coordinator._cloud_output_path = _CloudOutputPath(
        calls,
        sensor_published=False,
        public_published=True,
    )
    coordinator._publish_table_plane = lambda _depth, _header, _intrinsics, _projection: None
    coordinator._apply_roi = lambda depth, _header: (depth, True)
    coordinator._bridge = SimpleNamespace(
        cv2_to_imgmsg=lambda _depth, encoding: SimpleNamespace(
            header=None,
            encoding=encoding,
        )
    )
    coordinator._outputs = SimpleNamespace(publish_depth=lambda _message: None)
    output_status = []
    coordinator._on_output = lambda _header, successful: output_status.append(successful)

    coordinator._publish_depth(
        np.ones((1, 1), dtype=np.float32),
        SimpleNamespace(frame_id='camera', stamp=Time(sec=1)),
        (1.0, 1.0, 0.0, 0.0),
    )

    assert output_status == [False]


def test_depth_coordinator_reuses_projection_when_roi_returns_source_depth():
    calls = []
    cloud_path = _CloudOutputPath(calls)
    coordinator = object.__new__(StereoDepthCoordinator)
    coordinator._intrinsics = (1.0, 1.0, 0.0, 0.0)
    coordinator._cloud_output_path = cloud_path
    coordinator._publish_table_plane = lambda _depth, _header, _intrinsics, _projection: None
    coordinator._apply_roi = lambda depth, _header: (depth, True)
    coordinator._bridge = SimpleNamespace(
        cv2_to_imgmsg=lambda _depth, encoding: SimpleNamespace(
            header=None,
            encoding=encoding,
        )
    )
    coordinator._outputs = SimpleNamespace(publish_depth=lambda _message: None)
    output_status = []
    coordinator._on_output = lambda _header, successful: output_status.append(successful)

    coordinator._publish_depth(
        np.ones((1, 1), dtype=np.float32),
        SimpleNamespace(frame_id='camera', stamp=Time(sec=1)),
        (1.0, 1.0, 0.0, 0.0),
    )

    assert calls.count('projection') == 1
    assert cloud_path.sensor_projection is cloud_path.projection
    assert cloud_path.public_projection is cloud_path.projection
    assert output_status == [True]
