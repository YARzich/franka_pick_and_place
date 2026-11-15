"""
Test stereo geometry configuration, messages, and table/workspace helpers.

Responsibility: Verify ECC normalization, table-plane tracking/detection,
ROS message construction, clustering markers, and workspace projection.
"""

from builtin_interfaces.msg import Time
import numpy as np
from visualization_msgs.msg import Marker

from manipulation_perception.common.stereo_config import (
    ClusterMarkerConfig,
    EccConfig,
    TablePlaneConfig,
    WorkspaceConfig,
)
from manipulation_perception.common.stereo_depth_pipeline import EccAligner
from manipulation_perception.common.stereo_messages import (
    ClusterMarkerBuilder,
    PointCloud2Builder,
    table_plane_coeffs_message,
    TablePlaneMarkerBuilder,
)
from manipulation_perception.common.stereo_table_plane import TablePlaneDetector
from manipulation_perception.common.stereo_workspace import WorkspaceProjector
from manipulation_perception.common.table_plane_tracker import TablePlaneTracker


def test_ecc_config_and_disabled_aligner_are_predictable():
    config = EccConfig.normalized(
        enabled=False,
        max_iterations=0,
        eps=0.0,
        gauss_filter_size=4,
        update_every_n_frames=0,
    )

    assert config.max_iterations == 1
    assert config.eps == 1.0e-8
    assert config.gauss_filter_size == 5
    assert config.update_every_n_frames == 1

    right = np.zeros((2, 3), dtype=np.uint8)
    aligned, error = EccAligner(config).align(np.ones_like(right), right, frame_index=1)

    assert aligned is right
    assert error is None


def test_table_plane_tracker_handles_delay_sampling_and_lock():
    config = TablePlaneConfig.normalized(
        detect_once=True,
        init_delay_sec=1.0,
        init_required_frames=2,
        ransac_iterations=1,
        distance_threshold_m=0.0,
        normal_max_tilt_deg=20.0,
        marker_thickness_m=0.0,
        marker_alpha=2.0,
    )
    tracker = TablePlaneTracker(config, np.array([0.0, 0.0, 1.0], dtype=np.float32))

    assert config.ransac_iterations == 20
    assert config.distance_threshold_m == 1.0e-4
    assert config.marker_thickness_m == 0.001
    assert config.marker_alpha == 1.0
    assert tracker.ready_for_detection(10.0) is False
    assert tracker.ready_for_detection(10.5) is False
    assert tracker.ready_for_detection(11.1) is True

    first = tracker.add_sample((0.0, 1.0, 0.0, 1.0, 0.70), (0.0, 0.0, 1.0, -0.70))
    assert first is None

    second = tracker.add_sample((0.0, 1.2, 0.0, 1.0, 0.74), (0.0, 0.0, -1.0, 0.74))
    assert second is not None
    assert second.locked is True
    assert second.just_locked is True
    np.testing.assert_allclose(second.bounds, (0.0, 1.1, 0.0, 1.0, 0.72))
    np.testing.assert_allclose(second.coeffs[:3], (0.0, 0.0, 1.0))
    assert abs(second.coeffs[3] + 0.72) < 1.0e-6
    assert tracker.locked_output() is not None


def test_point_cloud_and_table_plane_message_builders_preserve_ros_contract():
    stamp = Time(sec=2, nanosec=3)
    cloud = PointCloud2Builder().build(
        np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32),
        stamp,
        'world',
    )

    assert cloud.header.stamp == stamp
    assert cloud.header.frame_id == 'world'
    assert cloud.width == 2
    assert [field.name for field in cloud.fields] == ['x', 'y', 'z', 'rgb']
    assert cloud.point_step == 16

    marker_builder = TablePlaneMarkerBuilder(
        TablePlaneConfig.normalized(
            detect_once=True,
            init_delay_sec=0.0,
            init_required_frames=1,
            ransac_iterations=20,
            distance_threshold_m=0.01,
            normal_max_tilt_deg=20.0,
            marker_thickness_m=0.004,
            marker_alpha=0.35,
        )
    )
    marker_array = marker_builder.build((0.0, 1.0, -0.5, 0.5, 0.75), stamp, 'world')

    assert len(marker_array.markers) == 1
    marker = marker_array.markers[0]
    assert marker.ns == 'table_plane'
    assert marker.type == Marker.CUBE
    assert marker.header.frame_id == 'world'
    assert marker.pose.position.z == 0.75
    assert marker.scale.z == 0.004
    assert marker.color.a == 0.35

    clear = TablePlaneMarkerBuilder.clear(stamp, 'world')
    assert clear.markers[0].action == Marker.DELETE
    assert list(table_plane_coeffs_message((0.0, 0.0, 1.0, -0.75)).data) == [0.0, 0.0, 1.0, -0.75]


def test_cluster_marker_builder_keeps_deleteall_and_filters_by_diagonal():
    stamp = Time(sec=4)
    config = ClusterMarkerConfig.normalized(
        voxel_size_m=0.01,
        min_points=2,
        max_points=10,
        min_diagonal_m=0.01,
        max_diagonal_m=0.20,
        marker_min_edge_m=0.02,
        marker_alpha=0.55,
    )
    builder = ClusterMarkerBuilder(config)
    points = np.array([[0.0, 0.0, 0.0], [0.02, 0.0, 0.0]], dtype=np.float32)

    marker_array = builder.build(points, [np.array([0, 1], dtype=np.int32)], stamp, 'world')

    assert marker_array.markers[0].action == Marker.DELETEALL
    assert len(marker_array.markers) == 2
    marker = marker_array.markers[1]
    assert marker.ns == 'tabletop_clusters'
    assert marker.header.frame_id == 'world'
    assert marker.scale.x == 0.02
    assert marker.scale.y == 0.02
    assert marker.color.a == 0.55


def test_workspace_projector_crops_by_robot_reach_and_transforms_output_frame():
    config = WorkspaceConfig.normalized(
        workspace_frame='world',
        robot_base_frame='panda_link0',
        point_cloud_frame='camera_debug',
        robot_reach_m=0.1,
        workspace_extra_ratio=0.5,
    )
    projector = WorkspaceProjector(config)
    source_to_workspace = np.eye(4, dtype=np.float32)
    base_to_workspace = np.eye(4, dtype=np.float32)
    points_source = np.array(
        [
            [0.00, 0.00, 0.00],
            [0.10, 0.00, 0.00],
            [0.16, 0.00, 0.00],
        ],
        dtype=np.float32,
    )

    workspace_points = projector.crop_source_points_to_workspace(
        points_source,
        source_to_workspace,
        base_to_workspace,
    )

    np.testing.assert_allclose(workspace_points, points_source[:2])
    assert projector.needs_output_transform is True

    workspace_to_output = np.eye(4, dtype=np.float32)
    workspace_to_output[:3, 3] = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    output = projector.project_output(workspace_points, workspace_to_output)

    assert output.frame_id == 'camera_debug'
    np.testing.assert_allclose(output.points, workspace_points + np.array([1.0, 2.0, 3.0]))


def test_table_plane_detector_returns_bounds_and_coeffs_or_failure_reason():
    config = TablePlaneConfig.normalized(
        detect_once=True,
        init_delay_sec=0.0,
        init_required_frames=1,
        ransac_iterations=80,
        distance_threshold_m=0.005,
        normal_max_tilt_deg=20.0,
        marker_thickness_m=0.004,
        marker_alpha=0.35,
    )
    detector = TablePlaneDetector(
        config,
        np.array([0.0, 0.0, 1.0], dtype=np.float32),
        np.random.default_rng(42),
    )

    too_few = detector.detect(np.array([[0.0, 0.0, 0.0]], dtype=np.float32))
    assert too_few.estimate is None
    assert too_few.failure == 'not_enough_points'

    xs, ys = np.meshgrid(np.linspace(-0.2, 0.2, 8), np.linspace(-0.1, 0.1, 6))
    points = np.column_stack(
        [
            xs.ravel(),
            ys.ravel(),
            np.full(xs.size, 0.75, dtype=np.float32),
        ]
    ).astype(np.float32)
    result = detector.detect(points)

    assert result.failure is None
    assert result.estimate is not None
    np.testing.assert_allclose(result.estimate.bounds, (-0.2, 0.2, -0.1, 0.1, 0.75))
    np.testing.assert_allclose(result.estimate.coeffs[:3], (0.0, 0.0, 1.0), atol=1.0e-5)
    assert abs(result.estimate.coeffs[3] + 0.75) < 1.0e-5
