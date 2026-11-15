"""
Test stereo configuration and TF lookup policy.

Responsibility: Preserve backend-neutral parameter normalization, fail-fast
scenario requirements, and explicit stamped/latest TF behavior.
"""

from types import SimpleNamespace

import numpy as np
import pytest
from std_msgs.msg import Header
from tf2_ros import TransformException

from manipulation_perception.common.stereo_config import (
    normalize_enhancer_tag,
    StereoSgbmConfig,
)
from manipulation_perception.common.stereo_node_parameters import (
    declare_stereo_node_parameters,
    read_stereo_node_parameters,
    validate_stereo_node_parameters,
)
from manipulation_perception.common.stereo_transform_provider import StereoTransformProvider


class _FakeParameterNode:
    def __init__(self, overrides: dict | None = None) -> None:
        self.values = dict(overrides or {})
        self.declared: list[tuple[str, object]] = []

    def declare_parameter(self, name: str, default_value: object) -> None:
        self.declared.append((name, default_value))
        self.values.setdefault(name, default_value)

    def get_parameter(self, name: str) -> SimpleNamespace:
        return SimpleNamespace(value=self.values[name])


def _valid_external_stereo_overrides(**overrides) -> dict[str, object]:
    values = {
        'use_external_disparity': True,
        'external_disparity_topic': '/stereo/disparity',
        'left_camera_info_topic': '/stereo/camera_info',
        'disparity_topic': '/stereo/disparity_filtered',
        'depth_topic': '/stereo/depth',
        'point_cloud_topic': '/stereo/points',
        'octomap_point_cloud_topic': '/stereo/octomap_points',
        'point_cloud_frame': 'camera_optical_frame',
        'workspace_frame': 'world',
        'robot_base_frame': 'base_link',
        'table_plane_coeffs_topic': '/stereo/table_plane',
        'baseline_m': 0.06,
        'robot_reach_m': 1.0,
        'publish_table_plane_marker': False,
        'publish_cluster_markers': False,
    }
    values.update(overrides)
    return values


def _read_and_validate_stereo_parameters(**overrides):
    node = _FakeParameterNode(_valid_external_stereo_overrides(**overrides))
    declare_stereo_node_parameters(node)
    parameters = read_stereo_node_parameters(node)
    validate_stereo_node_parameters(parameters)
    return parameters


class _FakeLogger:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def warn(self, message: str, **_kwargs) -> None:
        self.warnings.append(message)


def _header(sec=2, nanosec=3, frame_id='table_stereo_left_optical_frame'):
    header = Header()
    header.stamp.sec = sec
    header.stamp.nanosec = nanosec
    header.frame_id = frame_id
    return header


def test_stereo_config_normalizes_opencv_constraints():
    config = StereoSgbmConfig.normalized(
        min_disparity=0,
        num_disparities=17,
        block_size=2,
        p1=0,
        p2=0,
        pre_filter_cap=63,
        uniqueness_ratio=5,
        speckle_window_size=80,
        speckle_range=2,
        disp12_max_diff=1,
    )

    assert config.num_disparities == 32
    assert config.block_size == 3
    assert config.p1 == 8 * 3 * 3
    assert config.p2 == 32 * 3 * 3

    assert normalize_enhancer_tag('ecc') == 'ecc'
    assert normalize_enhancer_tag('unknown') == 'none'


def test_stereo_node_parameters_keep_algorithm_defaults_and_normalize():
    node = _FakeParameterNode(
        {
            'enhancer_tag': 'unknown',
            'num_disparities': 17,
            'block_size': 2,
            'point_cloud_stride': 0,
            'octomap_min_points': 0,
            'run_every_n_frames': 0,
            'min_depth_m': -1.0,
            'max_depth_m': 0.0,
            'sync_queue_size': 1,
            'sync_slop_sec': -0.1,
            'robot_reach_m': 0.0,
            'workspace_extra_ratio': -1.0,
            'cluster_min_points': 1,
            'cluster_max_points': 5,
        }
    )

    declare_stereo_node_parameters(node)
    params = read_stereo_node_parameters(node)

    assert ('left_image_topic', '') in node.declared
    assert ('left_image_frame_id', '') in node.declared
    assert ('right_image_frame_id', '') in node.declared
    assert ('local_stereo_frame_id', '') in node.declared
    assert ('octomap_point_cloud_topic', '') in node.declared
    assert ('octomap_min_points', 32) in node.declared
    assert ('detection_bbox_stamped_topic', '') in node.declared
    assert ('allow_latest_tf_fallback', False) in node.declared
    assert params.left_image_topic == ''
    assert params.local_stereo_frame_id == ''
    assert params.octomap_point_cloud_topic == ''
    assert not hasattr(params, 'detection_bbox_topic')
    assert params.detection_bbox_stamped_topic == ''
    assert params.allow_latest_tf_fallback is False
    assert params.enhancer_tag_raw == 'unknown'
    assert params.enhancer_tag == 'none'
    assert params.has_unknown_enhancer_tag is True
    assert params.stereo_config.num_disparities == 32
    assert params.stereo_config.block_size == 3
    assert params.point_cloud_stride == 1
    assert params.octomap_min_points == 1
    assert params.run_every_n_frames == 1
    assert params.min_depth_m == 0.0
    assert params.max_depth_m == float('inf')
    assert params.sync_queue_size == 2
    assert params.sync_slop_sec == 0.0
    assert params.workspace_config.workspace_radius_m == 0.0
    assert params.cluster_marker_config.min_points == 10
    assert params.cluster_marker_config.max_points == 10


def test_stereo_node_parameters_require_scenario_graph_and_geometry():
    node = _FakeParameterNode()
    declare_stereo_node_parameters(node)

    with pytest.raises(ValueError, match='required parameters'):
        validate_stereo_node_parameters(read_stereo_node_parameters(node))


@pytest.mark.parametrize(
    'parameter_name',
    (
        'detection_bbox_timeout_sec',
        'baseline_m',
        'min_valid_disparity_px',
        'min_depth_m',
        'max_depth_m',
        'sync_slop_sec',
        'robot_reach_m',
        'workspace_extra_ratio',
        'table_plane_init_delay_sec',
        'plane_distance_threshold_m',
        'plane_normal_max_tilt_deg',
        'plane_marker_thickness_m',
        'plane_marker_alpha',
        'cluster_voxel_size_m',
        'cluster_min_diagonal_m',
        'cluster_max_diagonal_m',
        'cluster_marker_min_edge_m',
        'cluster_marker_alpha',
        'ecc_eps',
    ),
)
@pytest.mark.parametrize('invalid_value', (float('nan'), float('inf'), float('-inf')))
def test_stereo_node_parameters_reject_non_finite_float_inputs(
    parameter_name,
    invalid_value,
):
    node = _FakeParameterNode(
        _valid_external_stereo_overrides(**{parameter_name: invalid_value})
    )
    declare_stereo_node_parameters(node)

    with pytest.raises(ValueError, match=parameter_name):
        read_stereo_node_parameters(node)


@pytest.mark.parametrize(
    ('parameter_name', 'invalid_value'),
    (
        ('detection_bbox_timeout_sec', -0.1),
        ('detection_bbox_timeout_sec', 0.0),
        ('min_valid_disparity_px', -0.1),
    ),
)
def test_stereo_node_parameters_reject_invalid_runtime_thresholds(
    parameter_name,
    invalid_value,
):
    with pytest.raises(ValueError, match=parameter_name):
        _read_and_validate_stereo_parameters(**{parameter_name: invalid_value})


@pytest.mark.parametrize('max_depth_m', (0.0, -1.0))
def test_stereo_node_parameters_keep_non_positive_max_depth_disable_semantics(max_depth_m):
    parameters = _read_and_validate_stereo_parameters(
        min_depth_m=-1.0,
        max_depth_m=max_depth_m,
    )

    assert parameters.min_depth_m == 0.0
    assert parameters.max_depth_m == float('inf')


def test_stereo_node_parameters_reject_contradictory_depth_bounds():
    with pytest.raises(ValueError, match='max_depth_m must be greater than min_depth_m'):
        _read_and_validate_stereo_parameters(min_depth_m=0.5, max_depth_m=0.5)


@pytest.mark.parametrize('plane_normal_max_tilt_deg', (-0.1, 90.0, 180.0))
def test_stereo_node_parameters_reject_invalid_plane_tilt(plane_normal_max_tilt_deg):
    with pytest.raises(ValueError, match='plane_normal_max_tilt_deg'):
        _read_and_validate_stereo_parameters(
            plane_normal_max_tilt_deg=plane_normal_max_tilt_deg
        )


def _transform(translation=(0.0, 0.0, 0.0)):
    return SimpleNamespace(
        transform=SimpleNamespace(
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            translation=SimpleNamespace(
                x=translation[0],
                y=translation[1],
                z=translation[2],
            ),
        )
    )


class _StampedMissingTfBuffer:
    def __init__(self) -> None:
        self.calls = []

    def lookup_transform(self, target_frame, source_frame, stamp, *, timeout):
        self.calls.append((target_frame, source_frame, stamp.nanoseconds, timeout.nanoseconds))
        if stamp.nanoseconds != 0:
            raise TransformException('stamped TF missing')
        return _transform((1.0, 2.0, 3.0))


def _tf_lookup_test_provider(*, allow_latest: bool):
    provider = object.__new__(StereoTransformProvider)
    logger = _FakeLogger()
    provider._buffer = _StampedMissingTfBuffer()
    provider._allow_latest_fallback = allow_latest
    provider._warned_about_latest_fallback = False
    provider._warned_about_tf = False
    provider._frame_index = lambda: 1
    provider._logger = logger
    return provider, logger


def test_stereo_tf_lookup_is_strict_by_default():
    provider, logger = _tf_lookup_test_provider(allow_latest=False)

    matrix = provider.lookup('world', 'camera', _header().stamp)

    assert matrix is None
    assert len(provider._buffer.calls) == 1
    assert 'allow_latest_tf_fallback=false' in logger.warnings[0]


def test_stereo_tf_lookup_uses_explicit_static_camera_fallback_once():
    provider, logger = _tf_lookup_test_provider(allow_latest=True)

    first_matrix = provider.lookup('world', 'camera', _header().stamp)
    second_matrix = provider.lookup('world', 'camera', _header().stamp)

    np.testing.assert_allclose(first_matrix[:3, 3], (1.0, 2.0, 3.0))
    np.testing.assert_allclose(second_matrix, first_matrix)
    assert len(provider._buffer.calls) == 4
    assert len(logger.warnings) == 1
    assert 'latest TF' in logger.warnings[0]
