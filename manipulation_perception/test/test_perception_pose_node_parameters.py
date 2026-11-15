"""
Test typed pose-node parameter loading and validation.

Responsibility: Verify injected contracts, depth semantics, backend checks, and warnings.
"""

from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.pose.pose_common_parameters import (  # noqa: E402,I100,I101,I201
    CommonPoseParameters,
    read_common_pose_parameters,
)
from manipulation_perception.pose.pose_gigapose_parameters import (  # noqa: E402,I100,I101,I201
    GigaPoseParameters,
    read_gigapose_parameters,
)
from manipulation_perception.pose.pose_megapose_parameters import (  # noqa: E402,I100,I101,I201
    MegaPoseParameters,
    read_megapose_parameters,
)
from manipulation_perception.pose.pose_node_parameters import (  # noqa: E402,I100,I101,I201
    declare_pose_node_parameters,
    pose_node_parameter_warnings,
    read_pose_node_parameters,
    validate_pose_node_parameters,
)


class _FakeParameterNode:
    def __init__(self, overrides: dict | None = None) -> None:
        self.values = dict(overrides or {})
        self.declared: list[tuple[str, object]] = []

    def declare_parameter(self, name: str, default_value: object) -> None:
        self.declared.append((name, default_value))
        self.values.setdefault(name, default_value)

    def get_parameter(self, name: str) -> SimpleNamespace:
        return SimpleNamespace(value=self.values[name])


def _required_pose_overrides(**updates) -> dict[str, object]:
    values: dict[str, object] = {
        'image_topic': '/camera/image',
        'camera_info_topic': '/camera/camera_info',
        'depth_topic': '/camera/depth',
        'detection_bbox_stamped_topic': '/detections/2d',
        'output_detections_topic': '/detections/3d',
        'output_frame_id': 'camera_optical_frame',
        'megapose_python': sys.executable,
        'megapose_data_dir': '/tmp/megapose-data',
        'object_label': 'test_object',
        'mesh_file_path': '/tmp/test_object.obj',
        'detection_mask_topic': '/detections/mask',
        'gigapose_python': sys.executable,
        'gigapose_repo_dir': '/tmp/gigapose',
        'gigapose_data_dir': '/tmp/gigapose-data',
        'gigapose_checkpoint_path': '/tmp/gigapose.ckpt',
        'gigapose_dataset_name': 'test_dataset',
        'gigapose_object_id': 1,
    }
    values.update(updates)
    return values


def test_pose_node_parameters_keep_algorithm_defaults_and_depth_semantics():
    node = _FakeParameterNode(
        _required_pose_overrides(
            depth_mode='auto',
            use_depth=True,
            score=2.0,
            run_every_n_frames=0,
            process_timeout_sec=0.1,
            bbox_timeout_sec=0.0,
            bsz_images=0,
            bsz_objects=0,
            renderer_workers=0,
        )
    )

    declare_pose_node_parameters(node)
    params = read_pose_node_parameters(node)

    assert ('image_topic', '') in node.declared
    assert all(name != 'detection_bbox_topic' for name, _ in node.declared)
    assert params.detection_bbox_stamped_topic == '/detections/2d'
    assert params.output_detections_topic == '/detections/3d'
    assert params.output_frame_id == 'camera_optical_frame'
    assert params.depth_mode == 'on'
    assert params.score == 1.0
    assert params.run_every_n_frames == 1
    assert params.process_timeout_sec == 1.0
    assert params.bbox_timeout_sec == 0.05
    assert params.bsz_images == 1
    assert params.bsz_objects == 1
    assert params.renderer_workers == 1
    assert params.example_name == 'test_object'
    assert params.uses_gigapose_coarse is False
    assert params.is_depth_disabled is False
    assert params.gigapose_checkpoint_path == Path('/tmp/gigapose.ckpt')
    assert params.gigapose_min_matching_score == 0.0


def test_pose_parameter_groups_can_be_read_as_independent_typed_configs():
    node = _FakeParameterNode(_required_pose_overrides())
    declare_pose_node_parameters(node)

    params = read_pose_node_parameters(node)
    common = read_common_pose_parameters(node)
    megapose = read_megapose_parameters(node)
    gigapose = read_gigapose_parameters(node)

    assert isinstance(common, CommonPoseParameters)
    assert isinstance(megapose, MegaPoseParameters)
    assert isinstance(gigapose, GigaPoseParameters)
    assert common.image_topic == '/camera/image'
    assert megapose.object_label == 'test_object'
    assert gigapose.gigapose_dataset_name == 'test_dataset'
    assert params.common == common
    assert params.megapose == megapose
    assert params.gigapose == gigapose


def test_empty_gigapose_checkpoint_uses_data_directory_default():
    node = _FakeParameterNode(
        _required_pose_overrides(
            gigapose_data_dir='/opt/gigapose-data',
            gigapose_checkpoint_path='',
        )
    )
    declare_pose_node_parameters(node)

    params = read_pose_node_parameters(node)

    assert params.gigapose_checkpoint_path == Path(
        '/opt/gigapose-data/pretrained/gigaPose_v1.ckpt'
    )


def test_pose_node_parameter_validation_reports_invalid_modes_without_file_checks():
    invalid_cases = [
        ({'mesh_units': 'cm'}, 'mesh_units'),
        ({'depth_mode': 'bad'}, 'depth_mode'),
        ({'coarse_estimator': 'bad'}, 'coarse_estimator'),
        ({'object_label': '  '}, 'object_label'),
        ({'object_label': '../outside'}, 'object_label'),
        ({'coarse_estimator': 'gigapose', 'gigapose_dataset_name': '../bad'}, 'dataset_name'),
        ({'coarse_estimator': 'gigapose', 'gigapose_object_id': 0}, 'object id'),
        (
            {
                'coarse_estimator': 'gigapose',
                'gigapose_min_matching_score': float('nan'),
            },
            'min_matching_score',
        ),
        (
            {
                'coarse_estimator': 'gigapose',
                'gigapose_min_matching_score': -0.01,
            },
            'min_matching_score',
        ),
        (
            {
                'coarse_estimator': 'gigapose',
                'gigapose_min_matching_score': 1.01,
            },
            'min_matching_score',
        ),
    ]

    for overrides, match in invalid_cases:
        node = _FakeParameterNode(_required_pose_overrides(**overrides))
        declare_pose_node_parameters(node)
        params = read_pose_node_parameters(node)

        with pytest.raises(ValueError, match=match):
            validate_pose_node_parameters(params, check_files=False)


def test_pose_node_parameter_validation_checks_gigapose_files(tmp_path: Path):
    mesh = tmp_path / 'object.obj'
    mesh.write_text('v 0 0 0\nv 1 1 1\n')
    repo = tmp_path / 'gigaPose'
    (repo / 'configs').mkdir(parents=True)
    (repo / 'configs' / 'test.yaml').write_text('ok: true\n')
    dinov2 = tmp_path / 'dinov2'
    dinov2.mkdir()
    (dinov2 / 'hubconf.py').write_text('# fake\n')
    checkpoint = tmp_path / 'gigaPose.ckpt'
    checkpoint.write_text('checkpoint')
    megapose_data_dir = tmp_path / 'megapose_data'
    megapose_data_dir.mkdir()
    gigapose_data_dir = tmp_path / 'gigapose_data'
    gigapose_data_dir.mkdir()

    node = _FakeParameterNode(
        _required_pose_overrides(
            coarse_estimator='gigapose',
            megapose_data_dir=str(megapose_data_dir),
            mesh_file_path=str(mesh),
            gigapose_repo_dir=str(repo),
            gigapose_dinov2_repo_dir=str(dinov2),
            gigapose_checkpoint_path=str(checkpoint),
            gigapose_data_dir=str(gigapose_data_dir),
            gigapose_python=sys.executable,
        )
    )

    declare_pose_node_parameters(node)
    params = read_pose_node_parameters(node)
    validate_pose_node_parameters(params)

    node.values['gigapose_object_id'] = 0
    params = read_pose_node_parameters(node)
    with pytest.raises(ValueError, match='object id'):
        validate_pose_node_parameters(params)


def test_pose_node_parameter_warnings_preserve_depth_disabled_message():
    node = _FakeParameterNode(
        _required_pose_overrides(
            depth_mode='off',
            megapose_model='megapose-1.0-RGBD',
        )
    )

    declare_pose_node_parameters(node)
    params = read_pose_node_parameters(node)

    assert pose_node_parameter_warnings(params)
