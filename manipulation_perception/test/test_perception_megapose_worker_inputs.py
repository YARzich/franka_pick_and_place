"""
Test MegaPose worker request parsing and coarse-estimate conversion.

Responsibility: Verify model selection, path precedence, clamping, labels, scores, and shapes.
"""

from pathlib import Path
import sys

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.pose.pose_megapose_worker_inputs import (  # noqa: E402
    build_megapose_coarse_estimates,
    parse_megapose_worker_request,
)


NAMED_MODELS = {'rgb': {'requires_depth': False, 'inference_parameters': {}}}


class FakeDataFrame(dict):
    pass


class FakePandas:
    DataFrame = FakeDataFrame


class FakeTorch:
    float32 = np.float32

    @staticmethod
    def as_tensor(values, dtype):
        return np.asarray(values, dtype=dtype)


class FakeTensorCollection:
    def __init__(self, *, infos, poses) -> None:
        self.infos = infos
        self.poses = poses
        self.cuda_called = False

    def cuda(self):
        self.cuda_called = True
        return self


def _add_instance_id(collection):
    collection.instance_ids_added = True
    return collection


def test_parse_worker_request_prefers_example_dir_and_clamps_batch_values(tmp_path: Path):
    request = parse_megapose_worker_request(
        {
            'id': 7,
            'model': 'rgb',
            'example_name': 'fallback',
            'example_dir': str(tmp_path / 'explicit_example'),
            'object_label': 'cube_5cm',
            'bsz_images': 0,
            'bsz_objects': -2,
            'renderer_workers': 0,
            'debug': False,
        },
        local_data_dir=tmp_path / 'data',
        named_models=NAMED_MODELS,
        env={'MEGAPOSE_WORKER_DEBUG': '1'},
    )

    assert request.request_id == 7
    assert request.debug is True
    assert request.example_dir == tmp_path / 'explicit_example'
    assert request.bsz_images == 1
    assert request.bsz_objects == 1
    assert request.renderer_workers == 1
    assert request.object_label == 'cube_5cm'


def test_parse_worker_request_uses_example_name_when_example_dir_is_missing(tmp_path: Path):
    request = parse_megapose_worker_request(
        {'model': 'rgb', 'example_name': 'franka_motion_cube_5cm'},
        local_data_dir=tmp_path / 'data',
        named_models=NAMED_MODELS,
    )

    assert request.example_dir == tmp_path / 'data' / 'examples' / 'franka_motion_cube_5cm'


def test_parse_worker_request_rejects_unknown_model_and_bad_coarse_payload(tmp_path: Path):
    with pytest.raises(ValueError, match='unknown MegaPose model'):
        parse_megapose_worker_request(
            {'model': 'missing', 'example_name': 'example'},
            local_data_dir=tmp_path,
            named_models=NAMED_MODELS,
        )

    with pytest.raises(ValueError, match='coarse_estimates'):
        parse_megapose_worker_request(
            {'model': 'rgb', 'example_name': 'example', 'coarse_estimates': []},
            local_data_dir=tmp_path,
            named_models=NAMED_MODELS,
        )


def test_build_coarse_estimates_adds_labels_scores_and_instance_ids():
    coarse = build_megapose_coarse_estimates(
        {
            'poses': np.zeros((2, 4, 4)).tolist(),
            'scores': [0.8, 0.6],
            'label': 'cube_from_coarse',
        },
        object_label='cube_5cm',
        torch_module=FakeTorch,
        pandas_module=FakePandas,
        tensor_collection_type=FakeTensorCollection,
        add_instance_id_fn=_add_instance_id,
    )

    assert coarse.cuda_called is True
    assert coarse.instance_ids_added is True
    assert coarse.infos['label'] == ['cube_from_coarse', 'cube_from_coarse']
    assert coarse.infos['coarse_input_score'] == [0.8, 0.6]
    assert coarse.poses.shape == (2, 4, 4)


def test_build_coarse_estimates_uses_object_label_and_ignores_mismatched_scores():
    coarse = build_megapose_coarse_estimates(
        {
            'poses': np.zeros((1, 4, 4)).tolist(),
            'scores': [0.8, 0.6],
        },
        object_label='cube_5cm',
        torch_module=FakeTorch,
        pandas_module=FakePandas,
        tensor_collection_type=FakeTensorCollection,
        add_instance_id_fn=_add_instance_id,
    )

    assert coarse.infos['label'] == ['cube_5cm']
    assert 'coarse_input_score' not in coarse.infos


def test_build_coarse_estimates_validates_shape_and_label():
    with pytest.raises(ValueError, match='\\[N,4,4\\]'):
        build_megapose_coarse_estimates(
            {'poses': np.zeros((4, 4)).tolist()},
            object_label='cube_5cm',
            torch_module=FakeTorch,
            pandas_module=FakePandas,
            tensor_collection_type=FakeTensorCollection,
            add_instance_id_fn=_add_instance_id,
        )

    with pytest.raises(ValueError, match='object label'):
        build_megapose_coarse_estimates(
            {'poses': np.zeros((1, 4, 4)).tolist()},
            object_label='',
            torch_module=FakeTorch,
            pandas_module=FakePandas,
            tensor_collection_type=FakeTensorCollection,
            add_instance_id_fn=_add_instance_id,
        )
