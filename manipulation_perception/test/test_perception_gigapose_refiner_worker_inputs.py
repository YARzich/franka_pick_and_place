"""
Test GigaPose refiner request parsing and pose-estimate construction.

Responsibility: Verify defaults, cache identity, labels, scores, timing, and shape validation.
"""

from pathlib import Path
import sys

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.pose.pose_gigapose_refiner_worker_inputs import (  # noqa: E402
    gigapose_refiner_backend_cache_key,
    make_gigapose_refiner_pose_estimates,
    parse_gigapose_refiner_worker_request,
)


class FakeDataFrame(dict):
    pass


class FakePandas:
    DataFrame = FakeDataFrame


class FakeTensor:
    def __init__(self, values) -> None:
        self.array = np.asarray(values, dtype=np.float32)
        self.ndim = self.array.ndim
        self.shape = self.array.shape
        self.cuda_called = False

    def cuda(self):
        self.cuda_called = True
        return self


class FakeTorch:
    float32 = np.float32

    @staticmethod
    def as_tensor(values, dtype):
        return FakeTensor(np.asarray(values, dtype=dtype))


class FakeTensorCollection:
    def __init__(self, *, infos, poses) -> None:
        self.infos = infos
        self.poses = poses


def _request(tmp_path: Path, **overrides) -> dict:
    request = {
        'id': 8,
        'image_path': str(tmp_path / 'rgb.png'),
        'K': [[100.0, 0.0, 50.0], [0.0, 100.0, 40.0], [0.0, 0.0, 1.0]],
        'repo_dir': str(tmp_path / 'repo'),
        'root_dir': str(tmp_path / 'data'),
        'dataset_name': 'franka_cube',
        'object_id': 3,
        'output_label': '',
        'megapose_models_root': str(tmp_path / 'megapose_models'),
        'bsz_images': 0,
        'bsz_objects': 0,
        'n_iterations': 0,
        'num_workers': 0,
        'coarse_estimates': {'poses': np.zeros((1, 4, 4)).tolist()},
    }
    request.update(overrides)
    return request


def test_parse_refiner_request_clamps_defaults_and_label(tmp_path: Path):
    request = parse_gigapose_refiner_worker_request(_request(tmp_path))

    assert request.request_id == 8
    assert request.output_label == 'obj_000003'
    assert request.bsz_images == 1
    assert request.bsz_objects == 1
    assert request.n_iterations == 1
    assert request.num_workers == 1
    assert request.camera_matrix.shape == (3, 3)
    assert request.refiner_models_dir == (
        tmp_path / 'data' / 'ros_runtime' / 'refiner_models' / 'franka_cube'
    )


def test_refiner_cache_key_documents_current_runtime_fields(tmp_path: Path):
    first = parse_gigapose_refiner_worker_request(
        _request(tmp_path, output_label='cube_a', num_workers=1)
    )
    second = parse_gigapose_refiner_worker_request(
        _request(tmp_path, output_label='cube_b', num_workers=1)
    )
    third = parse_gigapose_refiner_worker_request(_request(tmp_path, n_iterations=2))
    fourth = parse_gigapose_refiner_worker_request(_request(tmp_path, num_workers=9))

    assert gigapose_refiner_backend_cache_key(
        first
    ) == gigapose_refiner_backend_cache_key(second)
    assert gigapose_refiner_backend_cache_key(
        first
    ) != gigapose_refiner_backend_cache_key(third)
    assert gigapose_refiner_backend_cache_key(
        first
    ) != gigapose_refiner_backend_cache_key(fourth)


def test_make_pose_estimates_keeps_scores_and_elapsed_time():
    estimates = make_gigapose_refiner_pose_estimates(
        {
            'poses': np.zeros((2, 4, 4)).tolist(),
            'scores': [0.7, 0.4],
            'elapsed_sec': 0.25,
        },
        object_id=3,
        torch_module=FakeTorch,
        pandas_module=FakePandas,
        tensor_collection_type=FakeTensorCollection,
    )

    assert estimates.poses.cuda_called is True
    assert estimates.infos['label'] == ['obj_000003', 'obj_000003']
    assert estimates.infos['matching_score'] == [0.7, 0.4]
    assert estimates.infos['time'] == [0.25, 0.25]
    assert estimates.infos['hypothesis_id'] == [0, 1]


def test_make_pose_estimates_fills_missing_scores_and_validates_shape():
    estimates = make_gigapose_refiner_pose_estimates(
        {'poses': np.zeros((1, 4, 4)).tolist(), 'scores': [0.7, 0.4]},
        object_id=3,
        torch_module=FakeTorch,
        pandas_module=FakePandas,
        tensor_collection_type=FakeTensorCollection,
    )

    assert estimates.infos['matching_score'] == [1.0]

    with pytest.raises(ValueError, match='\\[N,4,4\\]'):
        make_gigapose_refiner_pose_estimates(
            {'poses': np.zeros((4, 4)).tolist()},
            object_id=3,
            torch_module=FakeTorch,
            pandas_module=FakePandas,
            tensor_collection_type=FakeTensorCollection,
        )
