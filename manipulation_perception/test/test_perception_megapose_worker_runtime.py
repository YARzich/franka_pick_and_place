"""
Test MegaPose inference runtime selection, caching, and error reporting.

Responsibility: Verify coarse-estimate paths, depth loading, model reuse, and bounded failures.
"""

from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.pose.pose_megapose_worker_runtime import (  # noqa: E402
    MegaPoseRuntimeBackend,
    MegaPoseRuntimeDependencies,
)
from test_perception_megapose_worker_inputs import (  # noqa: E402,I100,I101,I201
    FakePandas,
    FakeTensorCollection,
    FakeTorch,
)
from test_perception_megapose_worker_outputs import (  # noqa: E402,I100,I101,I201
    FakeObjectData,
    FakeTransform,
)


class FakeCudaObject:
    def __init__(self, value) -> None:
        self.value = value
        self.cuda_called = False

    def cuda(self):
        self.cuda_called = True
        return self


class FakePoseEstimator:
    def __init__(self) -> None:
        self.calls = []

    def run_inference_pipeline(self, observation, *, detections, coarse_estimates, **kwargs):
        self.calls.append(
            {
                'observation': observation,
                'detections': detections,
                'coarse_estimates': coarse_estimates,
                'kwargs': kwargs,
            }
        )
        pose_estimates = SimpleNamespace(
            infos={'label': ['cube_5cm']},
            poses=FakeCudaObject([np.eye(4).tolist()]),
        )
        pose_estimates.poses.cpu = lambda: pose_estimates.poses
        pose_estimates.poses.numpy = lambda: pose_estimates.poses.value
        return pose_estimates, {'timing_str': 'timing-ok'}


class FakeLoadedModel:
    def __init__(self, estimator: FakePoseEstimator) -> None:
        self.estimator = estimator

    def cuda(self):
        return self.estimator


class FakeRuntimeDeps:
    def __init__(self, tmp_path: Path) -> None:
        self.estimators: list[FakePoseEstimator] = []
        self.observation_calls = []
        self.detection_calls = []
        self.load_model_calls = []
        self.deps = MegaPoseRuntimeDependencies(
            pandas=FakePandas,
            torch=FakeTorch,
            local_data_dir=tmp_path / 'megapose_data',
            object_data_type=FakeObjectData,
            add_instance_id=lambda collection: collection,
            transform_type=FakeTransform,
            load_detections=self.load_detections,
            load_observation_tensor=self.load_observation_tensor,
            make_object_dataset=lambda example_dir: {'example_dir': example_dir},
            pandas_tensor_collection_type=FakeTensorCollection,
            named_models={
                'rgb': {'requires_depth': False, 'inference_parameters': {'model_arg': 'rgb'}},
                'rgbd': {'requires_depth': True, 'inference_parameters': {'model_arg': 'rgbd'}},
            },
            load_named_model=self.load_named_model,
        )

    def load_observation_tensor(self, example_dir: Path, *, load_depth: bool):
        self.observation_calls.append((example_dir, load_depth))
        return FakeCudaObject('observation')

    def load_detections(self, example_dir: Path):
        self.detection_calls.append(example_dir)
        return FakeCudaObject('detections')

    def load_named_model(self, model_name, object_dataset, *, n_workers, bsz_images):
        estimator = FakePoseEstimator()
        self.estimators.append(estimator)
        self.load_model_calls.append((model_name, object_dataset, n_workers, bsz_images))
        return FakeLoadedModel(estimator)


def test_runtime_uses_detections_without_coarse_estimates_and_caches_model(tmp_path: Path):
    deps = FakeRuntimeDeps(tmp_path)
    backend = MegaPoseRuntimeBackend(deps.deps)
    request = {
        'id': 1,
        'model': 'rgb',
        'example_dir': str(tmp_path / 'example'),
        'object_label': 'cube_5cm',
        'bsz_images': 2,
        'renderer_workers': 3,
    }

    first = backend.run_request(request)
    second = backend.run_request({**request, 'id': 2})

    assert first['ok'] is True
    assert second['ok'] is True
    assert first['object_data'][0]['label'] == 'cube_5cm'
    assert deps.observation_calls == [(tmp_path / 'example', False), (tmp_path / 'example', False)]
    assert deps.detection_calls == [tmp_path / 'example', tmp_path / 'example']
    assert len(deps.load_model_calls) == 1
    assert deps.load_model_calls[0][2:] == (3, 2)
    assert len(deps.estimators[0].calls) == 2
    assert deps.estimators[0].calls[0]['kwargs'] == {'model_arg': 'rgb'}


def test_runtime_skips_detections_when_coarse_estimates_are_present(tmp_path: Path):
    deps = FakeRuntimeDeps(tmp_path)
    backend = MegaPoseRuntimeBackend(deps.deps)

    response = backend.run_request(
        {
            'id': 1,
            'model': 'rgb',
            'example_dir': str(tmp_path / 'example'),
            'object_label': 'cube_5cm',
            'coarse_estimates': {'poses': np.zeros((1, 4, 4)).tolist()},
        }
    )

    assert response['ok'] is True
    assert deps.detection_calls == []
    assert deps.estimators[0].calls[0]['coarse_estimates'] is not None


def test_runtime_loads_depth_for_depth_model_and_reports_errors(tmp_path: Path):
    deps = FakeRuntimeDeps(tmp_path)
    backend = MegaPoseRuntimeBackend(deps.deps)

    ok_response = backend.run_request(
        {
            'id': 1,
            'model': 'rgbd',
            'example_dir': str(tmp_path / 'example'),
            'object_label': 'cube_5cm',
        }
    )
    error_response = backend.run_request({'id': 2, 'model': 'missing', 'example_name': 'example'})

    assert ok_response['ok'] is True
    assert deps.observation_calls[0] == (tmp_path / 'example', True)
    assert error_response['ok'] is False
    assert error_response['id'] == 2
    assert 'unknown MegaPose model' in error_response['error']
