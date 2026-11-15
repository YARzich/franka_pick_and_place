"""
Test lifecycle and cache ownership of the GigaPose refiner worker.

Responsibility: Verify backend reuse, cache-key invalidation, cleanup, and correlated errors.
"""

from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.pose.pose_gigapose_refiner_worker_runtime import (  # noqa: E402
    GigaPoseRefinerWorker,
)


def _request(tmp_path: Path, **overrides) -> dict:
    request = {
        'id': 1,
        'image_path': str(tmp_path / 'rgb.png'),
        'K': [[100.0, 0.0, 50.0], [0.0, 100.0, 40.0], [0.0, 0.0, 1.0]],
        'repo_dir': str(tmp_path / 'repo'),
        'root_dir': str(tmp_path / 'data'),
        'dataset_name': 'franka_cube',
        'object_id': 7,
        'output_label': 'cube_5cm',
        'megapose_models_root': str(tmp_path / 'models'),
        'bsz_images': 32,
        'bsz_objects': 8,
        'n_iterations': 4,
        'num_workers': 1,
        'coarse_estimates': {'poses': [[[1.0]]]},
    }
    request.update(overrides)
    return request


class FakeRefinerBackend:
    instances = []

    def __init__(self, request, dependencies=None) -> None:
        self.request = request
        self.dependencies = dependencies
        self.calls = []
        self.closed = False
        FakeRefinerBackend.instances.append(self)

    def run(self, request):
        self.calls.append(request)
        return {
            'id': request.request_id,
            'ok': True,
            'object_data': [{'label': request.output_label}],
            'timing': {},
            'elapsed_sec': 0.1,
        }

    def close(self):
        self.closed = True


def test_refiner_worker_reuses_backend_cache_and_closes_backends(tmp_path: Path):
    FakeRefinerBackend.instances = []
    worker = GigaPoseRefinerWorker(backend_type=FakeRefinerBackend, dependencies='deps')

    first = worker.run_request(_request(tmp_path, id=1, output_label='cube_a', num_workers=1))
    second = worker.run_request(_request(tmp_path, id=2, output_label='cube_b', num_workers=1))
    worker.close()

    assert first['ok'] is True
    assert second['ok'] is True
    assert len(FakeRefinerBackend.instances) == 1
    assert FakeRefinerBackend.instances[0].dependencies == 'deps'
    assert len(FakeRefinerBackend.instances[0].calls) == 2
    assert FakeRefinerBackend.instances[0].closed is True


def test_refiner_worker_cache_key_changes_when_runtime_model_fields_change(tmp_path: Path):
    FakeRefinerBackend.instances = []
    worker = GigaPoseRefinerWorker(backend_type=FakeRefinerBackend)

    worker.run_request(_request(tmp_path, id=1, n_iterations=4))
    worker.run_request(_request(tmp_path, id=2, n_iterations=5))
    worker.run_request(_request(tmp_path, id=3, n_iterations=4, num_workers=9))

    assert len(FakeRefinerBackend.instances) == 3


def test_refiner_prewarm_loads_cache_without_inference_and_first_frame_reuses_it(
    tmp_path: Path,
):
    FakeRefinerBackend.instances = []
    worker = GigaPoseRefinerWorker(backend_type=FakeRefinerBackend)
    prewarm = _request(tmp_path, id=1, command='prewarm')
    for frame_field in ('image_path', 'K', 'coarse_estimates', 'output_label'):
        prewarm.pop(frame_field, None)

    prewarm_response = worker.run_request(prewarm)
    inference_response = worker.run_request(_request(tmp_path, id=2, output_label='cube'))
    worker.close()

    assert prewarm_response == {'id': 1, 'ok': True, 'prewarmed': True}
    assert inference_response['ok'] is True
    assert len(FakeRefinerBackend.instances) == 1
    assert len(FakeRefinerBackend.instances[0].calls) == 1
    assert FakeRefinerBackend.instances[0].closed is True


def test_refiner_worker_reports_runtime_errors_with_original_id(tmp_path: Path):
    worker = GigaPoseRefinerWorker(backend_type=FakeRefinerBackend)

    response = worker.run_request({'id': 4})

    assert response['id'] == 4
    assert response['ok'] is False
    assert 'error' in response
    assert 'traceback' in response
