"""
Test lifecycle and cache ownership of the GigaPose coarse worker.

Responsibility: Verify backend reuse, request-local options, cache invalidation, and error IDs.
"""

from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.pose.pose_gigapose_worker_runtime import (  # noqa: E402
    GigaPoseWorker,
)


def _request(tmp_path: Path, **overrides) -> dict:
    request = {
        'id': 1,
        'image_path': str(tmp_path / 'rgb.png'),
        'mask_path': '',
        'bbox': [1, 2, 3, 4],
        'K': [[100.0, 0.0, 50.0], [0.0, 100.0, 40.0], [0.0, 0.0, 1.0]],
        'repo_dir': str(tmp_path / 'repo'),
        'root_dir': str(tmp_path / 'data'),
        'dataset_name': 'franka_cube',
        'object_id': 7,
        'checkpoint_path': str(tmp_path / 'checkpoint.ckpt'),
        'translation_scale': 0.001,
    }
    request.update(overrides)
    return request


class FakeGigaPoseBackend:
    instances = []

    def __init__(self, request, dependencies=None) -> None:
        self.request = request
        self.dependencies = dependencies
        self.calls = []
        FakeGigaPoseBackend.instances.append(self)

    def run(self, request):
        self.calls.append(request)
        return {
            'poses': [[[float(request.num_hypotheses)]]],
            'scores': [0.5],
            'elapsed_sec': 0.1,
        }


def test_worker_reuses_backend_cache_and_keeps_num_hypotheses_request_local(tmp_path: Path):
    FakeGigaPoseBackend.instances = []
    worker = GigaPoseWorker(backend_type=FakeGigaPoseBackend, dependencies='deps')

    first = worker.run_request(_request(tmp_path, id=1, num_hypotheses=2))
    second = worker.run_request(_request(tmp_path, id=2, num_hypotheses=9))

    assert first['ok'] is True
    assert first['poses'] == [[[2.0]]]
    assert second['ok'] is True
    assert second['poses'] == [[[9.0]]]
    assert len(FakeGigaPoseBackend.instances) == 1
    assert FakeGigaPoseBackend.instances[0].dependencies == 'deps'
    assert len(FakeGigaPoseBackend.instances[0].calls) == 2


def test_worker_cache_key_changes_when_runtime_model_fields_change(tmp_path: Path):
    FakeGigaPoseBackend.instances = []
    worker = GigaPoseWorker(backend_type=FakeGigaPoseBackend)

    worker.run_request(_request(tmp_path, id=1, translation_scale=0.001))
    worker.run_request(_request(tmp_path, id=2, translation_scale=0.002))

    assert len(FakeGigaPoseBackend.instances) == 2


def test_worker_prewarm_loads_cache_without_inference_and_first_frame_reuses_it(
    tmp_path: Path,
):
    FakeGigaPoseBackend.instances = []
    worker = GigaPoseWorker(backend_type=FakeGigaPoseBackend)
    prewarm = _request(tmp_path, id=1, command='prewarm')
    for frame_field in ('image_path', 'mask_path', 'bbox', 'K', 'num_hypotheses'):
        prewarm.pop(frame_field, None)

    prewarm_response = worker.run_request(prewarm)
    inference_response = worker.run_request(_request(tmp_path, id=2, num_hypotheses=5))

    assert prewarm_response == {'id': 1, 'ok': True, 'prewarmed': True}
    assert inference_response['ok'] is True
    assert len(FakeGigaPoseBackend.instances) == 1
    assert len(FakeGigaPoseBackend.instances[0].calls) == 1


def test_worker_reports_runtime_errors_with_original_id(tmp_path: Path):
    worker = GigaPoseWorker(backend_type=FakeGigaPoseBackend)

    response = worker.run_request({'id': 4})

    assert response['id'] == 4
    assert response['ok'] is False
    assert 'error' in response
    assert 'traceback' in response
