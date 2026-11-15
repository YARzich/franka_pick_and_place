"""
Test GigaPose coarse and refinement pipeline orchestration.

Responsibility: Verify coarse/refiner request flow, mask-derived ROI,
empty-result handling, and stop admission around runtime preparation.
"""

from pathlib import Path
import threading
import time

import numpy as np
import pytest

from perception_pose_backend_test_support import (  # noqa: I100,I201
    _gigapose_runner,
    FakeWorkerClient,
    K_VALUES,
    POSE,
)


def test_gigapose_refined_pipeline_no_poses_skips_refiner(tmp_path: Path):
    worker = FakeWorkerClient([{'id': 1, 'ok': True, 'poses': []}])
    refiner = FakeWorkerClient()
    runner, logs, _ = _gigapose_runner(tmp_path, worker=worker, refiner=refiner)
    rgb = np.zeros((10, 10, 3), dtype=np.uint8)

    assert runner.run_refined(rgb, None, K_VALUES, (1, 1, 8, 8)) is None
    assert refiner.requests == []
    assert any('coarse-stage' in message for message in logs.warnings)


def test_gigapose_refined_pipeline_passes_coarse_estimates_to_refiner(tmp_path: Path):
    worker = FakeWorkerClient(
        [{'id': 1, 'ok': True, 'poses': [[[1.0]]], 'scores': [0.9], 'elapsed_sec': 0.4}]
    )
    refiner = FakeWorkerClient(
        [{'id': 1, 'ok': True, 'object_data': [POSE], 'timing': {'refiner': 0.2}}]
    )
    runner, logs, _ = _gigapose_runner(
        tmp_path,
        worker=worker,
        refiner=refiner,
        min_matching_score=0.9,
    )
    rgb = np.zeros((10, 10, 3), dtype=np.uint8)
    mask = np.zeros((10, 10), dtype=np.uint8)
    mask[2:6, 3:7] = 1

    assert runner.run_refined(rgb, mask, K_VALUES, (1, 1, 8, 8)) is POSE
    assert worker.requests[0]['bbox'] == [3, 2, 7, 6]
    assert refiner.requests[0]['coarse_estimates'] == {
        'poses': [[[1.0]]],
        'scores': [0.9],
        'elapsed_sec': 0.4,
    }
    assert any('bbox из mask' in message for message in logs.infos)
    assert any('GigaPose refiner timing' in message for message in logs.infos)


def test_gigapose_quality_gate_skips_refiner_below_matching_threshold(
    tmp_path: Path,
):
    worker = FakeWorkerClient(
        [{'id': 1, 'ok': True, 'poses': [[[1.0]]], 'scores': [0.09]}]
    )
    refiner = FakeWorkerClient()
    runner, logs, _ = _gigapose_runner(
        tmp_path,
        worker=worker,
        refiner=refiner,
        min_matching_score=0.1,
    )
    rgb = np.zeros((10, 10, 3), dtype=np.uint8)

    assert runner.run_refined(rgb, None, K_VALUES, (1, 1, 8, 8)) is None
    assert refiner.requests == []
    assert any(
        'actual=0.090000' in message and 'threshold=0.100000' in message
        for message in logs.warnings
    )


@pytest.mark.parametrize(
    ('response', 'actual'),
    [
        ({'poses': [[[1.0]]]}, 'actual=unavailable'),
        ({'poses': [[[1.0]]], 'scores': []}, 'actual=unavailable'),
        ({'poses': [[[1.0]]], 'scores': [float('nan')]}, 'actual=non-finite'),
        ({'poses': [[[1.0]]], 'scores': ['bad']}, 'actual=malformed'),
        (
            {'poses': [[[1.0]], [[2.0]]], 'scores': [0.9]},
            'actual=count=1',
        ),
    ],
)
def test_gigapose_quality_gate_rejects_unusable_coarse_scores_before_refiner(
    tmp_path: Path,
    response: dict,
    actual: str,
):
    worker = FakeWorkerClient([{'id': 1, 'ok': True, **response}])
    refiner = FakeWorkerClient()
    runner, logs, _ = _gigapose_runner(
        tmp_path,
        worker=worker,
        refiner=refiner,
        min_matching_score=0.0,
    )
    rgb = np.zeros((10, 10, 3), dtype=np.uint8)

    assert runner.run_refined(rgb, None, K_VALUES, (1, 1, 8, 8)) is None
    assert refiner.requests == []
    assert any(
        actual in message and 'threshold=0.000000' in message
        for message in logs.warnings
    )


@pytest.mark.parametrize('threshold', [float('nan'), float('inf'), -0.01, 1.01])
def test_gigapose_runner_rejects_invalid_matching_threshold(
    tmp_path: Path,
    threshold: float,
):
    with pytest.raises(ValueError, match='min_matching_score'):
        _gigapose_runner(
            tmp_path,
            worker=FakeWorkerClient(),
            refiner=FakeWorkerClient(),
            min_matching_score=threshold,
        )


def test_gigapose_refiner_no_pose_returns_none(tmp_path: Path):
    worker = FakeWorkerClient([{'id': 1, 'ok': True, 'poses': [[[1.0]]], 'scores': [0.9]}])
    refiner = FakeWorkerClient([{'id': 1, 'ok': True, 'object_data': []}])
    runner, _, _ = _gigapose_runner(tmp_path, worker=worker, refiner=refiner)
    rgb = np.zeros((10, 10, 3), dtype=np.uint8)

    assert runner.run_refined(rgb, None, K_VALUES, (1, 1, 8, 8)) is None


def test_gigapose_coarse_pipeline_returns_label_poses_and_scores(tmp_path: Path):
    worker = FakeWorkerClient([{'id': 1, 'ok': True, 'poses': [[[1.0]]], 'scores': [0.9]}])
    refiner = FakeWorkerClient()
    runner, _, _ = _gigapose_runner(tmp_path, worker=worker, refiner=refiner)
    rgb = np.zeros((10, 10, 3), dtype=np.uint8)

    result = runner.run_coarse(rgb, None, K_VALUES, (1, 1, 8, 8))

    assert result == {'label': 'cube_5cm', 'poses': [[[1.0]]], 'scores': [0.9]}
    assert refiner.requests == []


def test_gigapose_stop_propagates_to_worker_clients(tmp_path: Path):
    worker = FakeWorkerClient()
    refiner = FakeWorkerClient()
    runner, _, _ = _gigapose_runner(tmp_path, worker=worker, refiner=refiner)

    runner.stop()

    assert worker.stopped is True
    assert refiner.stopped is True


def test_gigapose_startup_prewarm_is_sequential_idempotent_and_model_only(
    tmp_path: Path,
):
    request_order: list[str] = []

    class OrderedWorkerClient(FakeWorkerClient):
        def __init__(self, name: str) -> None:
            super().__init__([{'ok': True, 'prewarmed': True}])
            self.name = name

        def request(self, request: dict) -> dict:
            request_order.append(self.name)
            return super().request(request)

    worker = OrderedWorkerClient('coarse')
    refiner = OrderedWorkerClient('refiner')
    runner, logs, _ = _gigapose_runner(tmp_path, worker=worker, refiner=refiner)

    assert runner.start_prewarm() is True
    assert runner.start_prewarm() is False
    deadline = time.monotonic() + 1.0
    while len(request_order) < 2 and time.monotonic() < deadline:
        time.sleep(0.005)

    assert request_order == ['coarse', 'refiner']
    assert worker.requests[0]['command'] == 'prewarm'
    assert refiner.requests[0]['command'] == 'prewarm'
    for request in (worker.requests[0], refiner.requests[0]):
        assert 'image_path' not in request
        assert 'K' not in request
    assert any('GigaPose coarse startup prewarm завершён' in line for line in logs.infos)
    assert any('MegaPose refiner startup prewarm завершён' in line for line in logs.infos)
    runner.stop()
    assert runner.start_prewarm() is False


def test_gigapose_prewarm_requires_persistent_coarse_worker(tmp_path: Path):
    worker = FakeWorkerClient(persistent=False)
    refiner = FakeWorkerClient()
    runner, _, _ = _gigapose_runner(tmp_path, worker=worker, refiner=refiner)

    assert runner.start_prewarm() is False
    assert worker.requests == []
    assert refiner.requests == []


def test_gigapose_stop_during_coarse_prewarm_skips_refiner_and_late_success_log(
    tmp_path: Path,
):
    prewarm_started = threading.Event()
    release_prewarm = threading.Event()

    class BlockingPrewarmClient(FakeWorkerClient):
        def request(self, request: dict) -> dict:
            self.requests.append(request)
            prewarm_started.set()
            release_prewarm.wait(timeout=2.0)
            return {'ok': True, 'prewarmed': True}

    worker = BlockingPrewarmClient()
    refiner = FakeWorkerClient([{'ok': True, 'prewarmed': True}])
    runner, logs, _ = _gigapose_runner(tmp_path, worker=worker, refiner=refiner)
    inference_results: list[dict | None] = []

    assert runner.start_prewarm() is True
    assert prewarm_started.wait(timeout=0.5)
    inference_thread = threading.Thread(
        target=lambda: inference_results.append(
            runner.run_coarse(
                np.zeros((2, 3, 3), dtype=np.uint8),
                None,
                K_VALUES,
                (1, 1, 2, 2),
            )
        ),
        daemon=True,
    )
    inference_thread.start()
    runner.stop()
    inference_thread.join(timeout=1.0)
    release_prewarm.set()
    assert runner._prewarm.join(timeout_sec=1.0)  # noqa: SLF001 - lifecycle contract

    assert not inference_thread.is_alive()
    assert inference_results == [None]
    assert refiner.requests == []
    assert not any('startup prewarm завершён' in line for line in logs.infos)
    assert logs.warnings == []


def test_gigapose_first_inference_serializes_behind_coarse_prewarm(tmp_path: Path):
    prewarm_started = threading.Event()
    refiner_prewarm_started = threading.Event()
    release_prewarm = threading.Event()
    release_refiner_prewarm = threading.Event()

    class SerializedCoarseClient(FakeWorkerClient):
        def __init__(self) -> None:
            super().__init__()
            self._request_lock = threading.Lock()

        def request(self, request: dict) -> dict:
            with self._request_lock:
                self.requests.append(request)
                if request.get('command') == 'prewarm':
                    prewarm_started.set()
                    release_prewarm.wait(timeout=2.0)
                    return {'ok': True, 'prewarmed': True}
                return {'ok': True, 'poses': [[[1.0]]], 'scores': [0.9]}

    class BlockingRefinerClient(FakeWorkerClient):
        def request(self, request: dict) -> dict:
            self.requests.append(request)
            refiner_prewarm_started.set()
            release_refiner_prewarm.wait(timeout=2.0)
            return {'ok': True, 'prewarmed': True}

    worker = SerializedCoarseClient()
    refiner = BlockingRefinerClient()
    runner, _, _ = _gigapose_runner(tmp_path, worker=worker, refiner=refiner)
    results: list[dict | None] = []

    assert runner.start_prewarm() is True
    assert prewarm_started.wait(timeout=0.5)
    inference_thread = threading.Thread(
        target=lambda: results.append(
            runner.run_coarse(
                np.zeros((2, 3, 3), dtype=np.uint8),
                None,
                K_VALUES,
                (1, 1, 2, 2),
            )
        ),
        daemon=True,
    )
    inference_thread.start()
    assert [request.get('command') for request in worker.requests] == ['prewarm']

    release_prewarm.set()
    assert refiner_prewarm_started.wait(timeout=0.5)
    assert [request.get('command') for request in worker.requests] == ['prewarm']
    release_refiner_prewarm.set()
    inference_thread.join(timeout=1.0)
    runner.stop()

    assert not inference_thread.is_alive()
    assert [request.get('command') for request in worker.requests] == ['prewarm', None]
    assert results == [
        {'label': 'cube_5cm', 'poses': [[[1.0]]], 'scores': [0.9]}
    ]


def test_gigapose_stop_during_runtime_prepare_prevents_late_worker_start(tmp_path: Path):
    worker = FakeWorkerClient([{'id': 1, 'ok': True, 'poses': [[[1.0]]]}])
    refiner = FakeWorkerClient([{'id': 1, 'ok': True, 'object_data': [POSE]}])
    runner, _, runtime_input = _gigapose_runner(tmp_path, worker=worker, refiner=refiner)
    prepare_started = threading.Event()
    release_prepare = threading.Event()
    original_prepare = runtime_input.prepare

    def blocking_prepare(*args):
        prepare_started.set()
        release_prepare.wait(timeout=1.0)
        return original_prepare(*args)

    runtime_input.prepare = blocking_prepare
    results = []
    thread = threading.Thread(
        target=lambda: results.append(
            runner.run_refined(
                np.zeros((2, 3, 3), dtype=np.uint8),
                None,
                K_VALUES,
                (1, 1, 2, 2),
            )
        ),
        daemon=True,
    )

    thread.start()
    assert prepare_started.wait(timeout=0.5)
    runner.stop()
    release_prepare.set()
    thread.join(timeout=1.0)

    assert not thread.is_alive()
    assert results == [None]
    assert worker.requests == []
    assert refiner.requests == []
