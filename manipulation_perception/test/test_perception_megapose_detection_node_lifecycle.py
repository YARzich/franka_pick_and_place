"""
Test pose inference worker ownership and result publication ordering.

Responsibility: Verify worker shutdown, clock-generation invalidation,
late-result suppression, and backend failure handling.
"""

import subprocess
import threading
import time

from perception_pose_lifecycle_test_support import (
    FakeBackend,
    FakeInputBuffer,
    FakeLogger,
    make_actual_runner,
    make_header,
    make_image_message,
    make_pending_frames,
    POSE,
    run_backend_once,
)

import pytest

from manipulation_perception.common.header_sync import header_key
from manipulation_perception.pose.pose_inference_worker import InferenceWorkerLifecycle
from manipulation_perception.pose.pose_input_coordinator import PoseInputCoordinator


def test_inference_worker_lifecycle_rejects_late_start_and_joins():
    lifecycle = InferenceWorkerLifecycle()
    daemon_flags: list[bool] = []
    release_worker = threading.Event()
    worker_started = threading.Event()

    def blocking_worker() -> None:
        daemon_flags.append(threading.current_thread().daemon)
        worker_started.set()
        release_worker.wait(timeout=1.0)

    assert lifecycle.start(target=blocking_worker, args=(), name='test_worker')
    assert worker_started.wait(timeout=0.5)
    assert daemon_flags == [True]
    assert not lifecycle.start(target=blocking_worker, args=(), name='test_worker_2')
    assert not lifecycle.join(timeout_sec=0.01)
    lifecycle.request_shutdown()
    assert not lifecycle.start(target=blocking_worker, args=(), name='test_worker_3')
    release_worker.set()
    assert lifecycle.join(timeout_sec=1.0)


def test_runtime_cleanup_waits_for_stopped_backends_and_joined_writer(monkeypatch, tmp_path):
    runner, backend, gigapose, _logger, _publisher = make_actual_runner()
    workspace = tmp_path / 'owned-workspace'
    workspace.mkdir()
    writer_started = threading.Event()
    release_writer = threading.Event()
    events = []

    def write_until_released():
        writer_started.set()
        assert release_writer.wait(timeout=2.0)
        assert workspace.exists()
        events.append('writer joined')

    monkeypatch.setattr(backend, 'stop', lambda: events.append('megapose stopped'))
    monkeypatch.setattr(gigapose, 'stop', lambda: events.append('gigapose stopped'))

    def cleanup():
        assert 'writer joined' in events
        assert 'megapose stopped' in events
        assert 'gigapose stopped' in events
        events.append('cleanup')
        workspace.rmdir()

    runner._cleanup_runtime = cleanup
    assert runner._worker.start(target=write_until_released, args=(), name='workspace_writer')
    assert writer_started.wait(timeout=0.5)
    try:
        assert not runner.shutdown(timeout_sec=0.01)
        assert workspace.exists()
        assert 'cleanup' not in events
    finally:
        release_writer.set()
        assert runner.shutdown(timeout_sec=1.0)
    assert not workspace.exists()
    assert events[-1] == 'cleanup'


def test_runtime_cleanup_preserves_workspace_while_prewarm_has_not_joined(monkeypatch):
    runner, _backend, gigapose, _logger, _publisher = make_actual_runner()
    cleanups = []
    runner._cleanup_runtime = lambda: cleanups.append(True)
    monkeypatch.setattr(gigapose, 'stop', lambda: False)

    assert not runner.shutdown()
    assert cleanups == []
    monkeypatch.setattr(gigapose, 'stop', lambda: True)
    assert runner.shutdown()
    assert cleanups == [True]


def test_shutdown_reports_retained_workspace_after_uncertain_process_cleanup():
    runner, _backend, _gigapose, _logger, _publisher = make_actual_runner()
    runner._cleanup_runtime = lambda: False
    assert not runner.shutdown()


def test_inference_worker_start_is_atomic_with_join(monkeypatch):
    lifecycle = InferenceWorkerLifecycle()
    start_entered = threading.Event()
    release_start = threading.Event()
    start_results: list[bool] = []
    real_thread_start = threading.Thread.start

    def delayed_thread_start(thread: threading.Thread) -> None:
        if thread.name == 'owned_inference_worker':
            start_entered.set()
            assert release_start.wait(timeout=1.0)
        real_thread_start(thread)

    monkeypatch.setattr(threading.Thread, 'start', delayed_thread_start)
    starter = threading.Thread(
        target=lambda: start_results.append(
            lifecycle.start(target=lambda: None, args=(), name='owned_inference_worker')
        ),
        name='lifecycle_starter',
    )
    delayed_release = threading.Timer(0.2, release_start.set)
    try:
        starter.start()
        assert start_entered.wait(timeout=0.5)
        delayed_release.start()
        join_started = time.monotonic()
        assert not lifecycle.join(timeout_sec=0.01)
        assert time.monotonic() - join_started < 0.1
    finally:
        release_start.set()
        delayed_release.cancel()
        starter.join(timeout=1.0)

    assert not starter.is_alive()
    assert start_results == [True]
    assert lifecycle.join(timeout_sec=1.0)


def test_runner_suppresses_publish_when_shutdown_starts_during_backend_call():
    runner, backend, gigapose, logger, publisher = make_actual_runner()
    backend.lifecycle = runner._worker

    run_backend_once(runner, object())

    assert backend.calls == 1
    assert gigapose.calls == 0
    assert publisher.messages == []
    assert logger.warnings == []
    assert logger.errors == []
    assert runner.try_start(11.0)


def test_clock_jump_clears_sync_state_and_rejects_old_generation():
    runner, _backend, _gigapose, _logger, publisher = make_actual_runner()
    input_buffer = FakeInputBuffer()
    coordinator = object.__new__(PoseInputCoordinator)
    coordinator._runner = runner
    coordinator._logger = FakeLogger()
    coordinator._input_buffer = input_buffer
    coordinator._pending_frames = make_pending_frames(input_buffer)
    old_generation = runner.current_generation()
    old_image = make_image_message(make_header(sec=10, frame_id='camera'))
    old_key = header_key(old_image.header)
    coordinator._pending_frames.add(
        old_key,
        old_image,
        time.monotonic(),
        old_generation,
    )
    runner.publish_invalidation(old_image.header, old_generation)

    coordinator.on_clock_jump()
    new_generation = runner.current_generation()
    runner.publish_invalidation(make_header(sec=11, frame_id='camera'), old_generation)
    runner.publish_invalidation(make_header(sec=1, frame_id='camera'), new_generation)

    assert new_generation == old_generation + 1
    assert len(coordinator._pending_frames) == 0
    assert input_buffer.clear_calls == 1
    assert [message.header.stamp.sec for message in publisher.messages] == [10, 1]


@pytest.mark.parametrize('invalidation_sec', [10, 11])
def test_newer_invalidation_suppresses_interleaved_old_positive(
    monkeypatch,
    invalidation_sec,
):
    runner, _backend, _gigapose, _logger, publisher = make_actual_runner()
    old_header = make_header(sec=10, frame_id='camera')
    invalidation_header = make_header(sec=invalidation_sec, frame_id='camera')
    builder_started = threading.Event()
    release_builder = threading.Event()
    from manipulation_perception.pose import pose_inference_runner as runner_module

    real_builder = runner_module.build_pose_detection_array

    def blocking_builder(**kwargs):
        builder_started.set()
        assert release_builder.wait(timeout=1.0)
        return real_builder(**kwargs)

    monkeypatch.setattr(runner_module, 'build_pose_detection_array', blocking_builder)
    generation = runner.current_generation()
    worker = threading.Thread(
        target=runner._publish_detection,
        args=(POSE, old_header, generation),
    )
    worker.start()
    assert builder_started.wait(timeout=0.5)
    runner.publish_invalidation(invalidation_header, generation)
    release_builder.set()
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert len(publisher.messages) == 1
    assert publisher.messages[0].header.stamp.sec == invalidation_sec
    assert list(publisher.messages[0].detections) == []


def test_worker_no_pose_publishes_invalidation():
    backend = FakeBackend()
    backend.result = None
    runner, _backend, _gigapose, logger, publisher = make_actual_runner(backend=backend)
    image_header = make_header(sec=15, frame_id='camera')

    run_backend_once(runner, image_header)

    assert [message.header.stamp.sec for message in publisher.messages] == [15]
    assert any('не вернул pose' in message for message in logger.warnings)


@pytest.mark.parametrize(
    ('error', 'expected_log'),
    [
        (subprocess.TimeoutExpired('megapose', 1.0), 'превысил timeout'),
        (RuntimeError('backend failed'), 'Ошибка MegaPose inference'),
    ],
)
def test_worker_failure_publishes_invalidation(error, expected_log):
    backend = FakeBackend()
    backend.error = error
    runner, _backend, _gigapose, logger, publisher = make_actual_runner(backend=backend)
    image_header = make_header(sec=16, frame_id='camera')

    run_backend_once(runner, image_header)

    assert [message.header.stamp.sec for message in publisher.messages] == [16]
    assert any(expected_log in message for message in logger.errors)
