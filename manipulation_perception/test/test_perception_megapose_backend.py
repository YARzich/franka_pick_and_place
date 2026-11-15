"""
Test MegaPose worker, CLI fallback, and shutdown orchestration.

Responsibility: Verify request correlation, fallback policy, runtime-file
preparation, and deterministic termination of active CLI process groups.
"""

import os
from pathlib import Path
import subprocess
import sys
import threading
import time

import numpy as np
import pytest

from manipulation_perception.pose.pose_backend_common import RequestIdCounter
from manipulation_perception.pose.pose_megapose_backend import (
    MegaPoseBackendRunner,
    MegaPoseCliConfig,
    MegaPoseWorkerRequestConfig,
)
from perception_pose_backend_test_support import (  # noqa: I100,I201
    _megapose_runner,
    FakeRuntimeExample,
    FakeWorkerClient,
    K_VALUES,
    LogCapture,
    POSE,
)
from perception_process_test_support import (  # noqa: I100,I201
    _GRANDCHILD_CODE,
    _kill_leftover_pid,
    _wait_for_pid_file,
    _wait_for_process_exit,
    _wait_until,
)


def _real_cli_runner(tmp_path: Path, source: str, *, timeout_sec: float = 2.0):
    module_dir = tmp_path / 'megapose' / 'scripts'
    module_dir.mkdir(parents=True)
    (tmp_path / 'megapose' / '__init__.py').write_text('')
    (module_dir / '__init__.py').write_text('')
    (module_dir / 'run_inference_on_example.py').write_text(source)
    env = dict(os.environ)
    env['PYTHONPATH'] = f"{tmp_path}:{env.get('PYTHONPATH', '')}"
    output_path = tmp_path / 'output.json'
    logs = LogCapture()
    runner = MegaPoseBackendRunner(
        runtime_example=FakeRuntimeExample([output_path]),
        worker_client=FakeWorkerClient(),
        worker_config=MegaPoseWorkerRequestConfig(
            example_name='test_object',
            example_dir=tmp_path / 'example',
            object_label=POSE['label'],
            bsz_images=64,
            bsz_objects=8,
            renderer_workers=1,
        ),
        cli_config=MegaPoseCliConfig(
            python_path=sys.executable,
            example_name='test_object',
            output_pose_paths=[output_path],
            object_label=POSE['label'],
            process_timeout_sec=timeout_sec,
            env_factory=lambda: dict(env),
        ),
        use_worker=False,
        verbose=True,
        log_info=logs.info,
        log_error=logs.error,
        subprocess_run=None,
    )
    return runner, logs


@pytest.mark.parametrize('exit_code', [0, 7])
def test_megapose_cli_handles_large_stdout_stderr_and_exit_code(tmp_path: Path, exit_code: int):
    output_path = tmp_path / 'output.json'
    runner, logs = _real_cli_runner(
        tmp_path,
        "import json, sys\n"
        "from pathlib import Path\n"
        "print('stdout' * 50000, flush=True)\n"
        "print('stderr' * 50000, file=sys.stderr, flush=True)\n"
        f"Path({str(output_path)!r}).write_text(json.dumps([{POSE!r}]))\n"
        f"sys.exit({exit_code})\n",
    )
    try:
        result = runner.run('model', coarse_estimates=None)
        if exit_code == 0:
            assert result == POSE
            assert 'stdout' * 50000 in logs.infos
            assert logs.errors == []
        else:
            assert result is None
            assert len(logs.errors) == 1
            assert '(7)' in logs.errors[0]
            assert logs.errors[0].endswith('stderr' * 50000)
    finally:
        runner.stop()


def test_megapose_cli_timeout_captures_output_and_kills_children(tmp_path: Path):
    child_pid_file = tmp_path / 'cli_timeout_child.pid'
    runner, _ = _real_cli_runner(
        tmp_path,
        "import subprocess, sys, time\n"
        "print('stdout' * 50000, flush=True)\n"
        "print('stderr' * 50000, file=sys.stderr, flush=True)\n"
        f"subprocess.Popen([sys.executable, '-c', {_GRANDCHILD_CODE!r}, "
        f"{str(child_pid_file)!r}])\n"
        "time.sleep(30)\n",
        timeout_sec=0.5,
    )
    child_pid = 0
    try:
        started_at = time.monotonic()
        with pytest.raises(subprocess.TimeoutExpired) as raised:
            runner.run('model', coarse_estimates=None)
        assert time.monotonic() - started_at < 3.0
        assert raised.value.output == b'stdout' * 50000 + b'\n'
        assert raised.value.stderr == b'stderr' * 50000 + b'\n'
        child_pid = _wait_for_pid_file(child_pid_file)
        assert _wait_for_process_exit(child_pid)
    finally:
        runner.stop()
        if child_pid:
            _kill_leftover_pid(child_pid)


def test_request_id_counter_starts_from_one_and_keeps_incrementing():
    counter = RequestIdCounter()

    assert [counter.next_id(), counter.next_id(), counter.next_id()] == [1, 2, 3]


def test_request_id_counter_is_unique_across_prewarm_and_inference_threads():
    counter = RequestIdCounter()
    ids: list[int] = []
    threads = [
        threading.Thread(target=lambda: ids.extend(counter.next_id() for _ in range(100)))
        for _ in range(2)
    ]

    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=1.0)

    assert all(not thread.is_alive() for thread in threads)
    assert sorted(ids) == list(range(1, 201))


def test_megapose_worker_success_returns_pose_without_cli(tmp_path: Path):
    worker = FakeWorkerClient(
        [{'id': 1, 'ok': True, 'object_data': [POSE], 'timing': {'total': 1.2}}]
    )
    runner, subprocess_run, logs, _ = _megapose_runner(tmp_path, worker=worker)

    assert runner.run('megapose-model', coarse_estimates=None) is POSE
    assert worker.requests[0]['id'] == 1
    assert worker.requests[0]['model'] == 'megapose-model'
    assert 'coarse_estimates' not in worker.requests[0]
    assert subprocess_run.calls == []
    assert any('MegaPose worker timing' in message for message in logs.infos)


def test_megapose_cli_only_path_skips_worker_and_reads_output_file(tmp_path: Path):
    worker = FakeWorkerClient()
    runner, subprocess_run, _, _ = _megapose_runner(
        tmp_path,
        worker=worker,
        use_worker=False,
        output_pose=POSE,
    )

    assert runner.run('megapose-model', coarse_estimates=None) == POSE
    assert worker.requests == []
    assert len(subprocess_run.calls) == 1
    command = subprocess_run.calls[0][0][0]
    assert command[:4] == [
        sys.executable,
        '-m',
        'megapose.scripts.run_inference_on_example',
        'franka_motion_cube_5cm',
    ]


def test_megapose_worker_no_pose_falls_back_to_cli_output_file(tmp_path: Path):
    worker = FakeWorkerClient([{'id': 1, 'ok': True, 'object_data': []}])
    runner, subprocess_run, logs, _ = _megapose_runner(tmp_path, worker=worker, output_pose=POSE)

    assert runner.run('megapose-model', coarse_estimates=None) == POSE
    assert len(subprocess_run.calls) == 1
    assert any('worker не вернул pose' in message for message in logs.warnings)


def test_megapose_worker_exception_stops_worker_and_falls_back_to_cli(tmp_path: Path):
    worker = FakeWorkerClient(exc=RuntimeError('worker down'))
    runner, subprocess_run, logs, _ = _megapose_runner(tmp_path, worker=worker, output_pose=POSE)

    assert runner.run('megapose-model', coarse_estimates=None) == POSE
    assert worker.stopped is True
    assert len(subprocess_run.calls) == 1
    assert any('worker down' in message for message in logs.errors)


def test_megapose_coarse_estimates_force_worker_and_never_call_cli(tmp_path: Path):
    coarse_estimates = {'poses': [[[1.0]]], 'scores': [0.9]}
    worker = FakeWorkerClient([{'id': 1, 'ok': True, 'object_data': [POSE]}])
    runner, subprocess_run, _, _ = _megapose_runner(tmp_path, worker=worker, use_worker=False)

    assert runner.run('megapose-model', coarse_estimates=coarse_estimates) is POSE
    assert worker.requests[0]['coarse_estimates'] is coarse_estimates
    assert subprocess_run.calls == []


def test_megapose_coarse_estimates_no_pose_or_exception_return_none_without_cli(tmp_path: Path):
    coarse_estimates = {'poses': [], 'scores': []}
    worker = FakeWorkerClient([{'id': 1, 'ok': True, 'object_data': []}])
    runner, subprocess_run, logs, _ = _megapose_runner(tmp_path, worker=worker, use_worker=False)

    assert runner.run('megapose-model', coarse_estimates=coarse_estimates) is None
    assert subprocess_run.calls == []
    assert any('worker не вернул pose' in message for message in logs.warnings)

    failing_worker = FakeWorkerClient(exc=RuntimeError('worker down'))
    runner, subprocess_run, _, _ = _megapose_runner(
        tmp_path,
        worker=failing_worker,
        use_worker=False,
    )

    assert runner.run('megapose-model', coarse_estimates=coarse_estimates) is None
    assert failing_worker.stopped is True
    assert subprocess_run.calls == []


def test_megapose_runtime_example_prepare_is_part_of_runner(tmp_path: Path):
    worker = FakeWorkerClient([{'id': 1, 'ok': True, 'object_data': [POSE]}])
    runner, _, _, runtime_example = _megapose_runner(tmp_path, worker=worker)
    rgb = np.zeros((2, 3, 3), dtype=np.uint8)
    depth = np.ones((2, 3), dtype=np.float32)

    assert runner.run_with_runtime_example(
        rgb,
        depth,
        True,
        'model',
        K_VALUES,
        (1, 1, 2, 2),
    ) is POSE
    assert runtime_example.prepare_calls[0][2:] == (True, K_VALUES, (1, 1, 2, 2))


def test_megapose_stop_propagates_to_worker_client(tmp_path: Path):
    worker = FakeWorkerClient()
    runner, _, _, _ = _megapose_runner(tmp_path, worker=worker)

    runner.stop()

    assert worker.stopped is True


def test_megapose_stop_during_runtime_prepare_prevents_late_worker_or_cli_start(
    tmp_path: Path,
):
    worker = FakeWorkerClient([{'id': 1, 'ok': True, 'object_data': [POSE]}])
    runner, subprocess_run, _, runtime_example = _megapose_runner(tmp_path, worker=worker)
    prepare_started = threading.Event()
    release_prepare = threading.Event()
    original_prepare = runtime_example.prepare

    def blocking_prepare(*args) -> None:
        prepare_started.set()
        release_prepare.wait(timeout=1.0)
        original_prepare(*args)

    runtime_example.prepare = blocking_prepare
    results = []
    thread = threading.Thread(
        target=lambda: results.append(
            runner.run_with_runtime_example(
                np.zeros((2, 3, 3), dtype=np.uint8),
                None,
                False,
                'model',
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
    assert subprocess_run.calls == []


def test_megapose_stop_interrupts_active_cli_fallback(tmp_path: Path):
    module_dir = tmp_path / 'megapose' / 'scripts'
    grandchild_pid_file = tmp_path / 'cli_grandchild.pid'
    module_dir.mkdir(parents=True)
    (tmp_path / 'megapose' / '__init__.py').write_text('')
    (module_dir / '__init__.py').write_text('')
    (module_dir / 'run_inference_on_example.py').write_text(
        "import os, subprocess, sys, time\n"
        "subprocess.Popen([\n"
        "    sys.executable,\n"
        "    '-c',\n"
        f"    {_GRANDCHILD_CODE!r},\n"
        "    os.environ['MEGAPOSE_TEST_GRANDCHILD_PID_FILE'],\n"
        "])\n"
        "time.sleep(30)\n"
    )
    env = dict(os.environ)
    env['PYTHONPATH'] = f"{tmp_path}:{env.get('PYTHONPATH', '')}"
    env['MEGAPOSE_TEST_GRANDCHILD_PID_FILE'] = str(grandchild_pid_file)
    logs = LogCapture()
    worker = FakeWorkerClient()
    output_path = tmp_path / 'outputs' / 'object_data.json'
    runner = MegaPoseBackendRunner(
        runtime_example=FakeRuntimeExample([output_path]),
        worker_client=worker,
        worker_config=MegaPoseWorkerRequestConfig(
            example_name='franka_motion_cube_5cm',
            example_dir=tmp_path / 'example',
            object_label='cube_5cm',
            bsz_images=64,
            bsz_objects=8,
            renderer_workers=1,
        ),
        cli_config=MegaPoseCliConfig(
            python_path=sys.executable,
            example_name='franka_motion_cube_5cm',
            output_pose_paths=[output_path],
            object_label='cube_5cm',
            process_timeout_sec=30.0,
            env_factory=lambda: dict(env),
        ),
        use_worker=False,
        log_error=logs.error,
        subprocess_run=None,
    )
    results: list[dict | None] = []

    def run_cli() -> None:
        results.append(runner.run('megapose-model', coarse_estimates=None))

    def cli_is_active() -> bool:
        with runner._cli_process_lock:
            return runner._cli_process is not None

    thread = threading.Thread(target=run_cli, daemon=True)
    grandchild_pid = 0
    thread.start()
    try:
        assert _wait_until(cli_is_active)
        grandchild_pid = _wait_for_pid_file(grandchild_pid_file)
        started_at = time.monotonic()
        runner.stop()
        thread.join(timeout=1.0)
        assert not thread.is_alive()
        assert time.monotonic() - started_at < 1.0
        assert results == [None]
        assert _wait_for_process_exit(grandchild_pid)
        assert any('MegaPose subprocess' in message for message in logs.errors)
    finally:
        runner.stop()
        if grandchild_pid:
            _kill_leftover_pid(grandchild_pid)
