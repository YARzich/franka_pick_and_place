"""
Test pose-backend factory dependency injection.

Responsibility: Verify that one validated parameter snapshot configures
MegaPose and GigaPose runners without starting external processes.
"""

import io
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from manipulation_perception.pose import pose_megapose_backend, pose_worker_client
from manipulation_perception.pose.pose_backend_factory import create_pose_backend_runners
from perception_pose_backend_test_support import LogCapture  # noqa: I100,I201


def _parameters(tmp_path: Path):
    return SimpleNamespace(
        example_name='franka_motion_cube_5cm',
        example_dir=tmp_path / 'examples' / 'franka_motion_cube_5cm',
        gigapose_runtime_dir=tmp_path / 'gigapose_data' / 'ros_runtime' / 'inputs' / 'cube_5cm',
        common=SimpleNamespace(
            process_timeout_sec=7.0,
            verbose_subprocess=True,
        ),
        megapose=SimpleNamespace(
            object_label='cube_5cm',
            mesh_file_path=tmp_path / 'cube.obj',
            mesh_units='m',
            megapose_python=sys.executable,
            megapose_data_dir=tmp_path / 'megapose_data',
            persistent_worker=True,
            bsz_images=16,
            bsz_objects=4,
            renderer_workers=2,
        ),
        gigapose=SimpleNamespace(
            gigapose_python=sys.executable,
            gigapose_repo_dir=tmp_path / 'gigaPose',
            gigapose_dinov2_repo_dir=tmp_path / 'dinov2',
            gigapose_data_dir=tmp_path / 'gigapose_data',
            gigapose_persistent_worker=False,
            gigapose_dataset_name='franka_cube',
            gigapose_object_id=3,
            gigapose_checkpoint_path=tmp_path / 'checkpoint.ckpt',
            gigapose_template_count=42,
            gigapose_max_num_dets_per_forward=5,
            gigapose_translation_scale=0.002,
            gigapose_num_hypotheses=6,
            gigapose_min_matching_score=0.25,
            gigapose_refiner_bsz_images=32,
            gigapose_refiner_bsz_objects=8,
            gigapose_refiner_iterations=4,
        ),
    )


def test_backend_factory_wires_params_to_runners(tmp_path: Path):
    params = _parameters(tmp_path)
    logs = LogCapture()

    runners = create_pose_backend_runners(
        params,
        tmp_path / 'megapose_detection_node.py',
        log_info=logs.info,
        log_warn=logs.warn,
        log_error=logs.error,
        log_warn_throttled=logs.warn,
    )

    assert runners.megapose.worker_config.object_label == 'cube_5cm'
    assert runners.megapose.cli_config.example_name == 'franka_motion_cube_5cm'
    assert runners.megapose.use_worker is True
    assert runners.megapose.worker_client.persistent is True
    assert runners.gigapose.worker_client.persistent is False
    assert runners.gigapose.refiner_worker_client.shutdown_command == {'command': 'shutdown'}
    assert runners.gigapose.worker_config.object_id == 3
    assert runners.gigapose.refiner_config.n_iterations == 4
    assert runners.gigapose.min_matching_score == 0.25
    runners.megapose.stop()
    runners.gigapose.stop()
    runners.cleanup()


@pytest.mark.parametrize('backend', ['megapose', 'coarse', 'refiner', 'cli'])
def test_workspace_survives_failed_reap_after_timeout_detaches_handle(tmp_path, monkeypatch, backend):
    logs = LogCapture()
    runners = create_pose_backend_runners(
        _parameters(tmp_path), tmp_path / 'node.py', log_info=logs.info,
        log_warn=logs.warn, log_error=logs.error,
    )
    process = SimpleNamespace(stdin=io.StringIO(), stdout=None, stderr=None)

    def timeout(*_args, **_kwargs):
        raise subprocess.TimeoutExpired('worker', 0.01)

    try:
        if backend == 'cli':
            process.wait_for_exit = timeout
            monkeypatch.setattr(pose_megapose_backend, 'start_process_group', lambda *a, **k: process)
            monkeypatch.setattr(pose_megapose_backend, 'terminate_process_group', lambda *a, **k: False)
            with pytest.raises(subprocess.TimeoutExpired):
                runners.megapose._run_cli_process(['fake'])
            assert runners.megapose._cli_process is None
        else:
            client = {
                'megapose': runners.megapose.worker_client,
                'coarse': runners.gigapose.worker_client,
                'refiner': runners.gigapose.refiner_worker_client,
            }[backend]
            client.persistent = True
            client._process = process
            monkeypatch.setattr(client, '_ensure_process', lambda: process)
            monkeypatch.setattr(client, '_read_response', timeout)
            monkeypatch.setattr(pose_worker_client, 'terminate_process_group', lambda *a, **k: False)
            with pytest.raises(subprocess.TimeoutExpired):
                client.request({'id': 1})
            assert client._process is None
        runners.megapose.stop()
        runners.gigapose.stop()
        assert runners.cleanup() is False
        assert runners.workspace.root.exists()
        # A repeated stop cannot turn an earlier uncertain reap into evidence.
        runners.megapose.stop()
        runners.gigapose.stop()
        assert runners.cleanup() is False
    finally:
        # No real process was started; release the deliberately retained fixture.
        runners.workspace.cleanup()


def test_backend_instances_do_not_share_runtime_files_or_write_asset_roots(tmp_path):
    params = _parameters(tmp_path)
    model_dir = params.megapose.megapose_data_dir / 'megapose-models'
    model_dir.mkdir(parents=True)
    (model_dir / 'weights.bin').write_bytes(b'model')
    dataset_dir = params.gigapose.gigapose_data_dir / 'datasets'
    dataset_dir.mkdir(parents=True)
    (dataset_dir / 'model.json').write_text('immutable')
    template_dir = dataset_dir / 'templates' / 'franka_cube'
    (template_dir / '000003').mkdir(parents=True)
    (template_dir / '000003' / 'image.png').write_bytes(b'input asset')
    (template_dir / 'preprocessed').mkdir()
    (template_dir / 'preprocessed' / '000003.npz').write_bytes(b'shared cache')
    cache_dir = params.gigapose.gigapose_data_dir / 'torch' / 'hub'
    cache_dir.mkdir(parents=True)
    (cache_dir / 'weights.bin').write_bytes(b'cached')
    params.megapose.mesh_file_path.write_text('v 0 0 0\nv 0.1 0.1 0.1\n')
    logs = LogCapture()
    first = create_pose_backend_runners(
        params, tmp_path / 'node.py', log_info=logs.info,
        log_warn=logs.warn, log_error=logs.error,
    )
    second = create_pose_backend_runners(
        params, tmp_path / 'node.py', log_info=logs.info,
        log_warn=logs.warn, log_error=logs.error,
    )
    assert first.megapose.runtime_example.example_dir != second.megapose.runtime_example.example_dir
    assert first.gigapose.runtime_input.runtime_dir != second.gigapose.runtime_input.runtime_dir
    assert not first.megapose.runtime_example.example_dir.is_relative_to(params.megapose.megapose_data_dir)
    assert not first.gigapose.runtime_input.runtime_dir.is_relative_to(params.gigapose.gigapose_data_dir)
    try:
        rgb = np.zeros((2, 2, 3), dtype=np.uint8)
        for runner, value in ((first, 30), (second, 80)):
            runner.megapose.runtime_example.prepare(
                rgb + value, None, False, [1, 0, 0, 0, 1, 0, 0, 0, 1], (0, 0, 2, 2),
            )
            runner.gigapose.runtime_input.prepare(rgb + value, None)
        first_image = first.megapose.runtime_example.example_dir / 'image_rgb.png'
        second_image = second.megapose.runtime_example.example_dir / 'image_rgb.png'
        assert cv2.imread(str(first_image))[0, 0, 0] == 30
        assert cv2.imread(str(second_image))[0, 0, 0] == 80
        env = first.gigapose.worker_client.env_factory()
        private_cache = Path(env['TORCH_HOME']) / 'hub' / 'weights.bin'
        assert private_cache.read_bytes() == b'cached'
        assert not private_cache.is_symlink()
        private_cache.write_bytes(b'updated')
        assert (cache_dir / 'weights.bin').read_bytes() == b'cached'
        private_datasets = first.workspace.gigapose_data_dir / 'datasets'
        assert (private_datasets / 'model.json').resolve() == dataset_dir / 'model.json'
        private_templates = private_datasets / 'templates' / 'franka_cube'
        assert private_templates.resolve().is_relative_to(first.workspace.root)
        assert (private_templates / '000003' / 'image.png').read_bytes() == b'input asset'
        (private_templates / 'preprocessed').mkdir(exist_ok=True)
        (private_templates / 'preprocessed' / '000003.npz').write_bytes(b'private cache')
        assert (template_dir / 'preprocessed' / '000003.npz').read_bytes() == b'shared cache'
        assert first.gigapose.refiner_config.root_dir.is_relative_to(first.workspace.root)
        assert first.megapose.cli_config.env_factory()['MEGAPOSE_DATA_DIR'] == str(
            first.workspace.megapose_data_dir
        )
        assert not (params.megapose.megapose_data_dir / 'examples').exists()
        assert not (params.gigapose.gigapose_data_dir / 'ros_runtime').exists()
        assert not (params.gigapose.gigapose_data_dir / 'tmp').exists()
        first.megapose.stop()
        first.gigapose.stop()
        first.cleanup()
        assert not first.workspace.root.exists()
        assert second_image.is_file()
        assert (model_dir / 'weights.bin').read_bytes() == b'model'
    finally:
        for runner in (first, second):
            runner.megapose.stop()
            runner.gigapose.stop()
            runner.cleanup()
