"""Responsibility: Require an explicit offline dataset destination before writing files."""

import importlib.util
from pathlib import Path
import sys

import pytest


_SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/perception/prepare_gigapose_cube_dataset.py'


def _cli():
    spec = importlib.util.spec_from_file_location('dataset_cli', _SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_missing_destination_fails_before_writing(monkeypatch, capsys):
    monkeypatch.delenv('FRANKA_GPU_GIGAPOSE_DATA_DIR', raising=False)
    monkeypatch.setattr(sys, 'argv', [str(_SCRIPT)])
    with pytest.raises(SystemExit) as error:
        _cli().main()
    assert error.value.code == 2
    assert '--gigapose-data-dir' in capsys.readouterr().err


@pytest.mark.parametrize('via_environment', [False, True])
def test_explicit_destination_creates_dataset(tmp_path, monkeypatch, via_environment):
    mesh = tmp_path / 'cube.obj'
    mesh.write_text('v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n')
    destination = tmp_path / 'output'
    args = [str(_SCRIPT), '--mesh', str(mesh)]
    if via_environment:
        monkeypatch.setenv('FRANKA_GPU_GIGAPOSE_DATA_DIR', str(destination))
    else:
        monkeypatch.delenv('FRANKA_GPU_GIGAPOSE_DATA_DIR', raising=False)
        args += ['--gigapose-data-dir', str(destination)]
    monkeypatch.setattr(sys, 'argv', args)
    assert _cli().main() == 0
    assert (destination / 'datasets/franka_cube/models/models_info.json').is_file()
