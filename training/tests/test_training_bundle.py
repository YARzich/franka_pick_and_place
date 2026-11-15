"""Responsibility: Verify portable recipe preparation and CLI boundaries without ML runtimes."""

import importlib.util
from pathlib import Path
import shlex
import subprocess
import sys

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
CHECKPOINTS = (
    'cube_yolo26n_isaac_seg_v1.pt',
    'cube_yolo26n_isaac_seg_v2.pt',
    'cube_yolo26n_isaac_seg_v3.pt',
    'cube_yolo26n_isaac_seg_v4c.pt',
    'cube_yolo26n_isaac_v2.pt',
    'cube_yolo26s_isaac_seg_v5s.pt',
)


def load_script(name):
    """Import one tool without adding its directory to global Python paths."""
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('name', (
    'train_cube_yolo',
    'generate_cube_synthetic_dataset',
    'create_cube_screenshot_seg_dataset',
    'augment_yolo_seg_edges',
    'augment_yolo_seg_photometric',
    'augment_yolo_seg_hard_cases',
))
def test_copied_cli_help_needs_only_standard_library(name):
    """Inspecting CLI options must not import OpenCV, NumPy, Torch or Isaac."""
    result = subprocess.run(
        [sys.executable, '-S', str(SCRIPTS / f'{name}.py'), '--help'],
        text=True, capture_output=True, check=False,
    )
    assert result.returncode == 0, result.stderr
    assert 'usage:' in result.stdout


def test_missing_named_weights_fail_before_importing_ultralytics(tmp_path):
    """A model-like basename must not fall through to the upstream downloader."""
    data = tmp_path / 'data.yaml'
    data.write_text('names: {0: cube}\n')
    result = subprocess.run(
        [sys.executable, '-S', str(SCRIPTS / 'train_cube_yolo.py'),
         '--data', str(data), '--weights', 'missing-yolo.pt'],
        cwd=tmp_path, text=True, capture_output=True, check=False,
    )
    assert result.returncode == 1
    assert 'Weights file not found:' in result.stderr
    assert 'Ultralytics is not installed' not in result.stderr
    assert sorted(p.name for p in tmp_path.iterdir()) == ['data.yaml']


def test_training_cli_requires_explicit_weights(tmp_path):
    """No machine-specific or downloadable base model is selected implicitly."""
    result = subprocess.run(
        [sys.executable, '-S', str(SCRIPTS / 'train_cube_yolo.py'), '--data', 'data.yaml'],
        cwd=tmp_path, text=True, capture_output=True, check=False,
    )
    assert result.returncode == 2
    assert '--weights' in result.stderr
    assert list(tmp_path.iterdir()) == []


def test_generator_resolves_sibling_resources_without_isaac_or_output_writes(tmp_path):
    """Source resource defaults work independently of the parent checkout's name."""
    generator = load_script('generate_cube_synthetic_dataset')
    args = generator._build_arg_parser().parse_args(['--output_dir', str(tmp_path / 'data')])
    assert args.install_path == ''
    instance = generator.CubeSyntheticDatasetGenerator(args)
    for name in ('_scene_usd', '_robot_usd', '_camera_sdf', '_gazebo_world_sdf'):
        assert getattr(instance, name).is_file()
    assert not (tmp_path / 'data').exists()


def test_generator_bootstrap_failure_precedes_dataset_creation(tmp_path, monkeypatch):
    """A missing runtime or re-exec must not leave output to break the second launch."""
    generator = load_script('generate_cube_synthetic_dataset')
    output = tmp_path / 'new-dataset'
    monkeypatch.setattr(sys, 'argv', ['generator', '--output_dir', str(output)])

    def unavailable(_args):
        raise RuntimeError('Selected interpreter has no Isaac')

    monkeypatch.setattr(generator, '_prepare_isaac_environment', unavailable)
    with pytest.raises(RuntimeError, match='Selected interpreter'):
        generator.main()
    assert not output.exists()


def test_generator_missing_scene_preserves_existing_dataset(tmp_path):
    """A path typo must fail before --overwrite can remove an earlier dataset."""
    generator = load_script('generate_cube_synthetic_dataset')
    output = tmp_path / 'existing-dataset'
    output.mkdir()
    retained = output / 'retained-image.png'
    retained.write_bytes(b'existing dataset')
    args = generator._build_arg_parser().parse_args([
        '--output_dir', str(output), '--overwrite', '--scene_usd', str(tmp_path / 'missing.usd'),
    ])
    with pytest.raises(FileNotFoundError, match='Scene USD not found'):
        generator.CubeSyntheticDatasetGenerator(args).run()
    assert retained.read_bytes() == b'existing dataset'


@pytest.mark.parametrize('name,writer', (
    ('augment_yolo_seg_photometric', '_write_data_yaml'),
    ('augment_yolo_seg_hard_cases', '_write_manifest'),
    ('create_cube_screenshot_seg_dataset', '_write_manifest'),
    ('generate_cube_synthetic_dataset', None),
))
def test_generated_dataset_manifest_preserves_yaml_sensitive_paths(tmp_path, name, writer):
    """Output paths containing YAML punctuation must retain their exact filename."""
    module = load_script(name)
    output = tmp_path / 'images: marker # value'
    output.mkdir()
    if writer is not None:
        getattr(module, writer)(output)
    else:
        args = module._build_arg_parser().parse_args(['--output_dir', str(output)])
        module.CubeSyntheticDatasetGenerator(args)._write_data_yaml()
    data = yaml.safe_load((output / 'data.yaml').read_text())
    assert data['path'] == str(output)


@pytest.mark.parametrize('checkpoint', CHECKPOINTS)
def test_prepare_preserves_saved_settings_and_binds_paths(tmp_path, checkpoint):
    """Every archived run produces a complete native CLI config without saved output metadata."""
    tool = load_script('prepare_recipe')
    destination = tmp_path / 'new configuration'
    result = tool.prepare_recipe(
        checkpoint,
        datasets_root=tmp_path / 'datasets',
        weights_root=tmp_path / 'weights',
        runs_root=tmp_path / 'runs',
        output_dir=destination,
    )
    prepared = yaml.safe_load(result.read_text())
    original = yaml.safe_load((ROOT / 'recipes' / f'{Path(checkpoint).stem}.yaml').read_text())
    assert len(original) == 109
    assert prepared['data'] == str(destination / 'data.yaml')
    assert prepared['project'] == str(tmp_path / 'runs')
    assert prepared['model'].startswith(str(tmp_path / 'weights') + '/')
    assert 'save_dir' not in prepared
    for key, value in original.items():
        if key not in {'data', 'model', 'project', 'save_dir'}:
            assert prepared[key] == value, key
    assert '${' not in result.read_text()
    dataset = yaml.safe_load((destination / 'data.yaml').read_text())
    assert dataset['names'] == {0: 'cube'}
    assert dataset['path'].startswith(str(tmp_path / 'datasets') + '/')
    assert '${' not in (destination / 'data.yaml').read_text()
    # Inspection does not create datasets, input weights, or training results.
    assert not (tmp_path / 'datasets').exists()
    assert not (tmp_path / 'weights').exists()
    assert not (tmp_path / 'runs').exists()


def test_prepare_checks_all_inputs_before_creating_output(tmp_path):
    """The compound v5 recipe must report a missing member rather than silently dropping it."""
    tool = load_script('prepare_recipe')
    kwargs = dict(
        datasets_root=tmp_path / 'datasets', weights_root=tmp_path / 'weights',
        runs_root=tmp_path / 'runs', output_dir=tmp_path / 'prepared', check_inputs=True,
    )
    with pytest.raises(FileNotFoundError) as error:
        tool.prepare_recipe(CHECKPOINTS[-1], **kwargs)
    assert 'cube_problem2_hard_aug_seg_v5s/images/train' in str(error.value)
    assert 'yolo26s-seg.pt' in str(error.value)
    assert not kwargs['output_dir'].exists()


def test_prepare_accepts_present_inputs_without_loading_them(tmp_path):
    """Preflight checks paths only and never opens a serialized model."""
    tool = load_script('prepare_recipe')
    weights = tmp_path / 'weights'
    weights.mkdir()
    (weights / 'yolo26n-seg.pt').write_bytes(b'not a torch checkpoint')
    datasets = tmp_path / 'datasets'
    for split in ('train', 'val'):
        (datasets / 'cube_synthetic_yolo_seg_v1/images' / split).mkdir(parents=True)
    result = tool.prepare_recipe(
        CHECKPOINTS[0], datasets_root=datasets, weights_root=weights,
        runs_root=tmp_path / 'runs', output_dir=tmp_path / 'prepared', check_inputs=True,
    )
    assert result.is_file()


def test_prepare_does_not_overwrite_existing_files(tmp_path):
    """Repeated inspection must not destroy a hand-edited prepared config."""
    tool = load_script('prepare_recipe')
    output = tmp_path / 'prepared'
    output.mkdir()
    saved = output / 'train.yaml'
    saved.write_text('hand edited\n')
    with pytest.raises(FileExistsError):
        tool.prepare_recipe(
            CHECKPOINTS[0], datasets_root=tmp_path, weights_root=tmp_path,
            runs_root=tmp_path, output_dir=output,
        )
    assert saved.read_text() == 'hand edited\n'
    assert not (output / 'data.yaml').exists()


def test_unknown_placeholders_and_checkpoint_fail_before_writes(tmp_path):
    """Typos in a recipe or run selection must remain visible errors."""
    tool = load_script('prepare_recipe')
    with pytest.raises(KeyError):
        tool.bind_paths({'model': '${UNKNOWN_ROOT}/model.pt'}, {})
    with pytest.raises(ValueError, match='Unknown checkpoint'):
        tool.prepare_recipe(
            'unknown.pt', datasets_root=tmp_path, weights_root=tmp_path,
            runs_root=tmp_path, output_dir=tmp_path / 'prepared',
        )
    assert list(tmp_path.iterdir()) == []


def test_printed_cli_command_preserves_paths_with_spaces(tmp_path):
    """The command is an explicit suggestion; only YAML files are created."""
    output = tmp_path / 'config with spaces'
    result = subprocess.run(
        [sys.executable, str(SCRIPTS / 'prepare_recipe.py'),
         '--checkpoint', CHECKPOINTS[0], '--datasets-root', str(tmp_path / 'data'),
         '--weights-root', str(tmp_path / 'weights'), '--runs-root', str(tmp_path / 'runs'),
         '--output-dir', str(output)], text=True, capture_output=True, check=False,
    )
    assert result.returncode == 0, result.stderr
    assert shlex.split(result.stdout.splitlines()[-1]) == [
        'yolo', 'train', f'cfg={output / "train.yaml"}',
    ]
    assert sorted(p.name for p in tmp_path.iterdir()) == ['config with spaces']
