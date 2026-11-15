"""
Verify the Panda scenario and generic GigaPose checkpoint contract together.

Responsibility:
- Exercise launch-time validation for default and explicit checkpoint paths.
- Keep the scenario launch sentinel aligned with the generic pose backend.

Does not own:
- GigaPose inference, model downloads, or ROS graph startup.
"""

from pathlib import Path
import re
import sys

from launch import LaunchContext
import pytest


_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_PACKAGE_ROOT))

from franka_motion_runtime.bringup.launch_parts.camera.validation import (  # noqa: E402
    _validate_gigapose_setup,
)


def _write_file(path: Path, content: str = 'test') -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)


def _create_context(
    tmp_path: Path,
    *,
    checkpoint_override: str = '',
    create_default_checkpoint: bool = True,
) -> tuple[LaunchContext, Path]:
    megapose_data_dir = tmp_path / 'megapose'
    for run_dir in ('coarse-rgb-906902141', 'refiner-rgb-653307694'):
        _write_file(
            megapose_data_dir
            / 'megapose-models'
            / run_dir
            / 'checkpoint.pth.tar'
        )

    gigapose_repo_dir = tmp_path / 'gigapose-repo'
    _write_file(gigapose_repo_dir / 'configs' / 'test.yaml')

    gigapose_data_dir = tmp_path / 'gigapose-data'
    default_checkpoint = (
        gigapose_data_dir / 'pretrained' / 'gigaPose_v1.ckpt'
    )
    if create_default_checkpoint:
        _write_file(default_checkpoint)
    (gigapose_data_dir / 'datasets' / 'templates' / 'test_dataset' / '000001').mkdir(
        parents=True
    )
    _write_file(
        gigapose_data_dir
        / 'datasets'
        / 'templates'
        / 'test_dataset'
        / 'object_poses'
        / '000001.npy'
    )
    _write_file(
        gigapose_data_dir
        / 'datasets'
        / 'test_dataset'
        / 'models'
        / 'models_info.json'
    )

    context = LaunchContext()
    context.launch_configurations.update(
        {
            'mode': 'gigaPose',
            'megapose_data_dir': str(megapose_data_dir),
            'gigapose_python': sys.executable,
            'gigapose_repo_dir': str(gigapose_repo_dir),
            'gigapose_dinov2_repo_dir': '',
            'gigapose_data_dir': str(gigapose_data_dir),
            'gigapose_dataset_name': 'test_dataset',
            'gigapose_object_id': '1',
            'gigapose_checkpoint_path': checkpoint_override,
        }
    )
    return context, default_checkpoint


def test_empty_checkpoint_uses_existing_data_directory_default(tmp_path: Path):
    context, _ = _create_context(tmp_path)

    assert _validate_gigapose_setup(context) == []


def test_missing_default_checkpoint_reports_resolved_path(tmp_path: Path):
    context, default_checkpoint = _create_context(
        tmp_path,
        create_default_checkpoint=False,
    )

    with pytest.raises(RuntimeError, match=re.escape(str(default_checkpoint))):
        _validate_gigapose_setup(context)


def test_explicit_checkpoint_does_not_fall_back_to_existing_default(
    tmp_path: Path,
):
    explicit_checkpoint = tmp_path / 'explicit' / 'missing.ckpt'
    context, _ = _create_context(
        tmp_path,
        checkpoint_override=str(explicit_checkpoint),
    )

    with pytest.raises(RuntimeError, match=re.escape(str(explicit_checkpoint))):
        _validate_gigapose_setup(context)
