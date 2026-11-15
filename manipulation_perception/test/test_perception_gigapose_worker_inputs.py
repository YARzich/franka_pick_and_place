"""
Test GigaPose coarse-worker request and runtime-file preparation.

Responsibility: Verify defaults, cache identity, assets, half-open boxes, and optional masks.
"""

from pathlib import Path
import sys

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.pose.pose_gigapose_worker_inputs import (  # noqa: E402
    clamp_gigapose_bbox,
    gigapose_backend_cache_key,
    load_gigapose_mask,
    parse_gigapose_worker_request,
    validate_gigapose_runtime_files,
)


def _base_request(tmp_path: Path) -> dict:
    return {
        'id': 5,
        'image_path': str(tmp_path / 'rgb.png'),
        'mask_path': '',
        'bbox': [1, 2, 3, 4],
        'K': [[100.0, 0.0, 50.0], [0.0, 100.0, 40.0], [0.0, 0.0, 1.0]],
        'repo_dir': str(tmp_path / 'repo'),
        'root_dir': str(tmp_path / 'data'),
        'dataset_name': 'franka_cube',
        'object_id': 7,
        'checkpoint_path': str(tmp_path / 'checkpoint.ckpt'),
    }


def test_parse_gigapose_worker_request_clamps_defaults_and_paths(tmp_path: Path):
    request = parse_gigapose_worker_request(
        {
            **_base_request(tmp_path),
            'dinov2_repo_dir': str(tmp_path / 'dinov2'),
            'template_count': 0,
            'max_num_dets_per_forward': 0,
            'num_hypotheses': 0,
            'translation_scale': 0.002,
        }
    )

    assert request.request_id == 5
    assert request.repo_dir == tmp_path / 'repo'
    assert request.root_dir == tmp_path / 'data'
    assert request.dinov2_repo_dir == tmp_path / 'dinov2'
    assert request.template_count == 1
    assert request.max_num_dets_per_forward == 1
    assert request.num_hypotheses == 1
    assert request.translation_scale == 0.002
    assert request.bbox == (1, 2, 3, 4)
    assert request.camera_matrix.shape == (3, 3)


def test_gigapose_cache_key_preserves_current_runtime_fields(tmp_path: Path):
    first = parse_gigapose_worker_request({**_base_request(tmp_path), 'num_hypotheses': 3})
    second = parse_gigapose_worker_request({**_base_request(tmp_path), 'num_hypotheses': 9})
    third = parse_gigapose_worker_request({**_base_request(tmp_path), 'translation_scale': 0.01})

    assert gigapose_backend_cache_key(first) == gigapose_backend_cache_key(second)
    assert gigapose_backend_cache_key(first) != gigapose_backend_cache_key(third)


def test_validate_gigapose_runtime_files_checks_repo_templates_and_models_info(tmp_path: Path):
    request = parse_gigapose_worker_request(_base_request(tmp_path))

    with pytest.raises(FileNotFoundError, match='repo'):
        validate_gigapose_runtime_files(request)

    (request.repo_dir / 'configs').mkdir(parents=True)
    (request.repo_dir / 'configs' / 'test.yaml').write_text('config')
    request.checkpoint_path.write_text('checkpoint')

    template_root = request.root_dir / 'datasets' / 'templates' / request.dataset_name
    (template_root / f'{request.object_id:06d}').mkdir(parents=True)
    (template_root / 'object_poses').mkdir()
    (template_root / 'object_poses' / f'{request.object_id:06d}.npy').write_text('poses')

    with pytest.raises(FileNotFoundError, match='models_info'):
        validate_gigapose_runtime_files(request)

    models_dir = request.root_dir / 'datasets' / request.dataset_name / 'models'
    models_dir.mkdir(parents=True)
    (models_dir / 'models_info.json').write_text('{}')

    validate_gigapose_runtime_files(request)


def test_clamp_bbox_preserves_half_open_image_boundary():
    assert clamp_gigapose_bbox((-5, -4, 100, 90), width=10, height=8) == (0, 0, 10, 8)
    assert clamp_gigapose_bbox((9, 7, 9, 7), width=10, height=8) == (9, 7, 10, 8)


def test_load_mask_returns_none_for_missing_and_resizes_existing_mask(tmp_path: Path):
    cv2 = pytest.importorskip('cv2')

    assert load_gigapose_mask(tmp_path / 'missing.png', (4, 5)) is None

    mask_path = tmp_path / 'mask.png'
    cv2.imwrite(str(mask_path), np.array([[0, 255]], dtype=np.uint8))

    mask = load_gigapose_mask(mask_path, (4, 5))

    assert mask.shape == (4, 5)
    assert set(np.unique(mask)).issubset({0.0, 1.0})
    assert np.any(mask > 0.5)
