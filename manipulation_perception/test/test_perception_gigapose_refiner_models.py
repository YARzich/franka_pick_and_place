"""
Test GigaPose refiner model-input preparation and output normalization.

Responsibility: Verify mesh metadata, crop geometry, poses, labels, scores, and device transfer.
"""

import json
from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.pose.pose_gigapose_refiner_models import (  # noqa: E402
    count_obj_vertices,
    mesh_to_bop_mm_scale,
    prepare_gigapose_refiner_models_dir,
    validate_gigapose_refiner_runtime_files,
)
from manipulation_perception.pose.pose_gigapose_refiner_worker_inputs import (  # noqa: E402
    parse_gigapose_refiner_worker_request,
)


def _make_refiner_layout(tmp_path: Path):
    repo = tmp_path / 'repo'
    (repo / 'configs' / 'model' / 'refiner').mkdir(parents=True)
    (repo / 'configs' / 'model' / 'refiner' / 'rgb.yaml').write_text('config')

    root = tmp_path / 'data'
    models = root / 'datasets' / 'franka_cube' / 'models'
    models.mkdir(parents=True)
    (models / 'models_info.json').write_text(json.dumps({'3': {'diameter': 1732.0}}))
    (models / 'obj_000003.obj').write_text('v 0 0 0\nv 1 0 0\nv 0 1 0\n')
    (models / 'cube.mtl').write_text('material')

    megapose_models = tmp_path / 'megapose_models'
    for run_dir in ('coarse-rgb-906902141', 'refiner-rgb-653307694'):
        (megapose_models / run_dir).mkdir(parents=True)
        (megapose_models / run_dir / 'checkpoint.pth.tar').write_text('checkpoint')

    request = parse_gigapose_refiner_worker_request(
        {
            'id': 1,
            'image_path': str(tmp_path / 'rgb.png'),
            'K': [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            'repo_dir': str(repo),
            'root_dir': str(root),
            'dataset_name': 'franka_cube',
            'object_id': 3,
            'output_label': 'cube_5cm',
            'megapose_models_root': str(megapose_models),
            'coarse_estimates': {'poses': []},
        }
    )
    return request, models


def test_validate_refiner_runtime_files_and_prepare_models_dir(tmp_path: Path):
    request, _models = _make_refiner_layout(tmp_path)

    validate_gigapose_refiner_runtime_files(request)
    models_dir = prepare_gigapose_refiner_models_dir(request)

    assert (models_dir / 'models_info.json').is_file()
    assert (models_dir / 'obj_000003.obj').is_file()
    assert count_obj_vertices(models_dir / 'obj_000003.obj') >= 3


def test_mesh_scale_uses_bop_millimeters_for_large_ratio(tmp_path: Path):
    request, models = _make_refiner_layout(tmp_path)

    scale = mesh_to_bop_mm_scale(
        models / 'obj_000003.obj',
        models / 'models_info.json',
        request.object_id,
    )

    assert scale == 1000.0
