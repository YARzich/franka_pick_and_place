"""
Test creation and cleanup of runtime files for pose-estimation backends.

Responsibility: Preserve JSON schemas, image and mesh inputs, masks, and stale-output removal.
"""

import json
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.pose.pose_runtime_files import (  # noqa: E402,I100,I101,I201
    GigaPoseRuntimeInput,
    MegaPoseRuntimeExample,
    megapose_camera_data,
    megapose_object_data,
    megapose_output_pose_paths,
)


K_VALUES = [100.0, 0.0, 50.0, 0.0, 110.0, 60.0, 0.0, 0.0, 1.0]


def test_megapose_runtime_json_builders_keep_existing_shape_contract(tmp_path: Path):
    example_dir = tmp_path / 'example'

    assert megapose_camera_data(K_VALUES, (4, 6)) == {
        'K': [[100.0, 0.0, 50.0], [0.0, 110.0, 60.0], [0.0, 0.0, 1.0]],
        'resolution': [4, 6],
    }
    assert megapose_object_data('cube_5cm', (1, 2, 3, 4)) == [
        {'label': 'cube_5cm', 'bbox_modal': [1, 2, 3, 4]},
    ]
    assert megapose_output_pose_paths(example_dir) == [
        example_dir / 'outputs' / 'object_data.json',
        example_dir / 'outputs' / 'object_data_inf.json',
    ]


def test_megapose_runtime_example_writes_inputs_mesh_and_removes_stale_outputs(tmp_path: Path):
    mesh = tmp_path / 'cube.obj'
    mesh.write_text('v 0 0 0\nv 0.05 0 0\nv 0 0.05 0\nf 1 2 3\n')
    example_dir = tmp_path / 'example'
    stale_output = example_dir / 'outputs' / 'object_data.json'
    stale_output.parent.mkdir(parents=True)
    stale_output.write_text('[{"stale": true}]')

    writer = MegaPoseRuntimeExample(example_dir, 'cube_5cm', mesh, 'm')
    rgb = np.zeros((2, 3, 3), dtype=np.uint8)
    rgb[:, :, 0] = 255
    depth = np.full((2, 3), 0.5, dtype=np.float32)

    writer.prepare(rgb, depth, True, K_VALUES, (1, 1, 2, 2))

    assert not stale_output.exists()
    assert (example_dir / 'image_rgb.png').is_file()
    assert (example_dir / 'image_depth.png').is_file()
    assert (example_dir / 'meshes' / 'cube_5cm' / 'cube.obj').is_file()
    assert json.loads((example_dir / 'camera_data.json').read_text())['resolution'] == [2, 3]
    object_data = json.loads(
        (example_dir / 'inputs' / 'object_data.json').read_text()
    )
    assert object_data[0]['bbox_modal'] == [1, 1, 2, 2]

    writer.prepare(rgb, None, False, K_VALUES, (1, 1, 2, 2))
    assert not (example_dir / 'image_depth.png').exists()


def test_gigapose_runtime_input_writes_rgb_mask_and_removes_stale_mask(tmp_path: Path):
    writer = GigaPoseRuntimeInput(tmp_path / 'runtime')
    rgb = np.zeros((2, 2, 3), dtype=np.uint8)
    mask = np.array([[0, 1], [2, 0]], dtype=np.uint8)

    image_path, mask_path = writer.prepare(rgb, mask)

    assert image_path.is_file()
    assert mask_path is not None and mask_path.is_file()
    mask_image = cv2.imread(str(mask_path), cv2.IMREAD_UNCHANGED)
    assert sorted(np.unique(mask_image).tolist()) == [0, 255]

    image_path, mask_path = writer.prepare(rgb, None)
    assert image_path.is_file()
    assert mask_path is None
    assert not (tmp_path / 'runtime' / 'image_mask.png').exists()


def test_runtime_writer_removes_stale_image_and_raises_when_opencv_fails(
    tmp_path: Path,
    monkeypatch,
):
    runtime_dir = tmp_path / 'runtime'
    runtime_dir.mkdir()
    stale_image = runtime_dir / 'image_rgb.png'
    stale_image.write_bytes(b'stale')
    monkeypatch.setattr(cv2, 'imwrite', lambda *_args, **_kwargs: False)

    with pytest.raises(OSError, match='failed to write runtime image'):
        GigaPoseRuntimeInput(runtime_dir).prepare(
            np.zeros((2, 2, 3), dtype=np.uint8),
            None,
        )

    assert not stale_image.exists()
