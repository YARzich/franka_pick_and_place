"""
Test mesh inspection, scaling, densification, and material copying.

Responsibility: Preserve OBJ geometry and referenced assets across runtime mesh preparation.
"""

from pathlib import Path
import sys

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.pose.mesh_utils import (  # noqa: E402,I100,I101,I201
    copy_material_assets,
    count_obj_vertices,
    estimate_obj_size_m,
    obj_diameter_mm,
    read_obj_vertices,
    write_dense_scaled_obj,
    write_scaled_obj,
)


def _write_cube_like_obj(path: Path) -> None:
    path.write_text(
        '\n'.join(
            [
                'mtllib cube.mtl',
                'o cube',
                'v 0 0 0',
                'v 1 0 0',
                'v 0 1 0',
                'v 0 0 1',
                'usemtl cube',
                'f 1 2 3',
                'f -4 -3 -1',
            ]
        ) + '\n'
    )


def test_obj_vertex_read_count_scale_and_size(tmp_path: Path):
    source = tmp_path / 'cube.obj'
    scaled = tmp_path / 'scaled.obj'
    _write_cube_like_obj(source)

    assert count_obj_vertices(source) == 4
    np.testing.assert_allclose(
        read_obj_vertices(source),
        np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        ),
    )

    write_scaled_obj(source, scaled, scale=1000.0)
    assert 'v 1000 0 0' in scaled.read_text()
    assert obj_diameter_mm(source, 'm') == np.sqrt(3.0) * 1000.0
    assert estimate_obj_size_m(source, 'm') == (1.0, 1.0, 1.0)
    assert estimate_obj_size_m(source, 'mm') == (0.001, 0.001, 0.001)


def test_dense_obj_writer_preserves_material_lines_and_handles_negative_faces(tmp_path: Path):
    source = tmp_path / 'cube.obj'
    dense = tmp_path / 'dense.obj'
    _write_cube_like_obj(source)

    write_dense_scaled_obj(source, dense, scale=1.0, min_vertices=20, fallback='scaled')
    text = dense.read_text()

    assert 'mtllib cube.mtl' in text
    assert 'usemtl cube' in text
    assert text.count('\nv ') >= 20
    assert text.count('\nf ') >= 10


def test_dense_obj_fallback_can_scale_or_copy(tmp_path: Path):
    source = tmp_path / 'points.obj'
    scaled = tmp_path / 'scaled.obj'
    copied = tmp_path / 'copied.obj'
    source.write_text('v 1 2 3\n')

    write_dense_scaled_obj(source, scaled, scale=2.0, fallback='scaled')
    write_dense_scaled_obj(source, copied, scale=2.0, fallback='copy')

    assert scaled.read_text() == 'v 2 4 6\n'
    assert copied.read_text() == source.read_text()


def test_copy_material_assets_copies_mtl_and_referenced_textures(tmp_path: Path):
    source = tmp_path / 'cube.obj'
    target_dir = tmp_path / 'target'
    target_dir.mkdir()
    source.write_text('mtllib cube.mtl\nv 0 0 0\n')
    (tmp_path / 'cube.mtl').write_text('newmtl cube\nmap_Kd cube.png\n')
    (tmp_path / 'cube.png').write_text('texture')

    copy_material_assets(source, target_dir)

    assert (target_dir / 'cube.mtl').read_text() == 'newmtl cube\nmap_Kd cube.png\n'
    assert (target_dir / 'cube.png').read_text() == 'texture'
