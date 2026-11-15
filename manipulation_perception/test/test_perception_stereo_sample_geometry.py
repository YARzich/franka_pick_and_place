"""
Test exact local-stereo sample geometry validation.

Responsibility: Reject mismatched stamps, frames, resolutions, and calibration
before local SGBM can produce depth or point clouds.
"""

from types import SimpleNamespace

import pytest

from manipulation_perception.common.stereo_sample_geometry import (
    resolve_local_stereo_geometry,
)


def _header(*, sec=7, nanosec=11, frame_id='left_optical'):
    return SimpleNamespace(
        stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
        frame_id=frame_id,
    )


def _image(*, frame_id='left_optical', sec=7, width=4, height=3):
    return SimpleNamespace(
        header=_header(sec=sec, frame_id=frame_id),
        width=width,
        height=height,
    )


def _camera_info(*, frame_id='left_optical', sec=7, width=4, height=3):
    return SimpleNamespace(
        header=_header(sec=sec, frame_id=frame_id),
        width=width,
        height=height,
        k=[100.0, 0.0, 1.5, 0.0, 110.0, 1.0, 0.0, 0.0, 1.0],
        p=[],
    )


def _resolve(left=None, right=None, camera=None):
    return resolve_local_stereo_geometry(
        left or _image(),
        right or _image(frame_id='right_optical'),
        camera or _camera_info(),
        expected_left_frame='left_optical',
        expected_right_frame='right_optical',
    )


def test_local_geometry_returns_one_sample_scoped_intrinsics_snapshot():
    assert _resolve().intrinsics == (100.0, 110.0, 1.5, 1.0)


@pytest.mark.parametrize(
    'left,right,camera',
    [
        (_image(frame_id='other'), None, None),
        (None, _image(frame_id='other'), None),
        (None, _image(frame_id='right_optical', sec=8), None),
        (None, None, _camera_info(sec=8)),
        (None, None, _camera_info(frame_id='other')),
    ],
)
def test_local_geometry_rejects_frame_or_stamp_mismatch(left, right, camera):
    with pytest.raises(ValueError):
        _resolve(left, right, camera)


@pytest.mark.parametrize(
    'right,camera',
    [
        (_image(frame_id='right_optical', width=5), None),
        (None, _camera_info(width=5)),
    ],
)
def test_local_geometry_rejects_resolution_mismatch(right, camera):
    with pytest.raises(ValueError, match='resolutions'):
        _resolve(right=right, camera=camera)


def test_local_geometry_rejects_invalid_camera_calibration():
    camera = _camera_info()
    camera.k[0] = float('nan')
    with pytest.raises(ValueError, match='calibration'):
        _resolve(camera=camera)
