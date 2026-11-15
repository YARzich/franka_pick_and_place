"""
Test ROS disparity decoding and external stereo geometry selection.

Responsibility: Verify endianness, row padding, focal length, baseline, and invalid samples.
"""

from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.common.stereo_disparity import (  # noqa: E402,I100,I101,I201
    disparity_image_to_array,
    resolve_external_disparity_geometry,
)


def _disparity_image(
    *,
    height: int,
    width: int,
    step: int,
    data: bytes,
    is_bigendian: bool = False,
    encoding: str = '32FC1',
) -> SimpleNamespace:
    return SimpleNamespace(
        height=height,
        width=width,
        step=step,
        data=data,
        is_bigendian=is_bigendian,
        encoding=encoding,
    )


def _header(*, sec: int = 7, nanosec: int = 11, frame_id: str = 'camera_optical'):
    return SimpleNamespace(
        stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
        frame_id=frame_id,
    )


def _camera_info(
    *,
    width: int = 4,
    height: int = 3,
    frame_id: str = 'camera_optical',
    sec: int = 7,
    fx: float = 100.0,
):
    return SimpleNamespace(
        header=_header(sec=sec, frame_id=frame_id),
        width=width,
        height=height,
        k=[fx, 0.0, 1.5, 0.0, 110.0, 1.0, 0.0, 0.0, 1.0],
        p=[],
    )


def _disparity_message(
    *,
    width: int = 4,
    height: int = 3,
    frame_id: str = 'camera_optical',
    image_frame_id: str | None = None,
    sec: int = 7,
    image_sec: int | None = None,
    focal_length: float = 200.0,
    baseline: float = 0.07,
):
    return SimpleNamespace(
        header=_header(sec=sec, frame_id=frame_id),
        image=SimpleNamespace(
            header=_header(
                sec=sec if image_sec is None else image_sec,
                frame_id=frame_id if image_frame_id is None else image_frame_id,
            ),
            width=width,
            height=height,
        ),
        f=focal_length,
        t=baseline,
    )


def test_disparity_image_to_array_decodes_little_endian_rows():
    expected = np.array([[1.25, 2.5], [3.75, 4.0]], dtype=np.float32)
    image = _disparity_image(
        height=2,
        width=2,
        step=8,
        data=expected.astype('<f4').tobytes(),
    )

    actual = disparity_image_to_array(image)

    np.testing.assert_array_equal(actual, expected)
    assert actual.dtype == np.float32


def test_disparity_image_to_array_decodes_big_endian_rows():
    expected = np.array([[0.5, 10.0], [-2.0, 3.25]], dtype=np.float32)
    image = _disparity_image(
        height=2,
        width=2,
        step=8,
        data=expected.astype('>f4').tobytes(),
        is_bigendian=True,
    )

    np.testing.assert_array_equal(disparity_image_to_array(image), expected)


def test_disparity_image_to_array_ignores_row_padding():
    first_row = np.array([1.0, 2.0, 99.0], dtype='<f4').tobytes()
    second_row = np.array([3.0, 4.0, 99.0], dtype='<f4').tobytes()
    image = _disparity_image(
        height=2,
        width=2,
        step=12,
        data=first_row + second_row,
    )

    np.testing.assert_array_equal(
        disparity_image_to_array(image),
        np.array([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32),
    )


def test_disparity_image_to_array_rejects_truncated_data():
    image = _disparity_image(height=2, width=2, step=8, data=b'\x00' * 15)

    with pytest.raises(ValueError, match=r'data короче'):
        disparity_image_to_array(image)


def test_disparity_image_to_array_rejects_invalid_encoding():
    image = _disparity_image(
        height=1,
        width=2,
        step=8,
        data=b'\x00' * 8,
        encoding='16UC1',
    )

    with pytest.raises(ValueError, match=r"encoding='32FC1'"):
        disparity_image_to_array(image)


def test_external_geometry_uses_sample_focal_length_for_depth_and_cloud_projection():
    geometry = resolve_external_disparity_geometry(
        _disparity_message(focal_length=240.0, baseline=0.08),
        _camera_info(fx=100.0),
        baseline_fallback_m=0.06,
    )

    assert geometry.intrinsics == (240.0, 110.0, 1.5, 1.0)
    assert geometry.baseline_m == 0.08


def test_external_geometry_allows_valid_fallback_only_for_explicit_zero():
    geometry = resolve_external_disparity_geometry(
        _disparity_message(focal_length=0.0, baseline=0.0),
        _camera_info(fx=125.0),
        baseline_fallback_m=0.065,
    )

    assert geometry.intrinsics[0] == 125.0
    assert geometry.baseline_m == 0.065


@pytest.mark.parametrize('field,value', [('f', -1.0), ('f', np.nan), ('t', -0.1), ('t', np.inf)])
def test_external_geometry_rejects_negative_or_nonfinite_sample_values(field, value):
    values = {'focal_length': 200.0, 'baseline': 0.07}
    values['focal_length' if field == 'f' else 'baseline'] = value

    with pytest.raises(ValueError, match=rf'DisparityImage\.{field}'):
        resolve_external_disparity_geometry(
            _disparity_message(**values),
            _camera_info(),
            baseline_fallback_m=0.06,
        )


@pytest.mark.parametrize(
    'message,camera_info',
    [
        (_disparity_message(frame_id='other_optical'), _camera_info()),
        (_disparity_message(image_frame_id='other_optical'), _camera_info()),
        (_disparity_message(image_sec=8), _camera_info()),
        (_disparity_message(), _camera_info(sec=8)),
    ],
)
def test_external_geometry_rejects_header_mismatch(message, camera_info):
    with pytest.raises(ValueError, match=r'header.*точно совпадать'):
        resolve_external_disparity_geometry(message, camera_info, baseline_fallback_m=0.06)


def test_external_geometry_rejects_camera_info_resolution_mismatch():
    with pytest.raises(ValueError, match=r'resolution не совпадает'):
        resolve_external_disparity_geometry(
            _disparity_message(width=4, height=3),
            _camera_info(width=8, height=6),
            baseline_fallback_m=0.06,
        )


def test_external_geometry_rejects_invalid_zero_fallbacks():
    with pytest.raises(ValueError, match=r'CameraInfo не содержит'):
        resolve_external_disparity_geometry(
            _disparity_message(focal_length=0.0),
            _camera_info(fx=float('nan')),
            baseline_fallback_m=0.06,
        )

    with pytest.raises(ValueError, match=r'DisparityImage\.t=0'):
        resolve_external_disparity_geometry(
            _disparity_message(baseline=0.0),
            _camera_info(),
            baseline_fallback_m=0.0,
        )
