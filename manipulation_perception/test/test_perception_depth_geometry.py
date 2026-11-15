"""
Test pure stereo depth and camera-projection geometry.

Responsibility: Verify intrinsics fallback, invalid-depth filtering, projection, and ROS encoding.
"""

from pathlib import Path
import sys

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.common.depth_geometry import (  # noqa: E402,I100,I101,I201
    camera_intrinsics_from_matrices,
    clean_depth_for_pose,
    depth_to_mm_u16,
    disparity_to_depth,
    project_depth_to_points,
)


def test_camera_intrinsics_prefers_k_and_falls_back_to_p():
    assert camera_intrinsics_from_matrices(
        [100.0, 0.0, 50.0, 0.0, 110.0, 60.0, 0.0, 0.0, 1.0],
        [],
    ) == (100.0, 110.0, 50.0, 60.0)

    assert camera_intrinsics_from_matrices(
        [0.0] * 9,
        [120.0, 0.0, 61.0, 0.0, 0.0, 130.0, 71.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    ) == (120.0, 130.0, 61.0, 71.0)

    assert camera_intrinsics_from_matrices([], []) is None


def test_camera_intrinsics_rejects_nonfinite_k_and_uses_valid_projection_fallback():
    assert camera_intrinsics_from_matrices(
        [float('nan'), 0.0, 50.0, 0.0, 110.0, 60.0, 0.0, 0.0, 1.0],
        [120.0, 0.0, 61.0, 0.0, 0.0, 130.0, 71.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    ) == (120.0, 130.0, 61.0, 71.0)

    assert camera_intrinsics_from_matrices(
        [100.0, 0.0, float('inf'), 0.0, 110.0, 60.0, 0.0, 0.0, 1.0],
        [],
    ) is None


def test_disparity_to_depth_filters_invalid_values_and_depth_range():
    disparity = np.array([[0.0, 1.0, 2.0], [np.nan, 4.0, 100.0]], dtype=np.float32)

    depth = disparity_to_depth(
        disparity,
        fx=100.0,
        baseline_m=0.1,
        min_valid_disparity_px=0.5,
        min_depth_m=0.2,
        max_depth_m=6.0,
    )

    assert np.isnan(depth[0, 0])
    assert np.isnan(depth[0, 1])
    assert depth[0, 2] == 5.0
    assert np.isnan(depth[1, 0])
    assert depth[1, 1] == 2.5
    assert np.isnan(depth[1, 2])


def test_disparity_to_depth_returns_nan_map_without_fx():
    depth = disparity_to_depth(
        np.ones((2, 2), dtype=np.float32),
        fx=None,
        baseline_m=0.1,
        min_valid_disparity_px=0.1,
        min_depth_m=0.0,
        max_depth_m=10.0,
    )

    assert np.isnan(depth).all()


def test_project_depth_to_points_respects_stride_and_intrinsics():
    depth = np.array(
        [
            [1.0, np.nan, 2.0],
            [0.0, 3.0, 4.0],
            [5.0, 6.0, 7.0],
        ],
        dtype=np.float32,
    )

    points = project_depth_to_points(depth, fx=2.0, fy=4.0, cx=1.0, cy=1.0, stride=2)

    assert points is not None
    np.testing.assert_allclose(
        points,
        np.array(
            [
                [-0.5, -0.25, 1.0],
                [1.0, -0.5, 2.0],
                [-2.5, 1.25, 5.0],
                [3.5, 1.75, 7.0],
            ],
            dtype=np.float32,
        ),
    )


def test_depth_clean_and_uint16_conversion_are_ros_free():
    dirty = np.array([[1.25, np.nan], [np.inf, -np.inf]], dtype=np.float32)

    clean = clean_depth_for_pose(dirty)
    np.testing.assert_array_equal(clean, np.array([[1.25, 0.0], [0.0, 0.0]], dtype=np.float32))

    depth_mm = depth_to_mm_u16(dirty, (2, 2))
    np.testing.assert_array_equal(depth_mm, np.array([[1250, 0], [0, 0]], dtype=np.uint16))

    uint16_depth = np.array([[10, 20]], dtype=np.uint16)
    assert depth_to_mm_u16(uint16_depth, (1, 2)) is uint16_depth
