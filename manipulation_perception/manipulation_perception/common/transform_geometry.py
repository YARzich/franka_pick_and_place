"""
Pure rigid-transform math for unorganized XYZ point arrays.

Responsibility:
    Convert normalized quaternions to rotation matrices and apply explicit rigid
    transforms to metric XYZ arrays.

Does not own:
    TF lookup, frame-name validation, or ROS message conversion.
"""

from __future__ import annotations

import numpy as np


def quat_to_rot_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """
    Return a float32 rotation matrix for a unit `(x, y, z, w)` quaternion.

    The quaternion is dimensionless and must already be normalized. The matrix
    rotates column vectors from the quaternion's source frame into its target
    frame; no ROS TF lookup or frame-name validation is performed here.
    """
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float32,
    )


def apply_transform(points: np.ndarray, transform_matrix: np.ndarray) -> np.ndarray:
    """
    Apply `T_target_source` to source-frame XYZ points measured in metres.

    `points` must have shape `(N, 3)`, and `transform_matrix` must be a valid
    homogeneous transform with shape `(4, 4)`. The result has shape `(N, 3)`
    in the target frame. Empty input is returned unchanged.
    """
    if points.size == 0:
        return points
    homogeneous = np.concatenate(
        [points, np.ones((points.shape[0], 1), dtype=np.float32)], axis=1
    )
    transformed = (transform_matrix @ homogeneous.T).T
    return transformed[:, :3]


__all__ = ["apply_transform", "quat_to_rot_matrix"]
