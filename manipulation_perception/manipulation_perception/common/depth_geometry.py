"""
Depth and stereo geometry helpers without ROS graph ownership.

Responsibility:
- Extract camera intrinsics from CameraInfo matrices.
- Convert disparity/depth arrays and project depth to 3D points.
- Clean or convert depth arrays for pose-estimation backends.

Does not own:
- CameraInfo subscriptions or Image publication.
- TF lookup or workspace cropping.
- Stereo matcher state.
"""

from __future__ import annotations

import math

import cv2
import numpy as np


def camera_intrinsics_from_matrices(
    k_values,
    p_values,
) -> tuple[float, float, float, float] | None:
    """
    Return ``fx, fy, cx, cy`` in pixels from row-major K/P matrices.

    K is preferred. P supplies a rectified-camera fallback when K has no complete
    finite calibration. ``None`` means neither matrix defines a usable pinhole
    model with positive focal lengths.
    """
    def candidate(values, indices) -> tuple[float, float, float, float] | None:
        if len(values) <= max(indices):
            return None
        intrinsics = tuple(float(values[index]) for index in indices)
        if not all(math.isfinite(value) for value in intrinsics):
            return None
        if intrinsics[0] <= 0.0 or intrinsics[1] <= 0.0:
            return None
        return intrinsics

    return candidate(k_values, (0, 4, 2, 5)) or candidate(p_values, (0, 5, 2, 6))


def disparity_to_depth(
    disparity: np.ndarray,
    fx: float | None,
    baseline_m: float,
    min_valid_disparity_px: float,
    min_depth_m: float,
    max_depth_m: float,
) -> np.ndarray:
    """
    Convert pixel disparity to metric depth and mark invalid samples NaN.

    ``baseline_m`` and depth bounds are metres; disparity values and the
    validity threshold are pixels. The result remains aligned with the input
    optical image.
    """
    disparity_values = np.asarray(disparity, dtype=np.float32)
    filtered_disparity = disparity_values.copy()
    filtered_disparity[filtered_disparity <= min_valid_disparity_px] = np.nan

    depth = np.full_like(filtered_disparity, np.nan, dtype=np.float32)
    if fx is None or fx <= 0.0:
        return depth

    valid = np.isfinite(filtered_disparity)
    depth[valid] = (float(fx) * float(baseline_m)) / filtered_disparity[valid]
    valid_depth = np.isfinite(depth)
    if min_depth_m > 0.0:
        valid_depth &= depth >= float(min_depth_m)
    valid_depth &= depth <= float(max_depth_m)
    depth[~valid_depth] = np.nan
    return depth


def project_depth_to_points(
    depth: np.ndarray,
    fx: float | None,
    fy: float | None,
    cx: float | None,
    cy: float | None,
    stride: int,
) -> np.ndarray | None:
    """
    Project metric depth into the source camera optical frame.

    Returned rows are XYZ metres using REP-103 optical axes: x right, y down,
    z forward. ``None`` denotes missing intrinsics; an empty array denotes a
    valid camera model with no finite positive samples.
    """
    if fx is None or fy is None or cx is None or cy is None or fx <= 0.0 or fy <= 0.0:
        return None

    sample_stride = max(1, int(stride))
    sampled_depth = np.asarray(depth)[::sample_stride, ::sample_stride]
    valid = np.isfinite(sampled_depth) & (sampled_depth > 0.0)
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float32)

    v_coords, u_coords = np.indices(sampled_depth.shape, dtype=np.float32)
    u = u_coords * sample_stride
    v = v_coords * sample_stride
    z = sampled_depth
    x = (u - float(cx)) * z / float(fx)
    y = (v - float(cy)) * z / float(fy)
    return np.stack((x[valid], y[valid], z[valid]), axis=1).astype(np.float32)


def clean_depth_for_pose(depth: np.ndarray) -> np.ndarray:
    """Return float32 depth with non-finite samples replaced by zero metres."""
    return np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0).astype(np.float32, copy=False)


def depth_to_mm_u16(depth: np.ndarray, target_shape: tuple[int, int]) -> np.ndarray:
    """
    Resize metric depth with nearest-neighbour sampling and encode millimetres.

    Existing ``uint16`` input is already interpreted as millimetres. Invalid or
    negative float samples become zero; values saturate at the uint16 limit.
    """
    depth_array = np.asarray(depth)
    if depth_array.ndim > 2:
        depth_array = np.squeeze(depth_array)
    if depth_array.shape[:2] != target_shape:
        depth_array = cv2.resize(
            depth_array.astype(np.float32, copy=False),
            (target_shape[1], target_shape[0]),
            interpolation=cv2.INTER_NEAREST,
        )
    if depth_array.dtype == np.uint16:
        return depth_array
    depth_m = depth_array.astype(np.float32, copy=False)
    depth_m = np.nan_to_num(depth_m, nan=0.0, posinf=0.0, neginf=0.0)
    depth_mm = np.clip(depth_m * 1000.0, 0.0, 65535.0)
    return depth_mm.astype(np.uint16)
