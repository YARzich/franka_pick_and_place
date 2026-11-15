"""
Pure tabletop-plane estimation for points expressed in one Cartesian frame.

Responsibility:
    Fit, normalize, aggregate, and bound tabletop planes using metric XYZ arrays
    and explicit up-vector constraints.

Does not own:
    TF lookup, point-cloud decoding, or temporal tracking.
"""

from __future__ import annotations

import math

import numpy as np


def fit_table_plane_ransac(
    points: np.ndarray,
    iterations: int,
    distance_threshold_m: float,
    normal_max_tilt_deg: float,
    up_vector: np.ndarray,
    rng: np.random.Generator,
) -> tuple[np.ndarray, float, np.ndarray] | None:
    """
    Fit an upward-facing plane `normal @ point + d = 0` with RANSAC.

    `points` must have shape `(N, 3)` in one frame and use metres. `up_vector`
    must be a unit vector in that frame. `iterations` must be positive,
    `distance_threshold_m` non-negative, and `normal_max_tilt_deg` expresses the
    maximum angular deviation from up. The returned boolean mask indexes the
    original points; `d` is in metres because the returned normal is unit length.
    """
    if points.shape[0] < 3:
        return None

    normal_max_tilt_rad = math.radians(normal_max_tilt_deg)
    min_up_dot = math.cos(normal_max_tilt_rad)

    best_inlier_count = 0
    best_normal = None
    best_d = 0.0
    best_inliers = None

    for _ in range(iterations):
        sample_idx = rng.choice(points.shape[0], size=3, replace=False)
        p1, p2, p3 = points[sample_idx]
        normal = np.cross(p2 - p1, p3 - p1)
        normal_norm = np.linalg.norm(normal)
        if normal_norm < 1.0e-8:
            continue

        normal = (normal / normal_norm).astype(np.float32)
        up_dot = float(np.dot(normal, up_vector))
        if abs(up_dot) < min_up_dot:
            continue
        if up_dot < 0.0:
            normal = -normal

        d_value = -float(np.dot(normal, p1))
        distances = np.abs(points @ normal + d_value)
        inliers = distances <= distance_threshold_m
        inlier_count = int(np.count_nonzero(inliers))
        if inlier_count < best_inlier_count:
            continue

        inlier_points = points[inliers]
        centroid = np.mean(inlier_points, axis=0)
        _, _, vh = np.linalg.svd(inlier_points - centroid, full_matrices=False)
        refined_normal = vh[-1].astype(np.float32)
        refined_norm = np.linalg.norm(refined_normal)
        if refined_norm < 1.0e-8:
            continue
        refined_normal /= refined_norm

        refined_up_dot = float(np.dot(refined_normal, up_vector))
        if abs(refined_up_dot) < min_up_dot:
            continue
        if refined_up_dot < 0.0:
            refined_normal = -refined_normal

        refined_d = -float(np.dot(refined_normal, centroid))
        refined_distances = np.abs(points @ refined_normal + refined_d)
        refined_inliers = refined_distances <= distance_threshold_m
        refined_count = int(np.count_nonzero(refined_inliers))

        if refined_count >= best_inlier_count:
            best_inlier_count = refined_count
            best_normal = refined_normal
            best_d = refined_d
            best_inliers = refined_inliers

    if best_normal is None or best_inliers is None:
        return None
    return best_normal, best_d, best_inliers


def extract_table_plane_bounds(
    inlier_points: np.ndarray,
) -> tuple[float, float, float, float, float] | None:
    """
    Return axis-aligned `(x_min, x_max, y_min, y_max, median_z)` bounds.

    Points must have shape `(N, 3)`, share one frame, and use metres. At least
    three inliers are required. Bounds remain expressed in the input frame.
    """
    if inlier_points.shape[0] < 3:
        return None

    x_values = inlier_points[:, 0]
    y_values = inlier_points[:, 1]
    z_values = inlier_points[:, 2]
    return (
        float(np.min(x_values)),
        float(np.max(x_values)),
        float(np.min(y_values)),
        float(np.max(y_values)),
        float(np.median(z_values)),
    )


def normalize_plane_coeffs(
    normal: np.ndarray,
    d_value: float,
) -> tuple[np.ndarray, float] | None:
    """
    Normalize plane coefficients while preserving their input frame.

    The plane uses `normal @ point + d_value = 0`. Point coordinates and
    `d_value` must use the same length unit; this package supplies metres. A
    near-zero normal has no defined plane and returns `None`.
    """
    normal_norm = float(np.linalg.norm(normal))
    if normal_norm < 1.0e-8:
        return None
    return (normal / normal_norm).astype(np.float32), float(d_value) / normal_norm


def aggregate_table_plane_coeffs(
    coeff_samples: list[tuple[float, float, float, float]],
    up_vector: np.ndarray,
) -> tuple[float, float, float, float] | None:
    """
    Median-aggregate plane samples and orient the result toward frame up.

    Samples are `(nx, ny, nz, d)` coefficients for one coordinate frame; `d`
    uses metres. `up_vector` is a non-zero vector in that same frame. Invalid
    zero-normal samples are ignored, and `None` is returned if none remain.
    """
    if not coeff_samples:
        return None

    normals: list[np.ndarray] = []
    d_values: list[float] = []
    for nx, ny, nz, d_value in coeff_samples:
        normal = np.array([nx, ny, nz], dtype=np.float32)
        if float(np.dot(normal, up_vector)) < 0.0:
            normal = -normal
            d_value = -d_value
        normalized = normalize_plane_coeffs(normal, d_value)
        if normalized is None:
            continue
        normal_unit, d_unit = normalized
        normals.append(normal_unit)
        d_values.append(d_unit)

    if not normals:
        return None

    aggregated_normal = np.median(np.stack(normals, axis=0), axis=0).astype(np.float32)
    aggregated_d = float(np.median(np.asarray(d_values, dtype=np.float32)))
    normalized = normalize_plane_coeffs(aggregated_normal, aggregated_d)
    if normalized is None:
        return None

    normal_unit, d_unit = normalized
    if float(np.dot(normal_unit, up_vector)) < 0.0:
        normal_unit = -normal_unit
        d_unit = -d_unit
    return (
        float(normal_unit[0]),
        float(normal_unit[1]),
        float(normal_unit[2]),
        float(d_unit),
    )


__all__ = [
    "aggregate_table_plane_coeffs",
    "extract_table_plane_bounds",
    "fit_table_plane_ransac",
    "normalize_plane_coeffs",
]
