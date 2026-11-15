"""
Voxel-connectivity clustering for unorganized XYZ point arrays.

Responsibility:
    Quantize metric XYZ samples and find bounded 26-connected components for
    backend-neutral point-cluster extraction.

Does not own:
    Point-cloud transport, semantic classification, or tracking.
"""

from __future__ import annotations

import numpy as np


NEIGHBOR_26_OFFSETS = tuple(
    (dx, dy, dz)
    for dx in (-1, 0, 1)
    for dy in (-1, 0, 1)
    for dz in (-1, 0, 1)
    if (dx, dy, dz) != (0, 0, 0)
)


def cluster_points_voxel(
    points: np.ndarray,
    voxel_size_m: float,
    min_points: int,
    max_points: int,
) -> list[np.ndarray]:
    """
    Return input-index clusters connected through 26-neighbor voxels.

    `points` must have shape `(N, 3)` in one frame and use metres.
    `voxel_size_m` must be positive, and `0 <= min_points <= max_points`.
    Each returned int32 array indexes `points`; point coordinates are not copied.
    """
    if points.shape[0] == 0:
        return []

    voxel_coords = np.floor(points / voxel_size_m).astype(np.int32)
    unique_voxels, inverse = np.unique(voxel_coords, axis=0, return_inverse=True)
    voxel_count = unique_voxels.shape[0]
    if voxel_count == 0:
        return []

    point_indices_by_voxel: list[list[int]] = [[] for _ in range(voxel_count)]
    for point_index, voxel_index in enumerate(inverse):
        point_indices_by_voxel[int(voxel_index)].append(point_index)

    voxel_lookup = {tuple(voxel.tolist()): idx for idx, voxel in enumerate(unique_voxels)}
    visited = np.zeros(voxel_count, dtype=bool)
    clusters: list[np.ndarray] = []

    for start_idx in range(voxel_count):
        if visited[start_idx]:
            continue

        queue = [start_idx]
        visited[start_idx] = True
        cluster_indices: list[int] = []
        while queue:
            voxel_idx = queue.pop()
            cluster_indices.extend(point_indices_by_voxel[voxel_idx])
            vx, vy, vz = unique_voxels[voxel_idx]
            for dx, dy, dz in NEIGHBOR_26_OFFSETS:
                neighbor_idx = voxel_lookup.get((int(vx + dx), int(vy + dy), int(vz + dz)))
                if neighbor_idx is None or visited[neighbor_idx]:
                    continue
                visited[neighbor_idx] = True
                queue.append(neighbor_idx)

        cluster_size = len(cluster_indices)
        if cluster_size < min_points or cluster_size > max_points:
            continue
        clusters.append(np.asarray(cluster_indices, dtype=np.int32))

    return clusters


__all__ = ["NEIGHBOR_26_OFFSETS", "cluster_points_voxel"]
