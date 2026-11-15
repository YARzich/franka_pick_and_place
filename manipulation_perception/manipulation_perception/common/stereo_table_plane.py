"""
Table-plane detection helpers for stereo point clouds.

Responsibility:
- Estimate table-plane bounds and coefficients from workspace-frame points.
- Keep RANSAC/normalization math outside the ROS node.

Does not own:
- Warmup/sample lock state.
- TF lookup, marker creation or ROS publication.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from manipulation_perception.common.plane_geometry import (
    extract_table_plane_bounds,
    fit_table_plane_ransac,
    normalize_plane_coeffs,
)
from manipulation_perception.common.stereo_config import TablePlaneConfig


@dataclass(frozen=True)
class TablePlaneEstimate:
    """Metric workspace-frame table bounds and normalized plane equation."""

    bounds: tuple[float, float, float, float, float]
    coeffs: tuple[float, float, float, float]


@dataclass(frozen=True)
class TablePlaneDetectionResult:
    """One RANSAC attempt with a machine-readable failure reason."""

    estimate: TablePlaneEstimate | None
    failure: str | None = None


class TablePlaneDetector:
    """
    Fit an approximately horizontal plane from workspace-frame XYZ points.

    The injected generator makes RANSAC deterministic in tests. Inputs and
    bounds are metres; the configured up vector defines the positive normal
    orientation in the workspace frame.
    """

    def __init__(
        self,
        config: TablePlaneConfig,
        up_vector: np.ndarray,
        rng: np.random.Generator,
    ) -> None:
        self._config = config
        self._up_vector = up_vector
        self._rng = rng

    def detect(self, workspace_points: np.ndarray) -> TablePlaneDetectionResult:
        """Estimate table bounds and ``ax + by + cz + d = 0`` coefficients."""
        if workspace_points.shape[0] < 3:
            return TablePlaneDetectionResult(None, "not_enough_points")

        plane_result = fit_table_plane_ransac(
            workspace_points,
            self._config.ransac_iterations,
            self._config.distance_threshold_m,
            self._config.normal_max_tilt_deg,
            self._up_vector,
            self._rng,
        )
        if plane_result is None:
            return TablePlaneDetectionResult(None, "no_plane")

        plane_normal, plane_d, inliers = plane_result
        inlier_points = workspace_points[np.asarray(inliers, dtype=bool)]
        bounds = extract_table_plane_bounds(inlier_points)
        if bounds is None:
            return TablePlaneDetectionResult(None, "no_bounds")

        normalized_plane = normalize_plane_coeffs(plane_normal, plane_d)
        if normalized_plane is None:
            return TablePlaneDetectionResult(None, "invalid_coeffs")

        plane_normal_unit, plane_d_unit = normalized_plane
        coeffs = (
            float(plane_normal_unit[0]),
            float(plane_normal_unit[1]),
            float(plane_normal_unit[2]),
            float(plane_d_unit),
        )
        return TablePlaneDetectionResult(TablePlaneEstimate(bounds, coeffs))
