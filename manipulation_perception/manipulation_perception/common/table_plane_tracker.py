"""
State tracker for one-shot tabletop plane publication.

Responsibility:
- Own table-plane warmup timing, sample accumulation and lock state.
- Aggregate repeated plane estimates into the fixed table plane output.

Does not own:
- RANSAC plane detection from point clouds.
- TF lookup, marker creation or ROS publication.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from manipulation_perception.common.plane_geometry import aggregate_table_plane_coeffs
from manipulation_perception.common.stereo_config import TablePlaneConfig


@dataclass(frozen=True)
class TablePlaneOutput:
    """Aggregated metric plane state and whether this sample completed locking."""

    bounds: tuple[float, float, float, float, float]
    coeffs: tuple[float, float, float, float]
    locked: bool
    just_locked: bool


class TablePlaneTracker:
    """
    Own warmup, sample aggregation and optional one-shot plane locking.

    Warmup uses ROS sample timestamps supplied to ``ready_for_detection``;
    clock jumps are therefore visible to the policy. The mutable tracker assumes
    serialized access by its owning perception callback.
    """

    def __init__(self, config: TablePlaneConfig, up_vector: np.ndarray) -> None:
        self._config = config
        self._up_vector = up_vector
        self._first_depth_stamp_sec: float | None = None
        self._locked = False
        self._bounds_samples: list[tuple[float, float, float, float, float]] = []
        self._coeff_samples: list[tuple[float, float, float, float]] = []
        self._locked_bounds: tuple[float, float, float, float, float] | None = None
        self._locked_coeffs: tuple[float, float, float, float] | None = None

    @property
    def locked(self) -> bool:
        """Return whether a one-shot estimate has reached its fixed state."""
        return self._locked

    def locked_output(self) -> TablePlaneOutput | None:
        """Return the fixed estimate without reporting a new lock transition."""
        if self._locked_bounds is None or self._locked_coeffs is None:
            return None
        return TablePlaneOutput(
            bounds=self._locked_bounds,
            coeffs=self._locked_coeffs,
            locked=self._locked,
            just_locked=False,
        )

    def ready_for_detection(self, stamp_sec: float) -> bool:
        """Advance ROS-stamp warmup and report whether RANSAC may run."""
        if self._config.detect_once and self._locked:
            return False
        if self._first_depth_stamp_sec is None:
            self._first_depth_stamp_sec = float(stamp_sec)
            return False
        return float(stamp_sec) - self._first_depth_stamp_sec >= self._config.init_delay_sec

    def add_sample(
        self,
        bounds: tuple[float, float, float, float, float],
        coeffs: tuple[float, float, float, float],
    ) -> TablePlaneOutput | None:
        """Aggregate one metric estimate and return output once policy permits."""
        if not self._config.detect_once:
            return TablePlaneOutput(bounds=bounds, coeffs=coeffs, locked=False, just_locked=False)

        self._bounds_samples.append(bounds)
        self._coeff_samples.append(coeffs)
        if len(self._bounds_samples) < self._config.init_required_frames:
            return None

        aggregated_bounds = tuple(
            float(np.median([sample[index] for sample in self._bounds_samples]))
            for index in range(5)
        )
        aggregated_coeffs = aggregate_table_plane_coeffs(self._coeff_samples, self._up_vector)
        if aggregated_coeffs is None:
            return None

        self._locked = True
        self._locked_bounds = aggregated_bounds
        self._locked_coeffs = aggregated_coeffs
        return TablePlaneOutput(
            bounds=aggregated_bounds,
            coeffs=aggregated_coeffs,
            locked=True,
            just_locked=True,
        )
