"""
Configuration value objects for the stereo perception pipeline.

Responsibility:
- Normalize raw ROS parameter values into small immutable configs.
- Keep OpenCV and marker constraints close to the data they configure.

Does not own:
- ROS parameter declaration or retrieval.
- Runtime OpenCV objects, TF lookup, publishers or subscriptions.
"""

from __future__ import annotations

from dataclasses import dataclass


VALID_ENHANCER_TAGS = {"none", "ecc", "ess"}


def normalize_enhancer_tag(value: str) -> str:
    """Normalize a configured enhancer name, falling back to ``none``."""
    tag = str(value).strip().lower()
    return tag if tag in VALID_ENHANCER_TAGS else "none"


@dataclass(frozen=True)
class StereoSgbmConfig:
    """Validated OpenCV StereoSGBM parameters in pixel-domain units."""

    min_disparity: int
    num_disparities: int
    block_size: int
    p1: int
    p2: int
    pre_filter_cap: int
    uniqueness_ratio: int
    speckle_window_size: int
    speckle_range: int
    disp12_max_diff: int

    @classmethod
    def normalized(
        cls,
        *,
        min_disparity: int,
        num_disparities: int,
        block_size: int,
        p1: int,
        p2: int,
        pre_filter_cap: int,
        uniqueness_ratio: int,
        speckle_window_size: int,
        speckle_range: int,
        disp12_max_diff: int,
    ) -> "StereoSgbmConfig":
        """Enforce OpenCV disparity/block constraints and derive P1/P2 defaults."""
        if num_disparities <= 0:
            num_disparities = 128
        if num_disparities % 16 != 0:
            num_disparities = ((num_disparities + 15) // 16) * 16
        if block_size < 3:
            block_size = 3
        if block_size % 2 == 0:
            block_size += 1
        if p1 <= 0:
            p1 = 8 * block_size * block_size
        if p2 <= 0:
            p2 = 32 * block_size * block_size

        return cls(
            min_disparity=int(min_disparity),
            num_disparities=int(num_disparities),
            block_size=int(block_size),
            p1=int(p1),
            p2=int(p2),
            pre_filter_cap=int(pre_filter_cap),
            uniqueness_ratio=int(uniqueness_ratio),
            speckle_window_size=int(speckle_window_size),
            speckle_range=int(speckle_range),
            disp12_max_diff=int(disp12_max_diff),
        )


@dataclass(frozen=True)
class EccConfig:
    """Validated cadence and convergence controls for stateful ECC alignment."""

    enabled: bool
    max_iterations: int
    eps: float
    gauss_filter_size: int
    update_every_n_frames: int

    @classmethod
    def normalized(
        cls,
        *,
        enabled: bool,
        max_iterations: int,
        eps: float,
        gauss_filter_size: int,
        update_every_n_frames: int,
    ) -> "EccConfig":
        """Clamp convergence values and make the Gaussian kernel positive odd."""
        max_iterations = max(1, int(max_iterations))
        eps = max(1.0e-8, float(eps))
        gauss_filter_size = int(gauss_filter_size)
        if gauss_filter_size < 1:
            gauss_filter_size = 1
        if gauss_filter_size % 2 == 0:
            gauss_filter_size += 1
        update_every_n_frames = max(1, int(update_every_n_frames))
        return cls(
            enabled=bool(enabled),
            max_iterations=max_iterations,
            eps=eps,
            gauss_filter_size=gauss_filter_size,
            update_every_n_frames=update_every_n_frames,
        )


@dataclass(frozen=True)
class TablePlaneConfig:
    """Table-plane RANSAC and visualization policy in metric units."""

    detect_once: bool
    init_delay_sec: float
    init_required_frames: int
    ransac_iterations: int
    distance_threshold_m: float
    normal_max_tilt_deg: float
    marker_thickness_m: float
    marker_alpha: float

    @classmethod
    def normalized(
        cls,
        *,
        detect_once: bool,
        init_delay_sec: float,
        init_required_frames: int,
        ransac_iterations: int,
        distance_threshold_m: float,
        normal_max_tilt_deg: float,
        marker_thickness_m: float,
        marker_alpha: float,
    ) -> "TablePlaneConfig":
        """Return a config whose counts, distances and marker alpha are valid."""
        return cls(
            detect_once=bool(detect_once),
            init_delay_sec=max(0.0, float(init_delay_sec)),
            init_required_frames=max(1, int(init_required_frames)),
            ransac_iterations=max(20, int(ransac_iterations)),
            distance_threshold_m=max(1.0e-4, float(distance_threshold_m)),
            normal_max_tilt_deg=float(normal_max_tilt_deg),
            marker_thickness_m=max(0.001, float(marker_thickness_m)),
            marker_alpha=min(1.0, max(0.05, float(marker_alpha))),
        )


@dataclass(frozen=True)
class ClusterMarkerConfig:
    """Voxel-cluster admission and marker geometry policy in metres."""

    voxel_size_m: float
    min_points: int
    max_points: int
    min_diagonal_m: float
    max_diagonal_m: float
    marker_min_edge_m: float
    marker_alpha: float

    @classmethod
    def normalized(
        cls,
        *,
        voxel_size_m: float,
        min_points: int,
        max_points: int,
        min_diagonal_m: float,
        max_diagonal_m: float,
        marker_min_edge_m: float,
        marker_alpha: float,
    ) -> "ClusterMarkerConfig":
        """Normalize cluster bounds while preserving ``min <= max`` invariants."""
        voxel_size_m = max(0.003, float(voxel_size_m))
        min_points = max(10, int(min_points))
        max_points = max(min_points, int(max_points))
        min_diagonal_m = max(0.001, float(min_diagonal_m))
        max_diagonal_m = max(min_diagonal_m, float(max_diagonal_m))
        return cls(
            voxel_size_m=voxel_size_m,
            min_points=min_points,
            max_points=max_points,
            min_diagonal_m=min_diagonal_m,
            max_diagonal_m=max_diagonal_m,
            marker_min_edge_m=max(0.002, float(marker_min_edge_m)),
            marker_alpha=min(1.0, max(0.05, float(marker_alpha))),
        )


@dataclass(frozen=True)
class WorkspaceConfig:
    """TF frames and radial robot-reach crop used by stereo cloud outputs."""

    workspace_frame: str
    robot_base_frame: str
    point_cloud_frame: str
    robot_reach_m: float
    workspace_extra_ratio: float

    @classmethod
    def normalized(
        cls,
        *,
        workspace_frame: str,
        robot_base_frame: str,
        point_cloud_frame: str,
        robot_reach_m: float,
        workspace_extra_ratio: float,
    ) -> "WorkspaceConfig":
        """Preserve scenario reach for validation and clamp only the margin."""
        return cls(
            workspace_frame=str(workspace_frame),
            robot_base_frame=str(robot_base_frame),
            point_cloud_frame=str(point_cloud_frame),
            robot_reach_m=float(robot_reach_m),
            workspace_extra_ratio=max(0.0, float(workspace_extra_ratio)),
        )

    @property
    def workspace_radius_m(self) -> float:
        """Return robot reach plus the configured fractional safety margin."""
        return self.robot_reach_m * (1.0 + self.workspace_extra_ratio)
