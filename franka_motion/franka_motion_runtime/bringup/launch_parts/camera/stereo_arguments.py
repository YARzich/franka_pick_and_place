"""
Stereo point-cloud and ROI launch arguments for table-camera perception.

Responsibility: Declare SGBM/ESS point-cloud topics, frames, workspace limits and
workspace settings.

Does not own:
- Detector backend parameters.
- Pose-estimator backend parameters.
"""

from __future__ import annotations

from franka_motion_runtime.bringup.launch_parts.camera.argument_utils import (
    LaunchArgumentGroup,
    add_launch_argument,
)
from franka_motion_runtime.bringup.launch_parts.camera.constants import POINT_CLOUD_TOPIC, WORLD_FRAME
from franka_motion_runtime.bringup.launch_parts.camera.paths import TableCameraPaths


def create_stereo_arguments(paths: TableCameraPaths) -> LaunchArgumentGroup:
    """Declare depth/cloud topics, frames and workspace controls."""
    del paths
    configs = {}
    arguments = []

    add_launch_argument(
        configs,
        arguments,
        "point_cloud_topic",
        POINT_CLOUD_TOPIC,
        "Топик облака точек",
    )
    add_launch_argument(
        configs,
        arguments,
        "point_cloud_frame",
        WORLD_FRAME,
        "TF frame для PointCloud2 (обычно world для совмещения с роботом в RViz)",
    )
    add_launch_argument(
        configs,
        arguments,
        "point_cloud_stride",
        "4",
        "Downsample depth перед построением point cloud (каждый N-й пиксель)",
    )
    add_launch_argument(
        configs,
        arguments,
        "run_every_n_frames",
        "1",
        "Обрабатывать каждый N-й стереокадр",
    )
    add_launch_argument(
        configs,
        arguments,
        "workspace_frame",
        WORLD_FRAME,
        "TF frame, в котором вычисляется рабочая зона и плоскость стола",
    )
    add_launch_argument(
        configs,
        arguments,
        "robot_base_frame",
        "panda_link0",
        "TF frame базы робота для расчета рабочей зоны",
    )
    add_launch_argument(
        configs,
        arguments,
        "robot_reach_m",
        "0.855",
        "Номинальная досягаемость робота (м)",
    )
    add_launch_argument(
        configs,
        arguments,
        "workspace_extra_ratio",
        "0.25",
        "Дополнительный радиус рабочей зоны как доля от robot_reach_m",
    )
    return configs, arguments
