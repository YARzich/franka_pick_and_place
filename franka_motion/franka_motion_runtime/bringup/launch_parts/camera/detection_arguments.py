"""
Shared detection bbox launch arguments for table-camera perception.

Responsibility:
- Declare public bbox topics used by detector outputs and synchronized consumers.
- Keep legacy compact bbox compatibility separate from stereo-depth settings.

Does not own:
- YOLO model/runtime parameters.
- Stereo point-cloud, pose-estimator or backend-specific tuning.
"""

from __future__ import annotations

from franka_motion_runtime.bringup.launch_parts.camera.argument_utils import (
    LaunchArgumentGroup,
    add_launch_argument,
)
from franka_motion_runtime.bringup.launch_parts.camera.constants import (
    DETECTION_BBOX_LEGACY_TOPIC,
    DETECTION_BBOX_STAMPED_TOPIC,
)
from franka_motion_runtime.bringup.launch_parts.camera.paths import TableCameraPaths


def create_detection_arguments(paths: TableCameraPaths) -> LaunchArgumentGroup:
    """Declare stamped and legacy detection-output topic controls."""
    del paths
    configs = {}
    arguments = []

    add_launch_argument(
        configs,
        arguments,
        "detection_bbox_topic",
        DETECTION_BBOX_LEGACY_TOPIC,
        (
            "Legacy compact bbox куба от YOLO для совместимости "
            "с внешними потребителями."
        ),
    )
    add_launch_argument(
        configs,
        arguments,
        "detection_bbox_stamped_topic",
        DETECTION_BBOX_STAMPED_TOPIC,
        (
            "Stamped vision_msgs/Detection2DArray bbox куба для "
            "синхронизации pose и depth/point cloud ROI."
        ),
    )
    add_launch_argument(
        configs,
        arguments,
        "detection_bbox_timeout_sec",
        "0.5",
        (
            "Максимальное ожидание matching stamped bbox (сек) "
            "для ROI/pose синхронизации."
        ),
    )
    return configs, arguments
