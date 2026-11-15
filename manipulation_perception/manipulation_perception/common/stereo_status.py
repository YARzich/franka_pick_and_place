"""
Operator-facing startup status for the stereo ROS adapter.

Responsibility:
    Format a deterministic summary of the selected depth backend, input topics,
    output topics, synchronization mode, and optional ROI behavior.

Does not own:
    Logging, parameter validation, or pipeline state.
"""

from __future__ import annotations

from typing import Any


def format_stereo_startup_status(parameters: Any, *, roi_enabled: bool) -> str:
    """Describe the selected depth backend and owned output topics."""
    cluster_topic = (
        parameters.cluster_marker_topic
        if parameters.publish_cluster_markers
        else "disabled"
    )
    if parameters.use_external_disparity:
        return (
            "External disparity mode: "
            f"disparity_in={parameters.external_disparity_topic}, "
            f"camera_info={parameters.left_camera_info_topic}, sync=exact_stamp, "
            f"depth={parameters.depth_topic}, points={parameters.point_cloud_topic}, "
            f"octomap_points={parameters.octomap_point_cloud_topic}, "
            f"octomap_min_points={parameters.octomap_min_points}, "
            f"enhancer_tag={parameters.enhancer_tag}, "
            f"use_detection_bbox_roi={roi_enabled}, "
            f"cluster_markers={cluster_topic}"
        )
    return (
        "StereoSGBM ready: "
        f"left={parameters.left_image_topic}, right={parameters.right_image_topic}, "
        f"camera_info={parameters.left_camera_info_topic}, sync=exact_stamp, "
        f"disparity={parameters.disparity_topic}, depth={parameters.depth_topic}, "
        f"points={parameters.point_cloud_topic}, "
        f"octomap_points={parameters.octomap_point_cloud_topic}, "
        f"octomap_min_points={parameters.octomap_min_points}, "
        f"frame_id={parameters.local_stereo_frame_id}, "
        f"num_disparities={parameters.stereo_config.num_disparities}, "
        f"block_size={parameters.stereo_config.block_size}, "
        f"enhancer_tag={parameters.enhancer_tag}, "
        f"use_detection_bbox_roi={roi_enabled}, "
        f"cluster_markers={cluster_topic}"
    )


__all__ = ["format_stereo_startup_status"]
