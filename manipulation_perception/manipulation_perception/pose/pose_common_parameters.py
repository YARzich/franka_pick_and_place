"""
Common ROS I/O and inference scheduling parameters for pose estimation.

Responsibility:
    Declare, read, normalize, and validate backend-neutral topics, frames,
    timestamps, queue limits, and inference scheduling parameters.

Does not own:
    Backend-specific assets or ROS subscriptions.
"""

from __future__ import annotations

from dataclasses import dataclass

from manipulation_perception.common.config_validation import (
    require_frame_id,
    require_non_empty,
)
from manipulation_perception.pose.pose_parameter_access import (
    declare_parameters,
    ParameterNode,
    ParameterReader,
    ParameterSpec,
)


VALID_COARSE_ESTIMATORS = {"megapose", "gigapose"}
VALID_DEPTH_MODES = {"auto", "on", "true", "1", "rgbd", "icp", "off", "false", "0"}

_COMMON_PARAMETER_SPECS: tuple[ParameterSpec, ...] = (
    ("image_topic", ""),
    ("camera_info_topic", ""),
    ("depth_topic", ""),
    ("detection_bbox_stamped_topic", ""),
    ("output_detections_topic", ""),
    ("output_frame_id", ""),
    ("coarse_estimator", "megapose"),
    ("depth_mode", "on"),
    # Kept for compatibility with older launch/YAML files. It only affects auto mode.
    ("use_depth", False),
    ("run_every_n_frames", 30),
    ("min_inference_interval_sec", 1.0),
    ("bbox_timeout_sec", 0.7),
    ("process_timeout_sec", 120.0),
    ("score", 1.0),
    ("verbose_subprocess", False),
)


@dataclass(frozen=True)
class CommonPoseParameters:
    """Backend-independent ROS topics, timing policy and output metadata."""

    image_topic: str
    camera_info_topic: str
    depth_topic: str
    detection_bbox_stamped_topic: str
    output_detections_topic: str
    output_frame_id: str
    coarse_estimator: str
    depth_mode: str
    run_every_n_frames: int
    min_inference_interval_sec: float
    bbox_timeout_sec: float
    process_timeout_sec: float
    score: float
    verbose_subprocess: bool

    @property
    def uses_gigapose_coarse(self) -> bool:
        return self.coarse_estimator == "gigapose"

    @property
    def is_depth_disabled(self) -> bool:
        return self.depth_mode in {"off", "false", "0"}


def declare_common_pose_parameters(node: ParameterNode) -> None:
    """Declare backend-independent topics, timing and compatibility inputs."""
    declare_parameters(node, _COMMON_PARAMETER_SPECS)


def read_common_pose_parameters(node: ParameterNode) -> CommonPoseParameters:
    """Normalize one common config snapshot, including legacy depth semantics."""
    reader = ParameterReader(node)
    depth_mode = reader.text("depth_mode", fallback="on").lower()
    if reader.boolean("use_depth") and depth_mode == "auto":
        depth_mode = "on"

    return CommonPoseParameters(
        image_topic=str(reader.value("image_topic")),
        camera_info_topic=str(reader.value("camera_info_topic")),
        depth_topic=str(reader.value("depth_topic")),
        detection_bbox_stamped_topic=str(reader.value("detection_bbox_stamped_topic")),
        output_detections_topic=str(reader.value("output_detections_topic")),
        output_frame_id=str(reader.value("output_frame_id")),
        coarse_estimator=reader.text("coarse_estimator", fallback="megapose").lower(),
        depth_mode=depth_mode,
        run_every_n_frames=max(1, int(reader.value("run_every_n_frames"))),
        min_inference_interval_sec=max(
            0.0,
            float(reader.value("min_inference_interval_sec")),
        ),
        bbox_timeout_sec=max(0.05, float(reader.value("bbox_timeout_sec"))),
        process_timeout_sec=max(1.0, float(reader.value("process_timeout_sec"))),
        score=max(0.0, min(1.0, float(reader.value("score")))),
        verbose_subprocess=reader.boolean("verbose_subprocess"),
    )


def validate_common_pose_parameters(params: CommonPoseParameters) -> None:
    """Reject unsupported backend and depth policy selectors."""
    if params.depth_mode not in VALID_DEPTH_MODES:
        raise ValueError("depth_mode должен быть auto, on/rgbd/icp или off")
    if params.coarse_estimator not in VALID_COARSE_ESTIMATORS:
        raise ValueError("coarse_estimator должен быть 'megapose' или 'gigapose'")
    required = {
        "image_topic": params.image_topic,
        "camera_info_topic": params.camera_info_topic,
        "detection_bbox_stamped_topic": params.detection_bbox_stamped_topic,
        "output_detections_topic": params.output_detections_topic,
        "output_frame_id": params.output_frame_id,
    }
    if not params.is_depth_disabled:
        required["depth_topic"] = params.depth_topic
    require_non_empty(required, context="6D pose perception")
    require_frame_id(
        params.output_frame_id,
        "output_frame_id",
        context="6D pose perception",
    )


__all__ = [
    "CommonPoseParameters",
    "VALID_COARSE_ESTIMATORS",
    "VALID_DEPTH_MODES",
    "declare_common_pose_parameters",
    "read_common_pose_parameters",
    "validate_common_pose_parameters",
]
