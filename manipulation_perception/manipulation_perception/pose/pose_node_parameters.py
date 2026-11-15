"""
Typed facade for the pose node's grouped ROS parameters.

Responsibility:
    Compose validated common,
    MegaPose, and GigaPose configuration groups.

Does not own:
    ROS transport, backend construction, or runtime mutation.

    The flat constructor keeps backend wiring compact.
Declaration, normalization and validation live in typed common, MegaPose and
GigaPose modules so backend code can depend on an explicit configuration group.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import TypeVar

from manipulation_perception.pose.pose_common_parameters import (
    CommonPoseParameters,
    declare_common_pose_parameters,
    read_common_pose_parameters,
    VALID_COARSE_ESTIMATORS,
    VALID_DEPTH_MODES,
    validate_common_pose_parameters,
)
from manipulation_perception.pose.pose_gigapose_parameters import (
    declare_gigapose_parameters,
    GigaPoseParameters,
    read_gigapose_parameters,
    validate_gigapose_parameters,
)
from manipulation_perception.pose.pose_megapose_parameters import (
    declare_megapose_parameters,
    MegaPoseParameters,
    read_megapose_parameters,
    VALID_MESH_UNITS,
    validate_megapose_parameters,
)
from manipulation_perception.pose.pose_parameter_access import ParameterNode
from manipulation_perception.pose.pose_worker_requests import model_uses_depth


_Config = TypeVar("_Config")


def _config_view(source: object, config_type: type[_Config]) -> _Config:
    """Project the stable flat facade onto one typed backend configuration."""
    values = {
        field.name: getattr(source, field.name)
        for field in fields(config_type)
    }
    return config_type(**values)


@dataclass(frozen=True)
class PoseNodeParameters:
    """Stable flat API plus typed views for new backend-specific code."""

    image_topic: str
    camera_info_topic: str
    depth_topic: str
    detection_bbox_stamped_topic: str
    output_detections_topic: str
    output_frame_id: str
    coarse_estimator: str
    detection_mask_topic: str
    megapose_python: str
    megapose_data_dir: Path
    megapose_model: str
    object_label: str
    mesh_file_path: Path
    mesh_units: str
    depth_mode: str
    run_every_n_frames: int
    min_inference_interval_sec: float
    bbox_timeout_sec: float
    process_timeout_sec: float
    score: float
    verbose_subprocess: bool
    persistent_worker: bool
    bsz_images: int
    bsz_objects: int
    renderer_workers: int
    gigapose_python: str
    gigapose_repo_dir: Path
    gigapose_dinov2_repo_dir: Path | None
    gigapose_data_dir: Path
    gigapose_checkpoint_path: Path
    gigapose_dataset_name: str
    gigapose_object_id: int
    gigapose_template_count: int
    gigapose_max_num_dets_per_forward: int
    gigapose_translation_scale: float
    gigapose_num_hypotheses: int
    gigapose_min_matching_score: float
    gigapose_persistent_worker: bool
    gigapose_refiner_bsz_images: int
    gigapose_refiner_bsz_objects: int
    gigapose_refiner_iterations: int

    @property
    def common(self) -> CommonPoseParameters:
        return _config_view(self, CommonPoseParameters)

    @property
    def megapose(self) -> MegaPoseParameters:
        return _config_view(self, MegaPoseParameters)

    @property
    def gigapose(self) -> GigaPoseParameters:
        return _config_view(self, GigaPoseParameters)

    @property
    def uses_gigapose_coarse(self) -> bool:
        """Preserve the historical flat backend selector helper."""
        return self.common.uses_gigapose_coarse

    @property
    def is_depth_disabled(self) -> bool:
        """Preserve the historical flat depth-policy helper."""
        return self.common.is_depth_disabled

    @property
    def example_name(self) -> str:
        """Use the injected object label as the backend runtime example key."""
        return self.object_label


def declare_pose_node_parameters(node: ParameterNode) -> None:
    """Declare all public parameters without creating ROS graph entities."""
    declare_common_pose_parameters(node)
    declare_megapose_parameters(node)
    declare_gigapose_parameters(node)


def read_pose_node_parameters(node: ParameterNode) -> PoseNodeParameters:
    """Read one immutable snapshot after every group has been declared."""
    common = read_common_pose_parameters(node)
    megapose = read_megapose_parameters(node)
    gigapose = read_gigapose_parameters(node)
    return PoseNodeParameters(
        **asdict(common),
        **asdict(megapose),
        **asdict(gigapose),
    )


def validate_pose_node_parameters(
    params: PoseNodeParameters,
    *,
    check_files: bool = True,
) -> None:
    """Validate common policy and only the selected backend's dependencies."""
    validate_common_pose_parameters(params.common)
    validate_megapose_parameters(params.megapose, check_files=check_files)
    if params.common.uses_gigapose_coarse:
        validate_gigapose_parameters(params.gigapose, check_files=check_files)


def pose_node_parameter_warnings(params: PoseNodeParameters) -> list[str]:
    """Return valid but suspicious combinations for throttled ROS logging."""
    if params.common.is_depth_disabled and model_uses_depth(
        params.megapose.megapose_model
    ):
        return [
            "Выбрана MegaPose RGB-D/ICP модель, но depth_mode=off; "
            "inference, скорее всего, завершится ошибкой без image_depth.png."
        ]
    return []


__all__ = [
    "PoseNodeParameters",
    "VALID_COARSE_ESTIMATORS",
    "VALID_DEPTH_MODES",
    "VALID_MESH_UNITS",
    "declare_pose_node_parameters",
    "pose_node_parameter_warnings",
    "read_pose_node_parameters",
    "validate_pose_node_parameters",
]
