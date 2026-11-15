"""
ROS parameter declaration and reading for the stereo perception node.

Responsibility:
- Declare the public ROS parameter API of stereo_sgbm_pointcloud_node.
- Read raw parameter values from a ROS node-like object.
- Normalize values into the stereo helper config objects used by the node.

Does not own:
- Publishers, subscriptions, callbacks or TF lookup.
- Image processing, point cloud projection or marker publication.
- Launch/YAML defaults outside the node's fallback defaults.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from manipulation_perception.common.config_validation import (
    require_frame_id,
    require_non_empty,
    require_positive,
)
from manipulation_perception.common.parameter_utils import bool_value
from manipulation_perception.common.stereo_config import (
    ClusterMarkerConfig,
    EccConfig,
    normalize_enhancer_tag,
    StereoSgbmConfig,
    TablePlaneConfig,
    WorkspaceConfig,
)


@dataclass(frozen=True)
class StereoNodeParameters:
    """
    Normalized transport, TF and algorithm contract for the stereo node.

    Topic and frame strings remain public ROS API values. Distances are metres,
    synchronization tolerances are seconds in ROS time, and
    ``allow_latest_tf_fallback`` explicitly controls whether exact stamped TF
    lookup may degrade to latest TF.
    """

    left_image_topic: str
    right_image_topic: str
    left_camera_info_topic: str
    left_image_frame_id: str
    right_image_frame_id: str
    local_stereo_frame_id: str
    disparity_topic: str
    depth_topic: str
    point_cloud_topic: str
    octomap_point_cloud_topic: str
    point_cloud_frame: str
    allow_latest_tf_fallback: bool
    enhancer_tag_raw: str
    enhancer_tag: str
    use_external_disparity: bool
    external_disparity_topic: str
    use_detection_bbox_roi: bool
    detection_bbox_stamped_topic: str
    detection_bbox_timeout_sec: float
    baseline_m: float
    point_cloud_stride: int
    octomap_min_points: int
    run_every_n_frames: int
    min_valid_disparity_px: float
    min_depth_m: float
    max_depth_m: float
    sync_queue_size: int
    sync_slop_sec: float
    workspace_config: WorkspaceConfig
    table_plane_marker_topic: str
    table_plane_coeffs_topic: str
    table_plane_config: TablePlaneConfig
    publish_table_plane_marker: bool
    publish_cluster_markers: bool
    cluster_marker_topic: str
    cluster_marker_config: ClusterMarkerConfig
    ecc_config: EccConfig
    stereo_config: StereoSgbmConfig

    @property
    def has_unknown_enhancer_tag(self) -> bool:
        """Return whether an unsupported enhancer was normalized to ``none``."""
        return self.enhancer_tag_raw != self.enhancer_tag


def declare_stereo_node_parameters(node: Any) -> None:
    """Declare algorithm defaults while leaving scenario-owned values empty."""
    node.declare_parameter("left_image_topic", "")
    node.declare_parameter("right_image_topic", "")
    node.declare_parameter("left_camera_info_topic", "")
    node.declare_parameter("left_image_frame_id", "")
    node.declare_parameter("right_image_frame_id", "")
    node.declare_parameter("local_stereo_frame_id", "")
    node.declare_parameter("disparity_topic", "")
    node.declare_parameter("depth_topic", "")
    node.declare_parameter("point_cloud_topic", "")
    node.declare_parameter("octomap_point_cloud_topic", "")
    node.declare_parameter("point_cloud_frame", "")
    node.declare_parameter("allow_latest_tf_fallback", False)
    node.declare_parameter("enhancer_tag", "none")
    node.declare_parameter("use_external_disparity", False)
    node.declare_parameter("external_disparity_topic", "")
    node.declare_parameter("use_detection_bbox_roi", False)
    node.declare_parameter("detection_bbox_stamped_topic", "")
    node.declare_parameter("detection_bbox_timeout_sec", 0.5)

    node.declare_parameter("baseline_m", 0.0)
    node.declare_parameter("point_cloud_stride", 4)
    node.declare_parameter("octomap_min_points", 32)
    node.declare_parameter("run_every_n_frames", 1)
    node.declare_parameter("min_depth_m", 0.1)
    node.declare_parameter("max_depth_m", 20.0)

    node.declare_parameter("workspace_frame", "")
    node.declare_parameter("robot_base_frame", "")
    node.declare_parameter("robot_reach_m", 0.0)
    node.declare_parameter("workspace_extra_ratio", 0.25)

    node.declare_parameter("publish_table_plane_marker", True)
    node.declare_parameter("table_plane_marker_topic", "")
    node.declare_parameter("table_plane_coeffs_topic", "")
    node.declare_parameter("table_plane_detect_once", True)
    node.declare_parameter("table_plane_init_delay_sec", 3.0)
    node.declare_parameter("table_plane_init_required_frames", 5)
    node.declare_parameter("plane_ransac_iterations", 120)
    node.declare_parameter("plane_distance_threshold_m", 0.01)
    node.declare_parameter("plane_normal_max_tilt_deg", 20.0)
    node.declare_parameter("plane_marker_thickness_m", 0.004)
    node.declare_parameter("plane_marker_alpha", 0.35)

    node.declare_parameter("publish_cluster_markers", True)
    node.declare_parameter("cluster_marker_topic", "")
    node.declare_parameter("cluster_voxel_size_m", 0.012)
    node.declare_parameter("cluster_min_points", 40)
    node.declare_parameter("cluster_max_points", 200000)
    node.declare_parameter("cluster_min_diagonal_m", 0.03)
    node.declare_parameter("cluster_max_diagonal_m", 0.35)
    node.declare_parameter("cluster_marker_min_edge_m", 0.01)
    node.declare_parameter("cluster_marker_alpha", 0.55)

    node.declare_parameter("sync_queue_size", 10)
    node.declare_parameter("sync_slop_sec", 0.0)

    node.declare_parameter("min_disparity", 0)
    node.declare_parameter("num_disparities", 128)
    node.declare_parameter("block_size", 5)
    node.declare_parameter("p1", 0)
    node.declare_parameter("p2", 0)
    node.declare_parameter("pre_filter_cap", 63)
    node.declare_parameter("uniqueness_ratio", 5)
    node.declare_parameter("speckle_window_size", 80)
    node.declare_parameter("speckle_range", 2)
    node.declare_parameter("disp12_max_diff", 1)
    node.declare_parameter("min_valid_disparity_px", 0.1)
    node.declare_parameter("ecc_max_iterations", 30)
    node.declare_parameter("ecc_eps", 1.0e-4)
    node.declare_parameter("ecc_gauss_filter_size", 5)
    node.declare_parameter("ecc_update_every_n_frames", 1)


def _parameter_value(node: Any, name: str) -> Any:
    return node.get_parameter(name).value


def _bool_parameter(node: Any, name: str) -> bool:
    return bool_value(_parameter_value(node, name), name)


def _finite_float_parameter(node: Any, name: str) -> float:
    """Read one floating-point parameter without masking NaN or infinity."""
    value = float(_parameter_value(node, name))
    if not math.isfinite(value):
        raise ValueError(f"stereo perception: {name} must be finite")
    return value


def _require_finite_non_negative(value: float, name: str) -> None:
    """Require a finite metric threshold that may explicitly be zero."""
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(
            f"stereo perception: {name} must be finite and greater than or equal to zero"
        )


def read_stereo_node_parameters(node: Any) -> StereoNodeParameters:
    """Read and normalize the complete stereo node configuration snapshot."""
    point_cloud_frame = str(_parameter_value(node, "point_cloud_frame"))
    enhancer_tag_raw = str(_parameter_value(node, "enhancer_tag")).strip().lower()
    enhancer_tag = normalize_enhancer_tag(enhancer_tag_raw)
    max_depth_m = _finite_float_parameter(node, "max_depth_m")

    return StereoNodeParameters(
        left_image_topic=str(_parameter_value(node, "left_image_topic")),
        right_image_topic=str(_parameter_value(node, "right_image_topic")),
        left_camera_info_topic=str(_parameter_value(node, "left_camera_info_topic")),
        left_image_frame_id=str(_parameter_value(node, "left_image_frame_id")),
        right_image_frame_id=str(_parameter_value(node, "right_image_frame_id")),
        local_stereo_frame_id=str(_parameter_value(node, "local_stereo_frame_id")),
        disparity_topic=str(_parameter_value(node, "disparity_topic")),
        depth_topic=str(_parameter_value(node, "depth_topic")),
        point_cloud_topic=str(_parameter_value(node, "point_cloud_topic")),
        octomap_point_cloud_topic=str(_parameter_value(node, "octomap_point_cloud_topic")),
        point_cloud_frame=point_cloud_frame,
        allow_latest_tf_fallback=_bool_parameter(node, "allow_latest_tf_fallback"),
        enhancer_tag_raw=enhancer_tag_raw,
        enhancer_tag=enhancer_tag,
        use_external_disparity=_bool_parameter(node, "use_external_disparity"),
        external_disparity_topic=str(_parameter_value(node, "external_disparity_topic")),
        use_detection_bbox_roi=_bool_parameter(node, "use_detection_bbox_roi"),
        detection_bbox_stamped_topic=str(_parameter_value(node, "detection_bbox_stamped_topic")),
        detection_bbox_timeout_sec=_finite_float_parameter(node, "detection_bbox_timeout_sec"),
        baseline_m=_finite_float_parameter(node, "baseline_m"),
        point_cloud_stride=max(1, int(_parameter_value(node, "point_cloud_stride"))),
        octomap_min_points=max(1, int(_parameter_value(node, "octomap_min_points"))),
        run_every_n_frames=max(1, int(_parameter_value(node, "run_every_n_frames"))),
        min_valid_disparity_px=_finite_float_parameter(node, "min_valid_disparity_px"),
        min_depth_m=max(0.0, _finite_float_parameter(node, "min_depth_m")),
        max_depth_m=max_depth_m if max_depth_m > 0.0 else float("inf"),
        sync_queue_size=max(2, int(_parameter_value(node, "sync_queue_size"))),
        sync_slop_sec=max(0.0, _finite_float_parameter(node, "sync_slop_sec")),
        workspace_config=WorkspaceConfig.normalized(
            workspace_frame=str(_parameter_value(node, "workspace_frame")),
            robot_base_frame=str(_parameter_value(node, "robot_base_frame")),
            point_cloud_frame=point_cloud_frame,
            robot_reach_m=_finite_float_parameter(node, "robot_reach_m"),
            workspace_extra_ratio=_finite_float_parameter(node, "workspace_extra_ratio"),
        ),
        table_plane_marker_topic=str(_parameter_value(node, "table_plane_marker_topic")),
        table_plane_coeffs_topic=str(_parameter_value(node, "table_plane_coeffs_topic")),
        table_plane_config=TablePlaneConfig.normalized(
            detect_once=_bool_parameter(node, "table_plane_detect_once"),
            init_delay_sec=_finite_float_parameter(node, "table_plane_init_delay_sec"),
            init_required_frames=int(_parameter_value(node, "table_plane_init_required_frames")),
            ransac_iterations=int(_parameter_value(node, "plane_ransac_iterations")),
            distance_threshold_m=_finite_float_parameter(node, "plane_distance_threshold_m"),
            normal_max_tilt_deg=_finite_float_parameter(node, "plane_normal_max_tilt_deg"),
            marker_thickness_m=_finite_float_parameter(node, "plane_marker_thickness_m"),
            marker_alpha=_finite_float_parameter(node, "plane_marker_alpha"),
        ),
        publish_table_plane_marker=_bool_parameter(node, "publish_table_plane_marker"),
        publish_cluster_markers=_bool_parameter(node, "publish_cluster_markers"),
        cluster_marker_topic=str(_parameter_value(node, "cluster_marker_topic")),
        cluster_marker_config=ClusterMarkerConfig.normalized(
            voxel_size_m=_finite_float_parameter(node, "cluster_voxel_size_m"),
            min_points=int(_parameter_value(node, "cluster_min_points")),
            max_points=int(_parameter_value(node, "cluster_max_points")),
            min_diagonal_m=_finite_float_parameter(node, "cluster_min_diagonal_m"),
            max_diagonal_m=_finite_float_parameter(node, "cluster_max_diagonal_m"),
            marker_min_edge_m=_finite_float_parameter(node, "cluster_marker_min_edge_m"),
            marker_alpha=_finite_float_parameter(node, "cluster_marker_alpha"),
        ),
        ecc_config=EccConfig.normalized(
            enabled=enhancer_tag == "ecc",
            max_iterations=int(_parameter_value(node, "ecc_max_iterations")),
            eps=_finite_float_parameter(node, "ecc_eps"),
            gauss_filter_size=int(_parameter_value(node, "ecc_gauss_filter_size")),
            update_every_n_frames=int(_parameter_value(node, "ecc_update_every_n_frames")),
        ),
        stereo_config=StereoSgbmConfig.normalized(
            min_disparity=int(_parameter_value(node, "min_disparity")),
            num_disparities=int(_parameter_value(node, "num_disparities")),
            block_size=int(_parameter_value(node, "block_size")),
            p1=int(_parameter_value(node, "p1")),
            p2=int(_parameter_value(node, "p2")),
            pre_filter_cap=int(_parameter_value(node, "pre_filter_cap")),
            uniqueness_ratio=int(_parameter_value(node, "uniqueness_ratio")),
            speckle_window_size=int(_parameter_value(node, "speckle_window_size")),
            speckle_range=int(_parameter_value(node, "speckle_range")),
            disp12_max_diff=int(_parameter_value(node, "disp12_max_diff")),
        ),
    )


def validate_stereo_node_parameters(params: StereoNodeParameters) -> None:
    """Reject missing graph, frame, and workspace configuration before wiring."""
    required = {
        "left_camera_info_topic": params.left_camera_info_topic,
        "disparity_topic": params.disparity_topic,
        "depth_topic": params.depth_topic,
        "point_cloud_topic": params.point_cloud_topic,
        "octomap_point_cloud_topic": params.octomap_point_cloud_topic,
        "point_cloud_frame": params.point_cloud_frame,
        "workspace_frame": params.workspace_config.workspace_frame,
        "robot_base_frame": params.workspace_config.robot_base_frame,
        "table_plane_coeffs_topic": params.table_plane_coeffs_topic,
    }
    if params.use_external_disparity:
        required["external_disparity_topic"] = params.external_disparity_topic
    else:
        required.update(
            {
                "left_image_topic": params.left_image_topic,
                "right_image_topic": params.right_image_topic,
                "left_image_frame_id": params.left_image_frame_id,
                "right_image_frame_id": params.right_image_frame_id,
                "local_stereo_frame_id": params.local_stereo_frame_id,
            }
        )
    if params.use_detection_bbox_roi:
        required["detection_bbox_stamped_topic"] = params.detection_bbox_stamped_topic
    if params.publish_table_plane_marker:
        required["table_plane_marker_topic"] = params.table_plane_marker_topic
    if params.publish_cluster_markers:
        required["cluster_marker_topic"] = params.cluster_marker_topic
    require_non_empty(required, context="stereo perception")
    frame_ids = {
        "point_cloud_frame": params.point_cloud_frame,
        "workspace_frame": params.workspace_config.workspace_frame,
        "robot_base_frame": params.workspace_config.robot_base_frame,
    }
    if not params.use_external_disparity:
        frame_ids["left_image_frame_id"] = params.left_image_frame_id
        frame_ids["right_image_frame_id"] = params.right_image_frame_id
        frame_ids["local_stereo_frame_id"] = params.local_stereo_frame_id
    for name, frame_id in frame_ids.items():
        require_frame_id(frame_id, name, context="stereo perception")
    if (
        not params.use_external_disparity
        and params.local_stereo_frame_id != params.left_image_frame_id
    ):
        raise ValueError(
            "stereo perception: local_stereo_frame_id must equal left_image_frame_id"
        )
    require_positive(params.baseline_m, "baseline_m", context="stereo perception")
    require_positive(
        params.detection_bbox_timeout_sec,
        "detection_bbox_timeout_sec",
        context="stereo perception",
    )
    _require_finite_non_negative(
        params.min_valid_disparity_px,
        "min_valid_disparity_px",
    )
    _require_finite_non_negative(params.min_depth_m, "min_depth_m")
    if math.isfinite(params.max_depth_m):
        require_positive(params.max_depth_m, "max_depth_m", context="stereo perception")
        if params.max_depth_m <= params.min_depth_m:
            raise ValueError(
                "stereo perception: max_depth_m must be greater than min_depth_m "
                "when the upper depth bound is enabled"
            )
    elif params.max_depth_m != float("inf"):
        raise ValueError(
            "stereo perception: max_depth_m must be finite or the normalized positive infinity"
        )
    if params.sync_slop_sec != 0.0:
        raise ValueError(
            "stereo perception: sync_slop_sec must be 0 because local inputs use exact sync"
        )
    require_positive(
        params.workspace_config.robot_reach_m,
        "robot_reach_m",
        context="stereo perception",
    )
    _require_finite_non_negative(
        params.workspace_config.workspace_extra_ratio,
        "workspace_extra_ratio",
    )
    if not 0.0 <= params.table_plane_config.normal_max_tilt_deg < 90.0:
        raise ValueError(
            "stereo perception: plane_normal_max_tilt_deg must be in [0, 90)"
        )
