"""
Shared table-camera frames, topics, modes and default poses.

Responsibility:
- Centralize table-camera topic names, frame ids and mode sets used by launch code.
- Keep Gazebo and Isaac perception wiring on the same public topic contract.

Does not own:
- Backend-specific model paths or inference parameters.
- Runtime validation of external packages and files.
"""

from __future__ import annotations

CAMERA_NAME = "table_camera"
WORLD_FRAME = "world"

VALID_MODES = {"auto", "none", "ecc", "ess", "yolo", "foundationpose", "megapose", "gigapose"}
VALID_MODES_TEXT = "megaPose | gigaPose | foundationPose | auto | none | ecc | ess | yolo"
MEGAPOSE_MODES = ("megapose", "auto", "gigapose")
POSE_ESTIMATOR_INPUT_MODES = ("foundationpose", "megapose", "auto", "gigapose")
ESS_MODES = ("ess", "yolo", "foundationpose", "megapose", "auto", "gigapose")
SGBM_MODES = ("none", "ecc")
YOLO_MODES = ("yolo", "foundationpose", "megapose", "auto", "gigapose")

TABLE_BASE_POSE = {
    "x": "0.0",
    "y": "0.0",
    "z": "1.0",
    "roll": "0.0",
    "pitch": "0.0",
    "yaw": "0.0",
}
CAMERA_POSE = {
    "x": "1.0",
    "y": "0.0",
    "z": "1.35",
    "roll": "0.0",
    "pitch": "0.7853981634",
    "yaw": "3.14159",
}
STEREO_BASELINE_M = 0.06
STEREO_LEFT_WORLD_POSE = {
    "x": "1.0",
    "y": "-0.03",
    "z": "1.35",
    "roll": CAMERA_POSE["roll"],
    "pitch": CAMERA_POSE["pitch"],
    "yaw": CAMERA_POSE["yaw"],
}
STEREO_RIGHT_WORLD_POSE = {
    "x": "1.0",
    "y": "0.03",
    "z": "1.35",
    "roll": CAMERA_POSE["roll"],
    "pitch": CAMERA_POSE["pitch"],
    "yaw": CAMERA_POSE["yaw"],
}

STEREO_LEFT_FRAME_ID = "table_stereo_left_frame"
STEREO_RIGHT_FRAME_ID = "table_stereo_right_frame"
STEREO_LEFT_OPTICAL_FRAME_ID = "table_stereo_left_optical_frame"
STEREO_RIGHT_OPTICAL_FRAME_ID = "table_stereo_right_optical_frame"

LEFT_IMAGE_TOPIC = "/table_stereo/left/image_raw"
RIGHT_IMAGE_TOPIC = "/table_stereo/right/image_raw"
LEFT_CAMERA_INFO_TOPIC = "/table_stereo/left/camera_info"
RIGHT_CAMERA_INFO_TOPIC = "/table_stereo/right/camera_info"
LEFT_RESIZED_IMAGE_TOPIC = "/table_stereo/left/image_resize"
RIGHT_RESIZED_IMAGE_TOPIC = "/table_stereo/right/image_resize"
LEFT_RESIZED_CAMERA_INFO_TOPIC = "/table_stereo/left/camera_info_resize"
RIGHT_RESIZED_CAMERA_INFO_TOPIC = "/table_stereo/right/camera_info_resize"
DISPARITY_TOPIC = "/table_stereo/disparity"
DISPARITY_FIXED_TOPIC = "/table_stereo/disparity_fixed"
ESS_DEBUG_DISPARITY_TOPIC = "/table_stereo/disparity_ess_debug"
DEPTH_TOPIC = "/table_stereo/depth"
POINT_CLOUD_TOPIC = "/table_stereo/points"
TABLE_PLANE_COEFFS_TOPIC = "/table_stereo/table_plane_coeffs"
CUBE_POSE_TOPIC = "/table_stereo/cube_pose"
CUBE_MARKER_TOPIC = "/table_stereo/cube_geometry"
DETECTION_BBOX_LEGACY_TOPIC = "/table_stereo/cube_bbox"
DETECTION_BBOX_STAMPED_TOPIC = "/table_stereo/cube_detection2d"
MEGAPOSE_OUTPUT_TOPIC = "/table_stereo/megapose/output"
FOUNDATIONPOSE_OUTPUT_TOPIC = "/table_stereo/foundationpose/output"
FOUNDATIONPOSE_SEGMENTATION_TOPIC = "/table_stereo/foundationpose/segmentation"
FOUNDATIONPOSE_DEPTH_TOPIC = "/table_stereo/foundationpose/depth_clean"
FOUNDATIONPOSE_IMAGE_TOPIC = "/table_stereo/foundationpose/image"
FOUNDATIONPOSE_CAMERA_INFO_TOPIC = "/table_stereo/foundationpose/camera_info"
