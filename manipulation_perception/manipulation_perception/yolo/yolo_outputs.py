"""
ROS message payload builders for YOLO perception outputs.

Responsibility:
- Build the current Int32MultiArray bbox message from confirmed detections.
- Build stamped Detection2DArray bbox messages for synchronized pose and depth ROI consumers.
- Build the mono8 mask array that the ROS node converts to an Image.

Does not own:
- ROS publishers or CvBridge conversion.
- Detector inference or temporal filtering.
- The underlying bbox/mask geometry helpers.
"""

from __future__ import annotations

import copy

import numpy as np
from std_msgs.msg import Int32MultiArray
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from manipulation_perception.common.bbox_geometry import clip_half_open_bbox
from manipulation_perception.common.bbox_mask import mask_from_best_detection
from manipulation_perception.common.bbox_wire import bbox_payload_from_detection
from manipulation_perception.common.detection_candidate import DetectionCandidate


def detection_bbox_message(
    detections: list[DetectionCandidate],
    frame_shape: tuple[int, int] | tuple[int, int, int],
) -> Int32MultiArray:
    """Build the unchanged legacy inclusive integer bbox payload."""
    msg = Int32MultiArray()
    if not detections:
        msg.data = []
        return msg

    best = max(detections, key=lambda candidate: candidate.confidence)
    payload = bbox_payload_from_detection(best, frame_shape)
    msg.data = payload if payload is not None else []
    return msg


def detection_bbox_array_message(
    detections: list[DetectionCandidate],
    frame_shape: tuple[int, int] | tuple[int, int, int],
    header,
) -> Detection2DArray:
    """Build one stamped Detection2D from the highest-confidence candidate."""
    msg = Detection2DArray()
    msg.header = copy.deepcopy(header)
    if not detections:
        return msg

    best = max(detections, key=lambda candidate: candidate.confidence)
    frame_h, frame_w = frame_shape[:2]
    clipped = clip_half_open_bbox(best.xyxy, frame_w, frame_h)
    if clipped is None:
        return msg

    x1, y1, x2, y2 = clipped
    detection = Detection2D()
    detection.header = copy.deepcopy(header)
    detection.bbox.center.position.x = 0.5 * float(x1 + x2)
    detection.bbox.center.position.y = 0.5 * float(y1 + y2)
    detection.bbox.center.theta = 0.0
    detection.bbox.size_x = float(x2 - x1)
    detection.bbox.size_y = float(y2 - y1)

    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id = str(best.class_id)
    result.hypothesis.score = best.confidence
    detection.results.append(result)
    msg.detections.append(detection)
    return msg


def detection_mask_array(
    detections: list[DetectionCandidate],
    frame_shape: tuple[int, int] | tuple[int, int, int],
    fill_value: int,
) -> np.ndarray:
    """Rasterize the highest-confidence candidate as the published mono8 mask."""
    fill = int(np.clip(int(fill_value), 1, 255))
    return mask_from_best_detection(detections, frame_shape, fill)
