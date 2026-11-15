"""
Conversion helpers for stereo disparity ROS image payloads.

Responsibility:
- Validate a ``32FC1`` image layout carried by ``DisparityImage.image``.
- Decode little- or big-endian rows while ignoring per-row padding.
- Resolve exact stamped CameraInfo and per-sample stereo geometry.

Does not own:
- ROS subscriptions, publishers or logging.
- Disparity filtering, metric-depth conversion or calibration caching.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np

from manipulation_perception.common.depth_geometry import camera_intrinsics_from_matrices
from manipulation_perception.common.header_sync import header_key


Intrinsics = tuple[float, float, float, float]


@dataclass(frozen=True)
class ExternalDisparityGeometry:
    """Validated geometry borrowed by all outputs derived from one disparity sample."""

    intrinsics: Intrinsics
    baseline_m: float


def _positive_sample_value(value: float, fallback: float, field_name: str) -> float:
    """Use a positive sample value, or a valid fallback only for explicit zero."""
    sample_value = float(value)
    if not math.isfinite(sample_value) or sample_value < 0.0:
        raise ValueError(
            f"DisparityImage.{field_name} должен быть конечным и неотрицательным"
        )
    if sample_value > 0.0:
        return sample_value

    fallback_value = float(fallback)
    if not math.isfinite(fallback_value) or fallback_value <= 0.0:
        raise ValueError(
            f"DisparityImage.{field_name}=0 требует конечный положительный fallback"
        )
    return fallback_value


def resolve_external_disparity_geometry(
    message: Any,
    camera_info: Any,
    baseline_fallback_m: float,
) -> ExternalDisparityGeometry:
    """
    Validate one exact CameraInfo/disparity pair and resolve its stereo geometry.

    The outer disparity header, embedded image header and CameraInfo header must
    carry the same positive ROS timestamp and optical frame. CameraInfo supplies
    vertical/principal-point calibration, while positive ``message.f`` and
    ``message.t`` remain authoritative for this disparity sample. Exactly zero
    selects a validated CameraInfo/configuration fallback; negative and non-finite
    values are rejected.
    """
    disparity_key = header_key(message.header)
    image_key = header_key(message.image.header)
    camera_key = header_key(camera_info.header)
    if disparity_key is None or image_key is None or camera_key is None:
        raise ValueError(
            "DisparityImage и CameraInfo должны иметь положительный stamp "
            "и непустой frame_id"
        )
    frame_ids = (
        str(message.header.frame_id),
        str(message.image.header.frame_id),
        str(camera_info.header.frame_id),
    )
    stamps = tuple(
        (int(header.stamp.sec), int(header.stamp.nanosec))
        for header in (message.header, message.image.header, camera_info.header)
    )
    if (
        any(frame_id != frame_id.strip() for frame_id in frame_ids)
        or len(set(frame_ids)) != 1
        or any(sec < 0 or nanosec < 0 or nanosec >= 1_000_000_000 for sec, nanosec in stamps)
        or len(set(stamps)) != 1
        or disparity_key != image_key
        or disparity_key != camera_key
    ):
        raise ValueError(
            "DisparityImage.header, image.header и CameraInfo.header должны точно совпадать"
        )

    disparity_size = (int(message.image.width), int(message.image.height))
    camera_size = (int(camera_info.width), int(camera_info.height))
    if disparity_size[0] <= 0 or disparity_size[1] <= 0:
        raise ValueError("DisparityImage.image должен иметь положительные width и height")
    if camera_size != disparity_size:
        raise ValueError(
            "CameraInfo resolution не совпадает с DisparityImage.image: "
            f"{camera_size} != {disparity_size}"
        )

    camera_intrinsics = camera_intrinsics_from_matrices(camera_info.k, camera_info.p)
    if camera_intrinsics is None:
        raise ValueError(
            "CameraInfo не содержит конечную положительную pinhole calibration"
        )
    sample_fx = _positive_sample_value(message.f, camera_intrinsics[0], "f")
    baseline_m = _positive_sample_value(message.t, baseline_fallback_m, "t")
    return ExternalDisparityGeometry(
        intrinsics=(
            sample_fx,
            camera_intrinsics[1],
            camera_intrinsics[2],
            camera_intrinsics[3],
        ),
        baseline_m=baseline_m,
    )


def disparity_image_to_array(image: Any) -> np.ndarray:
    """Decode a sensor_msgs/Image-like disparity payload into a float32 array."""
    height = int(image.height)
    width = int(image.width)
    step = int(image.step)
    bytes_per_pixel = np.dtype(np.float32).itemsize
    min_step = width * bytes_per_pixel

    if height <= 0 or width <= 0:
        raise ValueError("DisparityImage.image должен иметь положительные height и width")
    if image.encoding and image.encoding != "32FC1":
        raise ValueError(
            "DisparityImage.image должен иметь "
            f"encoding='32FC1', получено {image.encoding!r}"
        )
    if step < min_step:
        raise ValueError(
            f"DisparityImage.image.step слишком мал: {step}, ожидается минимум {min_step}"
        )
    if step % bytes_per_pixel != 0:
        raise ValueError(
            f"DisparityImage.image.step должен делиться на {bytes_per_pixel}: {step}"
        )

    expected_size = step * height
    data = bytes(image.data)
    if len(data) < expected_size:
        raise ValueError(
            f"DisparityImage.image.data короче, чем height*step: {len(data)} < {expected_size}"
        )

    dtype = np.dtype(">f4" if image.is_bigendian else "<f4")
    row_values = step // bytes_per_pixel
    rows = np.frombuffer(data[:expected_size], dtype=dtype).reshape(height, row_values)
    return rows[:, :width].astype(np.float32, copy=True)
