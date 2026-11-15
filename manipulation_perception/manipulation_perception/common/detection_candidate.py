"""
Backend-neutral value types exchanged inside perception pipelines.

Responsibility:
    Define immutable detector candidates and image metadata that let ROS,
    inference, and output adapters exchange results without backend coupling.

Does not own:
    Detection inference, ROS messages, or frame synchronization.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import TypeAlias

import numpy as np


BBoxXYXY: TypeAlias = tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class DetectionCandidate:
    """
    Describe one detector result in source-image pixel coordinates.

    ``xyxy`` uses continuous half-open edge coordinates
    ``[x_min, y_min, x_max, y_max)``. It is not a pair of inclusive pixel
    indices. ROS/legacy output adapters are responsible for their historical
    integer endpoint conventions and clipping rules.

    ``mask`` is aligned with the same source image. The candidate owns an
    immutable, bytes-backed snapshot so temporal tracks cannot observe later
    backend mutations. Mask contents are excluded from value equality because
    NumPy arrays do not provide scalar equality.

    The value carries no timestamp or frame ID: the owning frame processor
    keeps it associated with the exact stamped source image until publication.
    """

    xyxy: BBoxXYXY
    confidence: float
    class_id: int
    mask: np.ndarray | None = field(default=None, compare=False, repr=False)
    class_name: str = ""

    def __post_init__(self) -> None:
        """Normalize and validate one backend result at the contract boundary."""
        if len(self.xyxy) != 4:
            raise ValueError("DetectionCandidate.xyxy must contain four coordinates")
        xyxy = tuple(float(value) for value in self.xyxy)
        if not all(math.isfinite(value) for value in xyxy):
            raise ValueError("DetectionCandidate.xyxy must be finite")
        if xyxy[2] <= xyxy[0] or xyxy[3] <= xyxy[1]:
            raise ValueError("DetectionCandidate.xyxy must have positive width and height")

        confidence = float(self.confidence)
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise ValueError("DetectionCandidate.confidence must be finite and in [0, 1]")

        object.__setattr__(self, "xyxy", xyxy)
        object.__setattr__(self, "confidence", confidence)
        object.__setattr__(self, "class_id", int(self.class_id))
        object.__setattr__(self, "class_name", str(self.class_name))
        if self.mask is not None:
            mask_array = np.asarray(self.mask)
            if mask_array.ndim != 2:
                raise ValueError("DetectionCandidate.mask must be a two-dimensional array")
            if 0 in mask_array.shape:
                raise ValueError("DetectionCandidate.mask dimensions must be positive")
            is_boolean = np.issubdtype(mask_array.dtype, np.bool_)
            is_real = np.issubdtype(mask_array.dtype, np.integer) or np.issubdtype(
                mask_array.dtype,
                np.floating,
            )
            if not (is_boolean or is_real):
                raise TypeError("DetectionCandidate.mask must use a real numeric or bool dtype")
            if not np.all(np.isfinite(mask_array)):
                raise ValueError("DetectionCandidate.mask must contain only finite values")
            contiguous_mask = np.ascontiguousarray(mask_array)
            immutable_mask = np.frombuffer(
                contiguous_mask.tobytes(order="C"),
                dtype=contiguous_mask.dtype,
            ).reshape(contiguous_mask.shape)
            object.__setattr__(self, "mask", immutable_mask)


__all__ = ["BBoxXYXY", "DetectionCandidate"]
