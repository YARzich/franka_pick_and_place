"""
Define framed geometric values used by V2 transfer intent.

Responsibility:
- Validate REP-103/REP-105 poses, translations, and unit quaternions.

Does not own:
- Object identity, recovery policy, ROS message conversion, or launch actions.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from franka_motion_runtime.frame_id import is_canonical_relative_frame_id
from franka_motion_runtime.tasks.task_validation import normalized_tuple


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


def _validate_frame(frame_id: str) -> None:
    """Require one canonical relative frame identifier."""
    if not isinstance(frame_id, str):
        raise TypeError("frame_id must be a string")
    if not is_canonical_relative_frame_id(frame_id):
        raise ValueError("frame_id must be a canonical relative frame")


@dataclass(frozen=True)
class Pose:
    """One REP-105 framed pose expressed in metres and XYZW quaternion order."""

    frame_id: str
    position: Vector3
    orientation_xyzw: Quaternion = (0.0, 0.0, 0.0, 1.0)

    def __post_init__(self) -> None:
        """Validate frame and quaternion invariants at scenario construction."""
        _validate_frame(self.frame_id)
        position = normalized_tuple(self.position, field="position", size=3)
        orientation = normalized_tuple(
            self.orientation_xyzw,
            field="orientation_xyzw",
            size=4,
        )
        object.__setattr__(self, "position", position)
        object.__setattr__(self, "orientation_xyzw", orientation)
        norm_squared = sum(value * value for value in orientation)
        if not math.isclose(norm_squared, 1.0, rel_tol=0.0, abs_tol=1.0e-6):
            raise ValueError("orientation_xyzw must be a unit quaternion")


@dataclass(frozen=True)
class TranslationTarget:
    """One REP-103 translation vector in a canonical frame."""

    frame_id: str
    vector: Vector3

    def __post_init__(self) -> None:
        """Validate the frame and normalize caller-owned numeric sequences."""
        _validate_frame(self.frame_id)
        object.__setattr__(
            self,
            "vector",
            normalized_tuple(self.vector, field="vector", size=3),
        )
