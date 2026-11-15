"""
Shared numeric validation for backend poses and stamped pose adapters.

Responsibility:
    Reject non-finite geometry and normalize quaternions only within the
    existing unit-length tolerance before a pose enters a public message.

Does not own:
    ROS message selection, frame transforms, or backend-specific payloads.
"""

from __future__ import annotations

import math
from typing import Sequence


def finite_values(
    values: Sequence[float],
    *,
    expected_size: int,
    name: str,
) -> tuple[float, ...]:
    """Validate and copy a fixed-size sequence of finite numeric components."""
    try:
        converted = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(
            f"{name} должен содержать {expected_size} конечных чисел"
        ) from exc
    if len(converted) != expected_size or not all(math.isfinite(value) for value in converted):
        raise ValueError(
            f"{name} должен содержать {expected_size} конечных чисел"
        )
    return converted


def validated_pose_components(
    quaternion_xyzw: Sequence[float],
    translation_xyz: Sequence[float],
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """Return a unit XYZW quaternion and finite metric XYZ translation."""
    quaternion = finite_values(quaternion_xyzw, expected_size=4, name="quaternion")
    translation = finite_values(translation_xyz, expected_size=3, name="translation")
    quaternion_norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isclose(quaternion_norm, 1.0, rel_tol=1.0e-3, abs_tol=1.0e-3):
        raise ValueError("quaternion должен быть ненулевым и нормализованным")
    return tuple(value / quaternion_norm for value in quaternion), translation
