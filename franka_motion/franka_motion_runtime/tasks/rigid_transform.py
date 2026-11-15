"""
Provide pure REP-103 rigid-transform math for scenario task resolution.

Responsibility:
- Validate immutable frame-free rigid transforms.
- Compute relative and composed poses with XYZW quaternion conventions.

Does not own:
- Object catalogs, sequence state, TF lookup, or ROS messages.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from franka_motion_runtime.tasks.model import Pose, Quaternion, Vector3


_QUATERNION_TOLERANCE = 1.0e-6


@dataclass(frozen=True)
class RigidTransform:
    """Represent one frame-free rigid transform in REP-103 units."""

    translation: Vector3
    rotation_xyzw: Quaternion

    def __post_init__(self) -> None:
        """Reject malformed transforms before rigid composition."""
        if (
            not isinstance(self.translation, (list, tuple))
            or not isinstance(self.rotation_xyzw, (list, tuple))
            or len(self.translation) != 3
            or len(self.rotation_xyzw) != 4
        ):
            raise ValueError("rigid transform has invalid dimensions")
        raw_values = (*self.translation, *self.rotation_xyzw)
        if any(
            isinstance(value, bool) or not isinstance(value, (int, float))
            for value in raw_values
        ):
            raise TypeError("rigid transform must contain only numbers")
        translation = tuple(float(value) for value in self.translation)
        rotation = tuple(float(value) for value in self.rotation_xyzw)
        if not all(math.isfinite(value) for value in (*translation, *rotation)):
            raise ValueError("rigid transform must contain only finite values")
        norm_squared = sum(value * value for value in rotation)
        if not math.isclose(
            norm_squared,
            1.0,
            rel_tol=0.0,
            abs_tol=_QUATERNION_TOLERANCE,
        ):
            raise ValueError("rigid transform rotation must be a unit quaternion")
        object.__setattr__(self, "translation", translation)
        object.__setattr__(self, "rotation_xyzw", rotation)


def _quaternion_conjugate(quaternion: Quaternion) -> Quaternion:
    """Return the inverse of a validated unit quaternion."""
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def _quaternion_multiply_raw(left: Quaternion, right: Quaternion) -> Quaternion:
    """Multiply quaternion coefficients without normalizing vector operands."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    """Compose two XYZW quaternions and normalize accumulated roundoff."""
    product = _quaternion_multiply_raw(left, right)
    norm = math.sqrt(sum(value * value for value in product))
    if not math.isfinite(norm) or norm <= _QUATERNION_TOLERANCE:
        raise ValueError("quaternion composition produced an invalid rotation")
    return tuple(value / norm for value in product)


def _rotate_vector(quaternion: Quaternion, vector: Vector3) -> Vector3:
    """Rotate one REP-103 vector by an XYZW unit quaternion."""
    vector_quaternion: Quaternion = (*vector, 0.0)
    rotated = _quaternion_multiply_raw(
        _quaternion_multiply_raw(quaternion, vector_quaternion),
        _quaternion_conjugate(quaternion),
    )
    return (rotated[0], rotated[1], rotated[2])


def relative_pose(parent: Pose, child: Pose) -> RigidTransform:
    """Express ``child`` as a rigid transform relative to ``parent``."""
    if parent.frame_id != child.frame_id:
        raise ValueError("relative poses must use the same frame_id")
    parent_inverse = _quaternion_conjugate(parent.orientation_xyzw)
    delta = tuple(
        child.position[index] - parent.position[index] for index in range(3)
    )
    return RigidTransform(
        translation=_rotate_vector(parent_inverse, delta),
        rotation_xyzw=_quaternion_multiply(
            parent_inverse,
            child.orientation_xyzw,
        ),
    )


def compose_pose(parent: Pose, child_in_parent: RigidTransform) -> Pose:
    """Apply a relative rigid transform to one framed parent pose."""
    rotated_translation = _rotate_vector(
        parent.orientation_xyzw,
        child_in_parent.translation,
    )
    return Pose(
        frame_id=parent.frame_id,
        position=tuple(
            parent.position[index] + rotated_translation[index]
            for index in range(3)
        ),
        orientation_xyzw=_quaternion_multiply(
            parent.orientation_xyzw,
            child_in_parent.rotation_xyzw,
        ),
    )
