"""
Validate immutable values shared by the V2 task intent modules.

Responsibility:
- Normalize numeric tuples, identifiers, and recovery-policy values.

Does not own:
- ROS messages, task composition, launch actions, or action transport.
"""

from __future__ import annotations

import math

from manipulation_msgs.msg import TransferObjectStepV2


def normalized_tuple(
    values: tuple[float, ...],
    *,
    field: str,
    size: int,
) -> tuple[float, ...]:
    """Return an immutable finite numeric tuple with one exact size."""
    if not isinstance(values, (list, tuple)) or len(values) != size:
        raise ValueError(f"{field} must contain exactly {size} values")
    if any(
        isinstance(value, bool) or not isinstance(value, (int, float))
        for value in values
    ):
        raise TypeError(f"{field} must contain only numbers")
    normalized = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in normalized):
        raise ValueError(f"{field} must contain only finite values")
    return normalized


def validate_identifier(value: str, *, field: str) -> None:
    """Apply the same token policy as the PlanningScene coordinator."""
    if not isinstance(value, str):
        raise TypeError(f"{field} must be a string")
    if (
        not value
        or "|" in value
        or any(character.isspace() or ord(character) < 32 for character in value)
    ):
        raise ValueError(f"{field} must be a non-empty token without whitespace")


def validate_recovery_policy(value: object) -> None:
    """Reject values outside the versioned transfer-step contract."""
    supported_policies = {
        TransferObjectStepV2.RECOVERY_NONE,
        TransferObjectStepV2.RECOVERY_SAFE_STOP,
        TransferObjectStepV2.RECOVERY_RETREAT,
        TransferObjectStepV2.RECOVERY_RETREAT_AND_HOME,
    }
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or value not in supported_policies
    ):
        raise ValueError("recovery_policy is outside the TransferObjectStepV2 contract")
