"""
Header key helpers for synchronized perception inputs.

Responsibility:
- Convert stamped ROS message headers into strict cache keys.
- Reject missing or non-positive stamps before they enter synchronized caches.
- Compare perception inputs by exact stamp and frame_id.
- Provide small utilities shared by pose and stereo ROI buffers.

Does not own:
- ROS subscriptions, publishers or message_filters setup.
- TF frame transforms or semantic frame compatibility decisions.
- Per-node cache lifetime policies.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from manipulation_perception.common.config_validation import is_valid_frame_id


@dataclass(frozen=True, order=True)
class HeaderKey:
    """
    Strict identity of one stamped ROS sample.

    ``stamp_ns`` is the positive ROS timestamp expressed in nanoseconds;
    ``frame_id`` is required and participates in equality and ordering.  The
    key deliberately does not infer compatibility between related TF frames.
    """

    stamp_ns: int
    frame_id: str


def header_key(header: Any) -> HeaderKey | None:
    """Build a strict sample key, rejecting empty frames and zero stamps."""
    if header is None:
        return None

    frame_id = str(getattr(header, "frame_id", "") or "")
    if not is_valid_frame_id(frame_id):
        return None

    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None

    stamp_ns = stamp_to_nanoseconds(stamp)
    if stamp_ns <= 0:
        return None

    return HeaderKey(stamp_ns, frame_id)


def headers_match(first_header: Any, second_header: Any) -> bool:
    """Return whether two headers identify the exact same stamped sample."""
    first_key = header_key(first_header)
    if first_key is None:
        return False
    return first_key == header_key(second_header)


def stamp_to_nanoseconds(stamp: Any) -> int:
    """Convert a structurally valid ROS stamp, returning ``-1`` when invalid."""
    sec = int(getattr(stamp, "sec", -1))
    nanosec = int(getattr(stamp, "nanosec", -1))
    if sec < 0 or nanosec < 0 or nanosec >= 1_000_000_000:
        return -1
    return sec * 1_000_000_000 + nanosec
