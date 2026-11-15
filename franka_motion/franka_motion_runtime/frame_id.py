"""
Define the package-wide canonical syntax policy for relative ROS frame IDs.

Responsibility:
- Reject empty, rooted, aliased, whitespace, and control-character frame IDs.
- Keep Python scenario and simulator boundaries aligned with the C++ policy.

Does not own:
- TF availability, REP-105 frame semantics, or pose timestamp validation.
"""

from __future__ import annotations

import unicodedata


def is_canonical_relative_frame_id(value: object) -> bool:
    """Return whether value has one unambiguous relative TF frame spelling."""
    if (
        not isinstance(value, str)
        or not value
        or value.startswith("/")
        or value.endswith("/")
        or "//" in value
    ):
        return False
    return not any(
        character.isspace() or unicodedata.category(character) == "Cc"
        for character in value
    )
