"""
Test the shared Python syntax policy for relative ROS frame IDs.

Responsibility: Keep task and simulator validation aligned on canonical aliases.
"""

import pytest

from franka_motion_runtime.frame_id import (
    is_canonical_relative_frame_id,
)


@pytest.mark.parametrize(
    "frame_id",
    [
        "world",
        "robot/base",
        "table_stereo_left_optical_frame",
        "cell_1/robot-2/tool.frame",
    ],
)
def test_accepts_canonical_relative_frames(frame_id: str) -> None:
    """Nested relative names remain valid when every segment is explicit."""
    assert is_canonical_relative_frame_id(frame_id)


@pytest.mark.parametrize(
    "frame_id",
    [
        None,
        42,
        "",
        "/world",
        "world/",
        "robot//base",
        "robot base",
        "robot\tbase",
        "robot\x00base",
        "robot\x7fbase",
    ],
)
def test_rejects_noncanonical_or_non_string_frames(frame_id: object) -> None:
    """Root aliases, empty segments, whitespace and controls are rejected."""
    assert not is_canonical_relative_frame_id(frame_id)
