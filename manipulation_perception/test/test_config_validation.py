"""
Test scenario-boundary validation helpers.

Responsibility: Reject empty settings, unsafe identifiers, invalid frames, and
non-positive geometry before ROS or backend resources are constructed.
"""

import pytest

from manipulation_perception.common.config_validation import (
    require_frame_id,
    require_identifier,
    require_non_empty,
    require_non_negative,
    require_positive,
)


def test_non_empty_validator_lists_missing_names():
    with pytest.raises(ValueError, match="first, second"):
        require_non_empty(
            {"second": " ", "present": "value", "first": ""},
            context="test",
        )


def test_positive_validator_rejects_zero_and_non_finite_values():
    for value in (0.0, -1.0, float("inf"), float("nan")):
        with pytest.raises(ValueError, match="reach_m"):
            require_positive(value, "reach_m", context="test")


def test_non_negative_validator_accepts_zero_and_rejects_invalid_values():
    require_non_negative(0.0, "timeout", context="test")
    require_non_negative(0.25, "timeout", context="test")

    for value in (-0.1, float("nan"), float("inf"), float("-inf")):
        with pytest.raises(ValueError, match="timeout"):
            require_non_negative(value, "timeout", context="test")


def test_identifier_validator_rejects_path_segments():
    require_identifier("object_01-v2", "object_label", context="test")
    for identifier in ("../object", " object", "object "):
        with pytest.raises(ValueError, match="object_label"):
            require_identifier(identifier, "object_label", context="test")


def test_frame_validator_follows_tf_frame_conventions():
    require_frame_id("robot/camera_optical_frame", "frame", context="test")
    for frame_id in (
        "",
        "/world",
        "camera frame",
        " camera",
        "camera ",
        "camera/",
        "camera//optical",
    ):
        with pytest.raises(ValueError, match="frame"):
            require_frame_id(frame_id, "frame", context="test")
