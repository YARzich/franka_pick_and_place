"""
Test the backend-neutral detection value type.

Responsibility: Verify normalization, immutable mask ownership, and strict geometry validation.
"""

from dataclasses import FrozenInstanceError
from pathlib import Path
import sys

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.common.detection_candidate import (  # noqa: E402,I100,I101,I201
    DetectionCandidate,
)


def test_candidate_normalizes_backend_values_and_freezes_fields() -> None:
    candidate = DetectionCandidate([1, 2, 5, 7], 0.75, 3, class_name="cube")

    assert candidate.xyxy == (1.0, 2.0, 5.0, 7.0)
    assert candidate.confidence == 0.75
    assert candidate.class_id == 3
    assert candidate.class_name == "cube"
    with pytest.raises(FrozenInstanceError):
        candidate.confidence = 0.1


def test_candidate_owns_an_immutable_mask_snapshot() -> None:
    source_mask = np.array([[0.0, 1.0], [1.0, 0.0]], dtype=np.float32)

    candidate = DetectionCandidate((0, 0, 2, 2), 0.9, 1, source_mask)

    assert candidate.mask is not None
    assert candidate.mask.flags.writeable is False
    np.testing.assert_array_equal(candidate.mask, source_mask)
    source_mask[0, 0] = 42.0
    assert candidate.mask[0, 0] == 0.0
    with pytest.raises(ValueError):
        candidate.mask[0, 0] = 1.0
    with pytest.raises(ValueError):
        candidate.mask.setflags(write=True)


def test_candidate_rejects_non_xyxy_coordinate_count() -> None:
    with pytest.raises(ValueError, match="four coordinates"):
        DetectionCandidate((1, 2, 3), 0.5, 1)


@pytest.mark.parametrize(
    ('xyxy', 'confidence', 'message'),
    [
        ((1, 2, 1, 3), 0.5, 'positive width'),
        ((1, 2, float('nan'), 3), 0.5, 'finite'),
        ((1, 2, 3, 4), float('inf'), 'in \\[0, 1\\]'),
        ((1, 2, 3, 4), -0.1, 'in \\[0, 1\\]'),
    ],
)
def test_candidate_rejects_invalid_geometry_and_confidence(
    xyxy,
    confidence,
    message,
) -> None:
    with pytest.raises(ValueError, match=message):
        DetectionCandidate(xyxy, confidence, 1)


def test_candidate_rejects_non_image_mask_shape() -> None:
    with pytest.raises(ValueError, match='two-dimensional'):
        DetectionCandidate((0, 0, 1, 1), 0.5, 1, np.zeros((1, 1, 1)))
    with pytest.raises(ValueError, match='positive'):
        DetectionCandidate((0, 0, 1, 1), 0.5, 1, np.empty((0, 1)))


@pytest.mark.parametrize(
    ('mask', 'error_type', 'message'),
    [
        (np.array([['foreground']]), TypeError, 'real numeric or bool'),
        (np.array([[np.nan]], dtype=np.float32), ValueError, 'finite values'),
        (np.array([[1.0 + 0.0j]]), TypeError, 'real numeric or bool'),
    ],
)
def test_candidate_rejects_masks_that_downstream_opencv_cannot_consume(
    mask,
    error_type,
    message,
) -> None:
    with pytest.raises(error_type, match=message):
        DetectionCandidate((0, 0, 1, 1), 0.5, 1, mask)
