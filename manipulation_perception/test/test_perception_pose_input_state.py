"""
Test pure bbox, mask, and active-input state policies for pose inference.

Responsibility: Verify freshness, scaling, binary masks, and GigaPose bbox precedence.
"""

from pathlib import Path
import sys

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.pose.pose_input_state import (  # noqa: E402,I100,I101,I201
    active_binary_mask,
    active_scaled_bbox,
    normalized_binary_mask,
    select_gigapose_bbox,
)


def test_active_scaled_bbox_handles_missing_stale_and_scaled_inputs():
    assert active_scaled_bbox(None, None, 0.0, 0.5, 10, 10) is None
    assert active_scaled_bbox((1, 1, 3, 3), None, 1.0, 0.5, 10, 10) is None

    assert active_scaled_bbox((10, 10, 30, 30), (100, 100), 0.1, 0.5, 20, 20) == (2, 2, 6, 6)
    assert active_scaled_bbox((0, 0, 10, 10), None, 0.1, 0.5, 10, 10) == (0, 0, 10, 10)


def test_binary_mask_policy_squeezes_resizes_and_thresholds():
    source = np.array([[[0, 2], [3, 0]]], dtype=np.uint8)
    mask = normalized_binary_mask(source, 4, 4)

    assert mask.shape == (4, 4)
    assert mask.dtype == np.uint8
    assert set(np.unique(mask).tolist()) == {0, 1}
    assert active_binary_mask(
        source,
        enabled=False,
        age_sec=0.0,
        timeout_sec=0.5,
        frame_height=4,
        frame_width=4,
    ) is None
    assert active_binary_mask(
        source,
        enabled=True,
        age_sec=1.0,
        timeout_sec=0.5,
        frame_height=4,
        frame_width=4,
    ) is None


def test_gigapose_bbox_selection_prefers_valid_mask_bbox_with_exclusive_end():
    detector_bbox = (1, 1, 8, 8)
    mask = np.zeros((10, 10), dtype=np.uint8)
    mask[2:6, 3:7] = 1

    assert select_gigapose_bbox(detector_bbox, mask, (10, 10)) == (3, 2, 7, 6)
    assert select_gigapose_bbox(detector_bbox, np.zeros_like(mask), (10, 10)) == detector_bbox

    tiny_mask = np.zeros_like(mask)
    tiny_mask[0, 0] = 1
    assert select_gigapose_bbox(detector_bbox, tiny_mask, (10, 10), min_pixels=16) == detector_bbox
    assert select_gigapose_bbox(
        detector_bbox,
        np.ones_like(mask),
        (10, 10),
    ) == (0, 0, 10, 10)
