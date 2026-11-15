"""
Test stamped frame adapters for stereo disparity messages.

Responsibility: Verify fallback and forced frame replacement without mutating the input message.
"""

from pathlib import Path
import sys

from stereo_msgs.msg import DisparityImage


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from manipulation_perception.common.frame_adapters import copy_disparity_with_fixed_frame  # noqa: E402,I100,I101,I201


def test_copy_disparity_with_fixed_frame_uses_fallback_without_mutating_input():
    msg = DisparityImage()
    msg.header.stamp.sec = 12
    msg.image.header.frame_id = ''
    msg.f = 100.0
    msg.t = 0.06

    out, image_frame_was_fixed = copy_disparity_with_fixed_frame(
        msg,
        fallback_frame_id='table_stereo_left_frame',
        force_frame_id='',
    )

    assert image_frame_was_fixed is True
    assert out.header.frame_id == 'table_stereo_left_frame'
    assert out.image.header.frame_id == 'table_stereo_left_frame'
    assert out.image.header.stamp.sec == 12
    assert out.f == 100.0
    assert out.t == 0.06
    assert msg.header.frame_id == ''
    assert msg.image.header.frame_id == ''


def test_copy_disparity_with_fixed_frame_force_overrides_both_headers():
    msg = DisparityImage()
    msg.header.frame_id = 'old_header'
    msg.image.header.frame_id = 'old_image'

    out, image_frame_was_fixed = copy_disparity_with_fixed_frame(
        msg,
        fallback_frame_id='fallback',
        force_frame_id='forced',
    )

    assert image_frame_was_fixed is False
    assert out.header.frame_id == 'forced'
    assert out.image.header.frame_id == 'forced'
    assert msg.header.frame_id == 'old_header'
    assert msg.image.header.frame_id == 'old_image'
