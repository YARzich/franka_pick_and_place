"""
Frame metadata adapters for small perception bridge nodes.

Responsibility:
- Build frame-corrected ROS message copies without owning ROS I/O.
- Preserve current fallback/force frame-id behavior for bridge adapters.

Does not own:
- TF lookup or transform computation.
- Image resizing/depth processing.
- Publisher/subscriber lifecycle.
"""

from __future__ import annotations

from copy import copy

from stereo_msgs.msg import DisparityImage


def copy_disparity_with_fixed_frame(
    msg: DisparityImage,
    fallback_frame_id: str,
    force_frame_id: str,
) -> tuple[DisparityImage, bool]:
    """
    Copy disparity metadata while enforcing a coherent stamped image frame.

    ``force_frame_id`` overrides both headers. Otherwise ``fallback_frame_id``
    fills only an empty outer frame and the embedded image inherits it. The
    returned flag reports that embedded frame repair; payload arrays are shared
    with the source message because this adapter changes metadata only.
    """
    out = DisparityImage()
    out.header = copy(msg.header)
    out.image = copy(msg.image)
    out.image.header = copy(msg.image.header)
    out.f = msg.f
    out.t = msg.t
    out.valid_window = msg.valid_window
    out.min_disparity = msg.min_disparity
    out.max_disparity = msg.max_disparity
    out.delta_d = msg.delta_d

    image_frame_was_fixed = False
    normalized_force_frame = str(force_frame_id).strip()
    normalized_fallback_frame = str(fallback_frame_id).strip()
    if normalized_force_frame:
        out.header.frame_id = normalized_force_frame
        out.image.header.frame_id = normalized_force_frame
    else:
        if not out.header.frame_id and normalized_fallback_frame:
            out.header.frame_id = normalized_fallback_frame

        if not out.image.header.frame_id:
            out.image.header.frame_id = out.header.frame_id
            image_frame_was_fixed = True

    if out.image.header.stamp.sec == 0 and out.image.header.stamp.nanosec == 0:
        out.image.header.stamp = out.header.stamp

    return out, image_frame_was_fixed
