"""
Test monotonic fail-closed pose publication.

Responsibility: Reject invalid stamps and preserve generation/invalidation
ordering before asynchronous pose output reaches ROS.
"""

from vision_msgs.msg import Detection3DArray

from manipulation_perception.pose.pose_output_publisher import (
    OrderedPoseOutputPublisher,
)


def _message(*, sec=1, nanosec=0):
    message = Detection3DArray()
    message.header.stamp.sec = sec
    message.header.stamp.nanosec = nanosec
    message.header.frame_id = 'camera'
    return message


def test_output_publisher_rejects_zero_and_structurally_invalid_stamps():
    published = []
    publisher = OrderedPoseOutputPublisher(
        publish=published.append,
        shutdown_requested=lambda: False,
    )

    assert publisher.publish(_message(sec=0), invalidation=False, generation=0) is False
    assert publisher.publish(
        _message(nanosec=1_000_000_000),
        invalidation=False,
        generation=0,
    ) is False
    assert published == []


def test_output_publisher_accepts_valid_current_generation_stamp():
    published = []
    publisher = OrderedPoseOutputPublisher(
        publish=published.append,
        shutdown_requested=lambda: False,
    )

    message = _message(sec=2)
    assert publisher.publish(message, invalidation=False, generation=0) is True
    assert published == [message]
