"""
Own ROS subscriptions and synchronizer for stereo input transport.

Responsibility:
    Construct the selected message-filter graph and deliver only synchronized
    stereo-image or disparity-plus-calibration samples to the adapter callback.

Does not own:
    Image decoding, depth estimation, TF lookup, or output publication.
"""

from __future__ import annotations

from typing import Any, Callable

from message_filters import Subscriber, TimeSynchronizer
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from stereo_msgs.msg import DisparityImage
from vision_msgs.msg import Detection2DArray


class StereoInputTransport:
    """
    Create either stereo-image or external-disparity ROS input wiring.

    External disparity is paired with CameraInfo by exact ROS timestamp before
    validation. Local left/right images and CameraInfo use one exact synchronizer.
    The transport borrows the node and callbacks. `close()`
    disconnects the synchronizer before subscriptions so no callback can observe
    partial node teardown.
    """

    def __init__(
        self,
        *,
        node: Any,
        parameters: Any,
        roi_enabled: bool,
        detection_callback: Callable,
        pending_depth_callback: Callable,
        stereo_callback: Callable,
        disparity_callback: Callable,
    ) -> None:
        self._camera_info_filter = None
        self._detection = None
        self._pending_timer = None
        if roi_enabled:
            self._detection = node.create_subscription(
                Detection2DArray,
                parameters.detection_bbox_stamped_topic,
                detection_callback,
                qos_profile_sensor_data,
            )
            pending_period_sec = max(
                0.05,
                min(0.25, parameters.detection_bbox_timeout_sec * 0.5),
            )
            self._pending_timer = node.create_timer(
                pending_period_sec,
                pending_depth_callback,
            )

        self._external_disparity = None
        self._left_image = None
        self._right_image = None
        self._synchronizer = None
        if parameters.use_external_disparity:
            self._camera_info_filter = Subscriber(
                node,
                CameraInfo,
                parameters.left_camera_info_topic,
                qos_profile=qos_profile_sensor_data,
            )
            self._external_disparity = Subscriber(
                node,
                DisparityImage,
                parameters.external_disparity_topic,
                qos_profile=qos_profile_sensor_data,
            )
            self._synchronizer = TimeSynchronizer(
                [self._camera_info_filter, self._external_disparity],
                queue_size=parameters.sync_queue_size,
            )
            self._synchronizer.registerCallback(disparity_callback)
        else:
            self._camera_info_filter = Subscriber(
                node,
                CameraInfo,
                parameters.left_camera_info_topic,
                qos_profile=qos_profile_sensor_data,
            )
            self._left_image = Subscriber(
                node,
                Image,
                parameters.left_image_topic,
                qos_profile=qos_profile_sensor_data,
            )
            self._right_image = Subscriber(
                node,
                Image,
                parameters.right_image_topic,
                qos_profile=qos_profile_sensor_data,
            )
            self._synchronizer = TimeSynchronizer(
                [self._left_image, self._right_image, self._camera_info_filter],
                queue_size=parameters.sync_queue_size,
            )
            self._synchronizer.registerCallback(stereo_callback)

    def close(self, node: Any) -> None:
        """Destroy all borrowed ROS handles in callback-safe order."""
        if self._synchronizer is not None:
            self._synchronizer.callbacks.clear()
            if hasattr(self._synchronizer, "queues"):
                self._synchronizer.queues.clear()
            if hasattr(self._synchronizer, "input_connections"):
                self._synchronizer.input_connections.clear()
            self._synchronizer = None
        if self._pending_timer is not None:
            node.destroy_timer(self._pending_timer)
            self._pending_timer = None
        if self._left_image is not None:
            node.destroy_subscription(self._left_image.sub)
            self._left_image = None
        if self._right_image is not None:
            node.destroy_subscription(self._right_image.sub)
            self._right_image = None
        if self._camera_info_filter is not None:
            node.destroy_subscription(self._camera_info_filter.sub)
            self._camera_info_filter = None
        if self._external_disparity is not None:
            node.destroy_subscription(self._external_disparity.sub)
            self._external_disparity = None
        for attribute in ("_detection",):
            subscription = getattr(self, attribute)
            if subscription is not None:
                node.destroy_subscription(subscription)
                setattr(self, attribute, None)


__all__ = ["StereoInputTransport"]
