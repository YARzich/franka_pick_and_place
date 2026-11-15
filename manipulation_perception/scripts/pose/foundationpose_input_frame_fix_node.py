#!/usr/bin/env python3


"""
ROS adapter that rewrites FoundationPose RGB and CameraInfo frame IDs.

Responsibility:
- Subscribe to resized RGB/CameraInfo inputs.
- Publish the same payloads with the configured optical output frame.
- Own ROS parameters, QoS and publisher/subscriber wiring.

Does not own:
- Image resizing or camera calibration generation.
- FoundationPose execution.
- TF lookup or pose publication.
"""

from __future__ import annotations

from pathlib import Path
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image


def _add_source_package_root_to_path() -> None:
    for parent in Path(__file__).resolve().parents:
        if (parent / "manipulation_perception").is_dir():
            parent_text = str(parent)
            if parent_text not in sys.path:
                sys.path.insert(0, parent_text)
            common_root = parent.parent / "manipulation_perception"
            if common_root.is_dir() and str(common_root) not in sys.path:
                sys.path.insert(0, str(common_root))
            return


_add_source_package_root_to_path()

from manipulation_perception.common.config_validation import (  # noqa: E402
    require_frame_id,
    require_non_empty,
)
from manipulation_perception.common.ros_shutdown import (  # noqa: E402
    should_ignore_shutdown_runtime_error,
)


class FoundationPoseInputFrameFixNode(Node):
    """
    Rewrite RGB and calibration metadata to one configured optical frame.

    This adapter changes ``frame_id`` only and preserves each message's ROS
    timestamp and payload. It must be used only when launch configuration knows
    both inputs describe the same optical coordinate system; no TF transform or
    calibration conversion is performed here.
    """

    def __init__(self) -> None:
        super().__init__("foundationpose_input_frame_fix")

        self.declare_parameter("input_image_topic", "")
        self.declare_parameter("input_camera_info_topic", "")
        self.declare_parameter("output_image_topic", "")
        self.declare_parameter("output_camera_info_topic", "")
        self.declare_parameter("output_frame_id", "")

        input_image_topic = str(self.get_parameter("input_image_topic").value)
        input_camera_info_topic = str(self.get_parameter("input_camera_info_topic").value)
        output_image_topic = str(self.get_parameter("output_image_topic").value)
        output_camera_info_topic = str(self.get_parameter("output_camera_info_topic").value)
        self._output_frame_id = str(self.get_parameter("output_frame_id").value)
        require_non_empty(
            {
                "input_image_topic": input_image_topic,
                "input_camera_info_topic": input_camera_info_topic,
                "output_image_topic": output_image_topic,
                "output_camera_info_topic": output_camera_info_topic,
                "output_frame_id": self._output_frame_id,
            },
            context="pose input frame adapter",
        )
        require_frame_id(
            self._output_frame_id,
            "output_frame_id",
            context="pose input frame adapter",
        )

        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._image_sub = self.create_subscription(
            Image,
            input_image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self._camera_info_sub = self.create_subscription(
            CameraInfo, input_camera_info_topic, self._on_camera_info, qos_profile_sensor_data
        )
        self._image_pub = self.create_publisher(Image, output_image_topic, reliable_qos)
        self._camera_info_pub = self.create_publisher(
            CameraInfo,
            output_camera_info_topic,
            reliable_qos,
        )

        self.get_logger().info(
            "FoundationPose frame-fix запущен: "
            f"image={input_image_topic}->{output_image_topic}, "
            f"camera_info={input_camera_info_topic}->{output_camera_info_topic}, "
            f"frame_id={self._output_frame_id}"
        )

    def _on_image(self, msg: Image) -> None:
        msg.header.frame_id = self._output_frame_id
        self._image_pub.publish(msg)

    def _on_camera_info(self, msg: CameraInfo) -> None:
        msg.header.frame_id = self._output_frame_id
        self._camera_info_pub.publish(msg)


def main() -> None:
    """Run the metadata adapter until ROS shutdown and destroy its entities."""
    rclpy.init()
    node = FoundationPoseInputFrameFixNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        if not should_ignore_shutdown_runtime_error(exc, context_ok=rclpy.ok()):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
