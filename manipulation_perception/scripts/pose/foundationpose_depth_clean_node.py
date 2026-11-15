#!/usr/bin/env python3


"""
ROS adapter that sanitizes depth images for FoundationPose input.

Responsibility:
- Subscribe to depth images and publish NaN/Inf-cleaned 32FC1 depth.
- Own ROS parameters, QoS, CvBridge conversion and publication.

Does not own:
- Depth generation or camera frame adaptation.
- FoundationPose execution.
- Generic depth-cleaning math, which lives in depth_geometry.py.
"""

from __future__ import annotations

from pathlib import Path
import sys

from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


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

from manipulation_perception.common.config_validation import require_non_empty  # noqa: E402
from manipulation_perception.common.depth_geometry import clean_depth_for_pose  # noqa: E402
from manipulation_perception.common.ros_shutdown import (  # noqa: E402
    should_ignore_shutdown_runtime_error,
)


class FoundationPoseDepthCleanNode(Node):
    """
    Adapt stamped ROS depth to FoundationPose's finite 32FC1 contract.

    The node preserves the input ROS stamp and optical ``frame_id`` verbatim;
    only invalid depth samples are sanitized by the pure geometry helper.
    Callbacks run in the node's default callback group and share one ``CvBridge``.
    """

    def __init__(self) -> None:
        super().__init__("foundationpose_depth_clean")

        self.declare_parameter("input_depth_topic", "")
        self.declare_parameter("output_depth_topic", "")

        input_depth_topic = str(self.get_parameter("input_depth_topic").value)
        output_depth_topic = str(self.get_parameter("output_depth_topic").value)
        require_non_empty(
            {
                "input_depth_topic": input_depth_topic,
                "output_depth_topic": output_depth_topic,
            },
            context="pose depth adapter",
        )

        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._bridge = CvBridge()
        self._depth_sub = self.create_subscription(
            Image,
            input_depth_topic,
            self._on_depth,
            reliable_qos,
        )
        self._depth_pub = self.create_publisher(Image, output_depth_topic, reliable_qos)

        self.get_logger().info(
            "FoundationPose depth cleaner запущен: "
            f"input={input_depth_topic}, output={output_depth_topic}"
        )

    def _on_depth(self, msg: Image) -> None:
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:  # pragma: no cover - runtime guard
            self.get_logger().error(f"Не удалось декодировать depth для FoundationPose: {exc}")
            return

        depth_clean = clean_depth_for_pose(depth)
        depth_msg = self._bridge.cv2_to_imgmsg(depth_clean, encoding="32FC1")
        depth_msg.header = msg.header
        self._depth_pub.publish(depth_msg)


def main() -> None:
    """Run the adapter until ROS shutdown, then destroy its entities."""
    rclpy.init()
    node = FoundationPoseDepthCleanNode()
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
