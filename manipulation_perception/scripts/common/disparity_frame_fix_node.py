#!/usr/bin/env python3


"""
ROS adapter that fixes ESS DisparityImage frame metadata.

Responsibility:
- Subscribe to a DisparityImage topic and publish a frame-corrected copy.
- Preserve the existing fallback/force frame-id behavior.
- Own the ROS parameters, publisher, subscription and shutdown cleanup.

Does not own:
- Disparity generation.
- Camera calibration or TF lookup.
- Downstream depth/point-cloud processing.
"""

from __future__ import annotations

from pathlib import Path
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from stereo_msgs.msg import DisparityImage


def _add_source_package_root_to_path() -> None:
    for parent in Path(__file__).resolve().parents:
        if (parent / "manipulation_perception").is_dir():
            parent_text = str(parent)
            if parent_text not in sys.path:
                sys.path.insert(0, parent_text)
            return


_add_source_package_root_to_path()

from manipulation_perception.common.config_validation import (  # noqa: E402
    require_frame_id,
    require_non_empty,
)
from manipulation_perception.common.frame_adapters import copy_disparity_with_fixed_frame  # noqa: E402
from manipulation_perception.common.ros_shutdown import should_ignore_shutdown_runtime_error  # noqa: E402


class DisparityFrameFixNode(Node):
    """Restore the image frame contract missing from some ESS outputs."""

    def __init__(self) -> None:
        super().__init__("disparity_frame_fix")
        self._shutting_down = False

        self.declare_parameter("input_disparity_topic", "")
        self.declare_parameter("output_disparity_topic", "")
        self.declare_parameter("fallback_frame_id", "")
        self.declare_parameter("force_frame_id", "")

        input_topic = str(self.get_parameter("input_disparity_topic").value)
        output_topic = str(self.get_parameter("output_disparity_topic").value)
        self._fallback_frame_id = str(self.get_parameter("fallback_frame_id").value).strip()
        self._force_frame_id = str(self.get_parameter("force_frame_id").value).strip()
        require_non_empty(
            {
                "input_disparity_topic": input_topic,
                "output_disparity_topic": output_topic,
                "fallback_frame_id": self._fallback_frame_id,
            },
            context="disparity frame adapter",
        )
        require_frame_id(
            self._fallback_frame_id,
            "fallback_frame_id",
            context="disparity frame adapter",
        )
        if self._force_frame_id:
            require_frame_id(
                self._force_frame_id,
                "force_frame_id",
                context="disparity frame adapter",
            )

        self._publisher = self.create_publisher(DisparityImage, output_topic, qos_profile_sensor_data)
        self._subscriber = self.create_subscription(
            DisparityImage,
            input_topic,
            self._on_disparity,
            qos_profile_sensor_data,
        )
        self._fixed_count = 0

        self.get_logger().info(
            f"Disparity frame fix enabled: {input_topic} -> {output_topic}, "
            f"fallback_frame_id={self._fallback_frame_id}, force_frame_id={self._force_frame_id}"
        )

    def _on_disparity(self, msg: DisparityImage) -> None:
        if self._shutting_down or not rclpy.ok():
            return

        out, image_frame_was_fixed = copy_disparity_with_fixed_frame(
            msg,
            self._fallback_frame_id,
            self._force_frame_id,
        )
        if image_frame_was_fixed:
            self._fixed_count += 1
            if self._fixed_count == 1:
                self.get_logger().warn(
                    "Получен DisparityImage с пустым image.header.frame_id, "
                    "включён runtime-fix."
                )

        self._publisher.publish(out)

    def destroy_node(self) -> bool:
        """Detach I/O before base teardown so no new callbacks are queued."""
        self._shutting_down = True
        if self._subscriber is not None:
            self.destroy_subscription(self._subscriber)
            self._subscriber = None
        if self._publisher is not None:
            self.destroy_publisher(self._publisher)
            self._publisher = None
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run with rclpy signal handling and deterministic publisher teardown."""
    rclpy.init(args=args)
    node = DisparityFrameFixNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        if not should_ignore_shutdown_runtime_error(exc, context_ok=rclpy.ok()):
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
