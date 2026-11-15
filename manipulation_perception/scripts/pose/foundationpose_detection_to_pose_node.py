#!/usr/bin/env python3


"""
ROS adapter from backend Detection3DArray output to generic object pose topics.

Responsibility:
- Own ROS parameters, TF lookup, subscriptions, publishers and shutdown.
- Transform the selected backend pose into the configured output frame.
- Publish PoseStamped and MarkerArray outputs for downstream planning/RViz.

Does not own:
- Detection3D pose selection rules.
- Bounding-box/arrow marker construction details.
- Pose-estimation backend execution.
"""

from __future__ import annotations

import copy
from pathlib import Path
import sys

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import MarkerArray


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
    require_non_negative,
    require_positive,
)
from manipulation_perception.common.pipeline_diagnostics import (  # noqa: E402
    PipelineObserver,
)
from manipulation_perception.common.ros_shutdown import (  # noqa: E402
    should_ignore_shutdown_runtime_error,
)
from manipulation_perception.pose.pose_markers import (  # noqa: E402
    delete_object_markers,
    object_geometry_markers,
)
from manipulation_perception.pose.pose_output_adapter import (  # noqa: E402
    pose_with_source_stamp,
    select_pose_from_detection_array,
)


class FoundationPoseDetectionToPoseNode(Node):
    """
    Adapt backend detections to the stable world-frame pose contract.

    TF is queried from the detection's source frame to ``output_frame`` at the
    exact detection ROS stamp; there is no latest-TF fallback. Pose output is
    volatile, while marker output is transient-local so late RViz subscribers
    receive the current geometry. Geometry/selection remain pure helpers.
    """

    def __init__(self) -> None:
        super().__init__("foundationpose_detection_to_pose")

        self.declare_parameter("input_detections_topic", "")
        self.declare_parameter("output_pose_topic", "")
        self.declare_parameter("output_marker_topic", "")
        self.declare_parameter("output_frame", "")
        self.declare_parameter("marker_namespace", "")
        self.declare_parameter("fallback_object_size_m", 0.0)
        self.declare_parameter("transform_timeout_sec", 0.05)

        detections_topic = str(self.get_parameter("input_detections_topic").value)
        output_pose_topic = str(self.get_parameter("output_pose_topic").value)
        output_marker_topic = str(self.get_parameter("output_marker_topic").value)
        self._output_frame = str(self.get_parameter("output_frame").value)
        self._marker_namespace = str(self.get_parameter("marker_namespace").value)
        self._fallback_object_size_m = float(
            self.get_parameter("fallback_object_size_m").value
        )
        require_non_empty(
            {
                "input_detections_topic": detections_topic,
                "output_pose_topic": output_pose_topic,
                "output_marker_topic": output_marker_topic,
                "output_frame": self._output_frame,
                "marker_namespace": self._marker_namespace,
            },
            context="Detection3D pose adapter",
        )
        require_positive(
            self._fallback_object_size_m,
            "fallback_object_size_m",
            context="Detection3D pose adapter",
        )
        require_frame_id(
            self._output_frame,
            "output_frame",
            context="Detection3D pose adapter",
        )
        transform_timeout_sec = float(
            self.get_parameter("transform_timeout_sec").value
        )
        require_non_negative(
            transform_timeout_sec,
            "transform_timeout_sec",
            context="Detection3D pose adapter",
        )
        self._transform_timeout = Duration(seconds=transform_timeout_sec)
        self._observer = PipelineObserver.create(self, "pose_estimation")

        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._detections_sub = self.create_subscription(
            Detection3DArray,
            detections_topic,
            self._on_detections,
            pose_qos,
        )
        self._pose_pub = self.create_publisher(PoseStamped, output_pose_topic, pose_qos)
        self._marker_pub = self.create_publisher(MarkerArray, output_marker_topic, marker_qos)

        self.get_logger().info(
            "Detection3D adapter запущен: "
            f"detections={detections_topic}, pose={output_pose_topic}, "
            f"marker={output_marker_topic}, frame={self._output_frame}"
        )

    def _on_detections(self, msg: Detection3DArray) -> None:
        self._observer.metrics.record_input()
        started_at = self._observer.metrics.begin_processing()
        try:
            self._process_detections(msg)
        finally:
            self._observer.metrics.record_processed(started_at)

    def _process_detections(self, msg: Detection3DArray) -> None:
        """Transform and publish one correlated backend detection message."""
        selection = select_pose_from_detection_array(msg)
        if selection is None:
            self._observer.metrics.record_drop_or_skip()
            delete_header = copy.deepcopy(msg.header)
            delete_header.frame_id = self._output_frame
            self._marker_pub.publish(
                delete_object_markers(delete_header, self._marker_namespace)
            )
            return

        transformed_pose = selection.pose_stamped
        if selection.pose_stamped.header.frame_id != self._output_frame:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._output_frame,
                    selection.pose_stamped.header.frame_id,
                    selection.pose_stamped.header.stamp,
                    self._transform_timeout,
                )
                transformed_pose = do_transform_pose_stamped(selection.pose_stamped, transform)
            except TransformException as exc:
                self._observer.metrics.record_error()
                self.get_logger().warn(
                    "Не удалось преобразовать позу 6D backend "
                    f"из {selection.pose_stamped.header.frame_id} в {self._output_frame}: {exc}"
                )
                return

        transformed_pose = pose_with_source_stamp(
            transformed_pose,
            selection.pose_stamped.header,
            self._output_frame,
        )

        marker_array = object_geometry_markers(
            transformed_pose,
            selection.detection,
            self._fallback_object_size_m,
            self._marker_namespace,
        )

        self._pose_pub.publish(transformed_pose)
        self._marker_pub.publish(marker_array)
        self._observer.record_output_header(transformed_pose.header)

    def destroy_node(self) -> None:
        """Stop diagnostic publication before destroying ROS entities."""
        self._observer.close()
        self._tf_buffer.jump_handle.unregister()
        super().destroy_node()


def main() -> None:
    """Run the detection adapter and close diagnostics before ROS shutdown."""
    rclpy.init()
    node = FoundationPoseDetectionToPoseNode()
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
