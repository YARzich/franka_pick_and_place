#!/usr/bin/env python3

"""
ROS adapter for stamped MegaPose/GigaPose object-pose inference.

Responsibility:
    Own ROS parameters, stamped input subscriptions, clock-jump handling,
    backend selection, result publication, and orderly worker shutdown.

Does not own:
    Detector bbox generation, stereo depth, or motion planning.

The node owns parameters, ROS subscriptions/publication, ROS clock-jump
handling and backend construction. Exact stamped input coordination and worker
lifetime are delegated to focused helpers.
"""

from __future__ import annotations

from pathlib import Path
import sys

from cv_bridge import CvBridge
import rclpy
from rclpy.clock import JumpThreshold, TimeJump
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray, Detection3DArray


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

from manipulation_perception.common.pipeline_diagnostics import (  # noqa: E402,I100,I101,I201
    PipelineObserver,
)
from manipulation_perception.common.ros_shutdown import (  # noqa: E402,I100,I101,I201
    should_ignore_shutdown_runtime_error,
)
from manipulation_perception.pose.mesh_utils import (  # noqa: E402,I100,I101,I201
    estimate_obj_size_m,
)
from manipulation_perception.pose.pose_backend_factory import (  # noqa: E402,I100,I101,I201
    create_pose_backend_runners,
)
from manipulation_perception.pose.pose_inference_runner import (  # noqa: E402,I100,I101,I201
    MegaPoseInferenceRunner,
)
from manipulation_perception.pose.pose_input_coordinator import (  # noqa: E402,I100,I101,I201
    PoseInputCoordinator,
)
from manipulation_perception.pose.pose_node_parameters import (  # noqa: E402,I100,I101,I201
    declare_pose_node_parameters,
    pose_node_parameter_warnings,
    read_pose_node_parameters,
    validate_pose_node_parameters,
)


class MegaPoseDetectionNode(Node):
    """
    Wire stamped pose inputs to a single owned backend runner.

    ROS callbacks execute on the node's default mutually exclusive group. The
    runner owns its one background inference thread and suppresses results after
    shutdown or a ROS-time generation reset.
    """

    def __init__(self) -> None:
        super().__init__("megapose_detection")

        declare_pose_node_parameters(self)
        parameters = read_pose_node_parameters(self)
        validate_pose_node_parameters(parameters)
        common = parameters.common
        megapose = parameters.megapose
        gigapose = parameters.gigapose
        self._observer = PipelineObserver.create(self, "megapose_detection")
        for warning in pose_node_parameter_warnings(parameters):
            self.get_logger().warn(warning)

        self._image_topic = common.image_topic
        self._camera_info_topic = common.camera_info_topic
        self._depth_topic = common.depth_topic
        self._bbox_topic = common.detection_bbox_stamped_topic
        self._output_topic = common.output_detections_topic
        self._mask_topic = gigapose.detection_mask_topic

        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._detections_pub = self.create_publisher(
            Detection3DArray,
            self._output_topic,
            reliable_qos,
        )

        backends = create_pose_backend_runners(
            parameters,
            Path(__file__),
            log_info=self.get_logger().info,
            log_warn=self.get_logger().warn,
            log_error=self.get_logger().error,
            log_warn_throttled=lambda message: self.get_logger().warn(
                message, throttle_duration_sec=3.0
            ),
            on_worker_restart=self._observer.metrics.record_worker_restart,
            on_worker_timeout=self._observer.metrics.record_worker_timeout,
        )
        try:
            self._runner = MegaPoseInferenceRunner(
                parameters=parameters,
                megapose_backend=backends.megapose,
                gigapose_pipeline=backends.gigapose,
                publish=self._detections_pub.publish,
                object_size_m=estimate_obj_size_m(
                    megapose.mesh_file_path,
                    megapose.mesh_units,
                ),
                logger=self.get_logger(),
                begin_processing=self._observer.metrics.begin_processing,
                record_processed=self._observer.metrics.record_processed,
                on_output=self._observer.record_output_header,
                on_error=self._observer.metrics.record_error,
                on_drop_or_skip=self._observer.metrics.record_drop_or_skip,
                on_worker_timeout=self._observer.metrics.record_worker_timeout,
                cleanup_runtime=backends.cleanup,
            )
            self._inputs = PoseInputCoordinator(
                parameters=parameters,
                bridge=CvBridge(),
                clock=self.get_clock(),
                runner=self._runner,
                logger=self.get_logger(),
                on_input=self._observer.metrics.record_input,
                on_drop_or_skip=self._observer.metrics.record_drop_or_skip,
                on_error=self._observer.metrics.record_error,
            )

            self._camera_info_sub = self.create_subscription(
                CameraInfo,
                self._camera_info_topic,
                self._inputs.on_camera_info,
                reliable_qos,
            )
            self._bbox_sub = self.create_subscription(
                Detection2DArray,
                self._bbox_topic,
                self._inputs.on_bbox,
                qos_profile_sensor_data,
            )
            self._image_sub = self.create_subscription(
                Image,
                self._image_topic,
                self._inputs.on_image,
                qos_profile_sensor_data,
            )
            self._mask_sub = None
            if common.uses_gigapose_coarse:
                self._mask_sub = self.create_subscription(
                    Image,
                    self._mask_topic,
                    self._inputs.on_mask,
                    qos_profile_sensor_data,
                )
            self._depth_sub = None
            if not common.is_depth_disabled:
                self._depth_sub = self.create_subscription(
                    Image,
                    self._depth_topic,
                    self._inputs.on_depth,
                    qos_profile_sensor_data,
                )

            jump_threshold = JumpThreshold(
                min_forward=None,
                min_backward=Duration(seconds=-1),
                on_clock_change=True,
            )
            self._clock_jump_handle = self.get_clock().create_jump_callback(
                jump_threshold,
                post_callback=self._on_clock_jump,
            )
            self.get_logger().info(
                "MegaPose detection node запущен: "
                f"image={self._image_topic}, camera_info={self._camera_info_topic}, "
                f"stamped_bbox={self._bbox_topic}, "
                f"coarse_estimator={common.coarse_estimator}, "
                f"depth_mode={common.depth_mode}, model={megapose.megapose_model}, "
                f"mesh={megapose.mesh_file_path}, data_dir={megapose.megapose_data_dir}, "
                f"persistent_worker={megapose.persistent_worker}, output={self._output_topic}"
            )
            if common.uses_gigapose_coarse and gigapose.gigapose_persistent_worker:
                backends.gigapose.start_prewarm()
        except Exception as error:
            try:
                if hasattr(self, "_runner"):
                    self._runner.shutdown(timeout_sec=2.0)
                else:
                    backends.megapose.stop()
                    if backends.gigapose.stop() is not False:
                        backends.cleanup()
            except Exception as cleanup_error:
                error.add_note(f"Pose startup cleanup failed: {cleanup_error}")
            raise

    def _on_clock_jump(self, _time_jump: TimeJump) -> None:
        self._inputs.on_clock_jump()

    def destroy_node(self) -> bool:
        """Invalidate outputs, stop backend processes and join the worker."""
        self._observer.close()
        clock_jump_handle = getattr(self, "_clock_jump_handle", None)
        if clock_jump_handle is not None:
            clock_jump_handle.unregister()
            self._clock_jump_handle = None
        if not self._runner.shutdown(timeout_sec=2.0):
            self.get_logger().warn(
                "MegaPose shutdown не подтвердил завершение всех writers/processes; "
                "runtime workspace сохранён, поздние результаты не публикуются."
            )
        return super().destroy_node()


def main() -> None:
    """Run the adapter in the default single-threaded executor."""
    rclpy.init()
    node = MegaPoseDetectionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as error:
        if not should_ignore_shutdown_runtime_error(error, context_ok=rclpy.ok()):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
