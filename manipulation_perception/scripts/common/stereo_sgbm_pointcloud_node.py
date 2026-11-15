#!/usr/bin/env python3

"""
ROS transport adapter for local SGBM or external stereo disparity.

Responsibility:
    Own ROS parameters, input transport, TF handles, output publishers, and
    shutdown while delegating depth and point-cloud computation to helpers.

Does not own:
    Detector inference, pose estimation, or MoveIt scene updates.

The node owns parameters, subscriptions, publishers and callback shutdown. TF
policy and depth/cloud coordination are delegated so the input transport can be
replaced by a measured C++ composable backend without changing the ROS graph.
"""

from __future__ import annotations

from pathlib import Path
import sys

from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from stereo_msgs.msg import DisparityImage


def _add_source_package_root_to_path() -> None:
    for parent in Path(__file__).resolve().parents:
        if (parent / "manipulation_perception").is_dir():
            parent_text = str(parent)
            if parent_text not in sys.path:
                sys.path.insert(0, parent_text)
            return


_add_source_package_root_to_path()

from manipulation_perception.common.pipeline_diagnostics import (  # noqa: E402,I100,I101,I201
    PipelineObserver,
)
from manipulation_perception.common.ros_shutdown import (  # noqa: E402,I100,I101,I201
    should_ignore_shutdown_runtime_error,
)
from manipulation_perception.common.stereo_depth_coordinator import (  # noqa: E402,I100,I101,I201
    StereoDepthCoordinator,
)
from manipulation_perception.common.stereo_input_processor import (  # noqa: E402,I100,I101,I201
    StereoInputProcessor,
)
from manipulation_perception.common.stereo_input_transport import (  # noqa: E402,I100,I101,I201
    StereoInputTransport,
)
from manipulation_perception.common.stereo_node_parameters import (  # noqa: E402,I100,I101,I201
    declare_stereo_node_parameters,
    read_stereo_node_parameters,
    validate_stereo_node_parameters,
)
from manipulation_perception.common.stereo_ros_outputs import StereoRosOutputs  # noqa: E402,I100,I101,I201
from manipulation_perception.common.stereo_status import (  # noqa: E402,I100,I101,I201
    format_stereo_startup_status,
)
from manipulation_perception.common.stereo_transform_provider import (  # noqa: E402,I100,I101,I201
    StereoTransformProvider,
)


class StereoSgbmPointCloudNode(Node):
    """
    Wire stereo/disparity input transport to the depth coordinator.

    Callbacks are serialized by the default executor. A busy callback drops a
    new high-bandwidth sample, bounding latency and memory. Every derived output
    retains the selected source sample stamp and configured optical frame.
    """

    def __init__(self) -> None:
        super().__init__("stereo_sgbm_pointcloud")
        declare_stereo_node_parameters(self)
        parameters = read_stereo_node_parameters(self)
        validate_stereo_node_parameters(parameters)

        self._use_external_disparity = parameters.use_external_disparity
        if not self._use_external_disparity and not parameters.local_stereo_frame_id:
            raise ValueError(
                "local_stereo_frame_id должен быть непустым в локальном SGBM режиме"
            )
        self._run_every_n_frames = parameters.run_every_n_frames
        self._bridge = CvBridge()
        self._observer = PipelineObserver.create(self, "stereo_depth_pointcloud")
        self._frame_counter = 0
        self._is_processing = False
        self._shutting_down = False

        self._outputs = StereoRosOutputs.create(self, parameters)
        self._transforms = StereoTransformProvider(
            node=self,
            allow_latest_fallback=parameters.allow_latest_tf_fallback,
            logger=self.get_logger(),
            frame_index=lambda: self._frame_counter,
        )
        self._depth_coordinator = StereoDepthCoordinator(
            parameters=parameters,
            bridge=self._bridge,
            outputs=self._outputs,
            transform_lookup=self._transforms.lookup,
            clock=self.get_clock(),
            logger=self.get_logger(),
            context_ok=rclpy.ok,
            frame_index=lambda: self._frame_counter,
            on_output=self._observer.record_output_header,
            on_drop_or_skip=self._observer.metrics.record_drop_or_skip,
            begin_processing=self._observer.metrics.begin_processing,
            record_processed=self._observer.metrics.record_processed,
            on_error=self._observer.metrics.record_error,
        )
        self._input_processor = StereoInputProcessor(
            parameters=parameters,
            bridge=self._bridge,
            outputs=self._outputs,
            depth_coordinator=self._depth_coordinator,
            logger=self.get_logger(),
            record_processed=self._observer.metrics.record_processed,
            record_error=self._observer.metrics.record_error,
        )

        self._transport = StereoInputTransport(
            node=self,
            parameters=parameters,
            roi_enabled=self._depth_coordinator.roi_enabled,
            detection_callback=self._depth_coordinator.update_detection,
            pending_depth_callback=self._depth_coordinator.process_pending_depth_frames,
            stereo_callback=self._on_stereo_pair,
            disparity_callback=self._on_external_disparity,
        )

        self.get_logger().info(
            format_stereo_startup_status(
                parameters,
                roi_enabled=self._depth_coordinator.roi_enabled,
            )
        )
        if parameters.has_unknown_enhancer_tag:
            self.get_logger().warn(
                f"Неизвестный enhancer_tag='{parameters.enhancer_tag_raw}', "
                "использую fallback 'none'."
            )

    def _on_stereo_pair(
        self,
        left_message: Image,
        right_message: Image,
        camera_info: CameraInfo,
    ) -> None:
        """Process one exact-stamp local stereo and calibration sample."""
        started_at = self._begin_selected_frame()
        if started_at is None:
            return
        try:
            self._input_processor.process_stereo_pair(
                left_message,
                right_message,
                camera_info,
                frame_index=self._frame_counter,
                started_at=started_at,
            )
        finally:
            self._is_processing = False

    def _on_external_disparity(
        self,
        camera_info: CameraInfo,
        message: DisparityImage,
    ) -> None:
        """Process one exactly stamp-synchronized calibration/disparity sample."""
        started_at = self._begin_selected_frame()
        if started_at is None:
            return
        try:
            self._input_processor.process_external_disparity(
                message,
                camera_info,
                started_at=started_at,
            )
        finally:
            self._is_processing = False

    def _begin_selected_frame(self) -> float | None:
        if self._shutting_down or not rclpy.ok():
            return None
        self._observer.metrics.record_input()
        if self._is_processing:
            self._observer.metrics.record_drop_or_skip()
            return None
        self._frame_counter += 1
        if self._frame_counter % self._run_every_n_frames != 0:
            self._observer.metrics.record_drop_or_skip()
            return None
        self._is_processing = True
        return self._observer.metrics.begin_processing()

    def destroy_node(self) -> bool:
        """Detach high-bandwidth transport before destroying ROS resources."""
        self._shutting_down = True
        self._is_processing = False
        self._observer.close()
        self._depth_coordinator.close()
        self._transport.close(self)
        self._outputs.destroy(self)
        self._transforms.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the adapter in the default single-threaded executor."""
    rclpy.init(args=args)
    node: StereoSgbmPointCloudNode | None = None
    try:
        node = StereoSgbmPointCloudNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as error:
        if not should_ignore_shutdown_runtime_error(error, context_ok=rclpy.ok()):
            raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
