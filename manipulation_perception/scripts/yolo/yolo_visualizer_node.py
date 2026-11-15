#!/usr/bin/env python3


"""
ROS adapter for YOLO detection and optional debug visualization.

Responsibility:
- Own ROS parameters, subscriptions, publishers, CvBridge and callback lifecycle.
- Coordinate the headless YOLO detector for per-frame detection.
- Publish the project bbox/mask outputs and optionally show the OpenCV debug window.

Does not own:
- Parameter normalization, mutable runtime configuration or model path/device helpers.
- Detection backend orchestration, post-processing math, output payload
  building or debug drawing primitives.
- OpenCV control-panel state and mouse slider logic.
"""

from __future__ import annotations

from pathlib import Path
import signal
import sys

from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from vision_msgs.msg import Detection2DArray


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

from manipulation_perception.common.pipeline_diagnostics import PipelineObserver  # noqa: E402
from manipulation_perception.common.ros_shutdown import (  # noqa: E402
    should_ignore_shutdown_runtime_error,
)
from manipulation_perception.yolo.yolo_debug_session import YoloDebugSession  # noqa: E402
from manipulation_perception.yolo.yolo_detector import HeadlessYoloDetector  # noqa: E402
from manipulation_perception.yolo.yolo_frame_processor import YoloFrameProcessor  # noqa: E402
from manipulation_perception.yolo.yolo_model_runtime import (  # noqa: E402
    load_yolo_runtime,
    resolve_class_filter,
)
from manipulation_perception.yolo.yolo_node_parameters import (  # noqa: E402
    declare_yolo_visualizer_parameters,
    read_yolo_visualizer_parameters,
    validate_yolo_visualizer_parameters,
)
from manipulation_perception.yolo.yolo_output_adapter import YoloOutputAdapter  # noqa: E402
from manipulation_perception.yolo.yolo_runtime_controller import (  # noqa: E402
    YoloRuntimeController,
)
from manipulation_perception.yolo.yolo_status import format_yolo_startup_status  # noqa: E402


class YoloVisualizerNode(Node):
    """Thin ROS adapter for the YOLO detector and optional debug session."""

    def __init__(self) -> None:
        super().__init__("yolo_visualizer")
        self._bridge = CvBridge()
        self._shutting_down = False

        declare_yolo_visualizer_parameters(self)
        parameters = read_yolo_visualizer_parameters(self)
        validate_yolo_visualizer_parameters(parameters)
        self._observer = PipelineObserver.create(self, "yolo_detection")

        self._image_topic = parameters.image_topic
        self._model_path = parameters.model_path
        self._mask_model_path = parameters.mask_model_path
        self._class_whitelist = parameters.class_whitelist
        self._use_yolo_world = parameters.use_yolo_world
        self._open_vocab_classes = parameters.open_vocab_classes

        runtime = load_yolo_runtime(
            model_path=self._model_path,
            mask_model_path=self._mask_model_path,
            use_yolo_world=self._use_yolo_world,
            open_vocab_classes=self._open_vocab_classes,
            enable_runtime_controls=parameters.enable_runtime_controls,
            confidence=parameters.confidence,
            device=parameters.device,
            imgsz=parameters.imgsz,
            logger=self.get_logger(),
        )
        self._mask_model_available = runtime.mask_model_available
        self._class_ids_filter = resolve_class_filter(
            getattr(runtime.model, "names", {}),
            self._class_whitelist,
        )
        if self._class_whitelist and self._class_ids_filter is None:
            self.get_logger().warn(
                f"class_whitelist={self._class_whitelist} не совпал с именами модели; "
                "фильтрация классов отключена."
            )
        self._runtime_controller = YoloRuntimeController.from_parameters(
            parameters,
            available_class_ids_filter=self._class_ids_filter,
            mask_model_available=self._mask_model_available,
        )
        self._detector = HeadlessYoloDetector(
            model=runtime.model,
            model_path=self._model_path,
            mask_model=runtime.mask_model,
            mask_model_path=self._mask_model_path,
            sahi_model=runtime.sahi_model,
            get_sliced_prediction=runtime.get_sliced_prediction,
            logger=self.get_logger(),
        )
        self._debug_session = YoloDebugSession(
            window_name=parameters.window_name,
            enable_window=parameters.enable_window,
            enable_runtime_controls=parameters.enable_runtime_controls,
            show_fps=parameters.show_fps,
            runtime_controller=self._runtime_controller,
            log_info=self.get_logger().info,
        )

        self._image_sub = self.create_subscription(
            Image,
            self._image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self._bbox_pub = None
        self._bbox_stamped_pub = None
        if parameters.publish_detection_bbox:
            self._bbox_pub = self.create_publisher(
                Int32MultiArray,
                parameters.detection_bbox_topic,
                qos_profile_sensor_data,
            )
            self._bbox_stamped_pub = self.create_publisher(
                Detection2DArray,
                parameters.detection_bbox_stamped_topic,
                qos_profile_sensor_data,
            )
        self._mask_pub = None
        if parameters.publish_detection_mask:
            reliable_mask_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self._mask_pub = self.create_publisher(
                Image,
                parameters.detection_mask_topic,
                reliable_mask_qos,
            )

        self._outputs = YoloOutputAdapter(
            bridge=self._bridge,
            bbox_publisher=self._bbox_pub,
            stamped_bbox_publisher=self._bbox_stamped_pub,
            mask_publisher=self._mask_pub,
            bbox_frame_id=parameters.detection_bbox_frame_id,
            mask_fill_value=parameters.detection_mask_fill_value,
            on_output=self._observer.record_output_header,
        )
        self._frame_processor = YoloFrameProcessor(
            bridge=self._bridge,
            detector=self._detector,
            runtime_controller=self._runtime_controller,
            outputs=self._outputs,
            debug_session=self._debug_session,
            logger=self.get_logger(),
            record_input=self._observer.metrics.record_input,
            record_drop_or_skip=self._observer.metrics.record_drop_or_skip,
            begin_processing=self._observer.metrics.begin_processing,
            record_processed=self._observer.metrics.record_processed,
            record_error=self._observer.metrics.record_error,
        )

        initial_runtime_state = self._runtime_controller.state
        self.get_logger().info(
            format_yolo_startup_status(
                parameters=parameters,
                state=initial_runtime_state,
                detector=self._detector,
                class_ids_filter=self._class_ids_filter,
                controls_enabled=self._debug_session.controls_enabled,
                mask_model_available=self._mask_model_available,
            )
        )

    def request_shutdown(self) -> None:
        """Reject new image callbacks before ROS graph teardown begins."""
        self._shutting_down = True

    def _image_callback(self, msg: Image) -> None:
        if self._shutting_down or not rclpy.ok():
            return
        self._frame_processor.process(msg)

    def destroy_node(self) -> bool:
        """Close diagnostics, ROS entities and the OpenCV debug session."""
        self._shutting_down = True
        self._observer.close()
        if self._image_sub is not None:
            self.destroy_subscription(self._image_sub)
            self._image_sub = None
        if self._bbox_pub is not None:
            self.destroy_publisher(self._bbox_pub)
            self._bbox_pub = None
        if self._bbox_stamped_pub is not None:
            self.destroy_publisher(self._bbox_stamped_pub)
            self._bbox_stamped_pub = None
        if self._mask_pub is not None:
            self.destroy_publisher(self._mask_pub)
            self._mask_pub = None
        self._debug_session.close()
        return super().destroy_node()


def main(args=None) -> None:
    """Run with local signals that close callback admission before teardown."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node: YoloVisualizerNode | None = None
    try:
        node = YoloVisualizerNode()
        _install_shutdown_signal_handlers(node)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        if not should_ignore_shutdown_runtime_error(exc, context_ok=rclpy.ok()):
            raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def _install_shutdown_signal_handlers(node: YoloVisualizerNode) -> None:
    def _request_shutdown(_signum: int, _frame) -> None:
        node.request_shutdown()
        rclpy.try_shutdown()
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _request_shutdown)
    signal.signal(signal.SIGTERM, _request_shutdown)


if __name__ == "__main__":
    main()
