"""
Own pose backend execution, worker lifetime and ordered ROS output.

Responsibility:
    Serialize accepted pose snapshots through the selected backend, manage its
    worker resources, and publish results in source-frame order.

Does not own:
    ROS subscriptions, stamped input association, or parameter declaration.
"""

from __future__ import annotations

import subprocess
from typing import Any, Callable

import numpy as np
from sensor_msgs.msg import CameraInfo

from manipulation_perception.pose.pose_detection_messages import (
    build_empty_pose_detection_array,
    build_pose_detection_array,
)
from manipulation_perception.pose.pose_inference_gate import PoseInferenceGate
from manipulation_perception.pose.pose_inference_worker import InferenceWorkerLifecycle
from manipulation_perception.pose.pose_node_parameters import PoseNodeParameters
from manipulation_perception.pose.pose_output_publisher import OrderedPoseOutputPublisher
from manipulation_perception.pose.pose_worker_requests import (
    model_uses_depth,
    select_megapose_model,
)


class MegaPoseInferenceRunner:
    """
    Run one pose inference at a time and suppress stale threaded results.

    The runner owns the backend worker thread and both backend clients. Public
    methods are called by the ROS executor; `_run_worker` executes on the owned
    worker thread. `shutdown()` prevents new starts, stops backend processes and
    joins the worker with a steady-clock timeout supplied by its lifecycle.
    Observer callbacks are borrowed, thread-safe sinks; they do not control work.
    """

    def __init__(
        self,
        *,
        parameters: PoseNodeParameters,
        megapose_backend: Any,
        gigapose_pipeline: Any,
        publish: Any,
        object_size_m: float,
        logger: Any,
        begin_processing: Callable[[], float] | None = None,
        record_processed: Callable[[float], None] | None = None,
        on_output: Callable[[Any, bool], None] | None = None,
        on_error: Callable[[], None] | None = None,
        on_drop_or_skip: Callable[[int], None] | None = None,
        on_worker_timeout: Callable[[], None] | None = None,
        cleanup_runtime: Callable[[], bool | None] | None = None,
    ) -> None:
        common = parameters.common
        megapose = parameters.megapose
        self._coarse_estimator = common.coarse_estimator
        self._megapose_model = megapose.megapose_model
        self._depth_mode = common.depth_mode
        self._process_timeout_sec = common.process_timeout_sec
        self._output_frame_id = common.output_frame_id
        self._object_label = megapose.object_label
        self._object_size_m = object_size_m
        self._score = common.score
        self._logger = logger
        self._begin_processing = begin_processing or (lambda: 0.0)
        self._record_processed = record_processed or (lambda _started_at: None)
        self._on_output = on_output or (lambda _header, _successful: None)
        self._on_error = on_error or (lambda: None)
        self._on_drop_or_skip = on_drop_or_skip or (lambda _count=1: None)
        self._on_worker_timeout = on_worker_timeout or (lambda: None)
        self._cleanup_runtime = cleanup_runtime or (lambda: None)
        self._last_logged_model: str | None = None
        self._megapose_backend = megapose_backend
        self._gigapose_pipeline = gigapose_pipeline
        self._worker = InferenceWorkerLifecycle()
        self._gate = PoseInferenceGate(
            run_every_n_frames=common.run_every_n_frames,
            min_interval_sec=common.min_inference_interval_sec,
        )
        self._ordered_output = OrderedPoseOutputPublisher(
            publish=publish,
            shutdown_requested=self._worker.is_shutdown_requested,
        )

    def shutdown_requested(self) -> bool:
        """Return whether shutdown has made all late publications invalid."""
        return self._worker.is_shutdown_requested()

    def accepts_frame(self) -> bool:
        """Apply frame decimation before an RGB frame enters the pending queue."""
        return self._gate.accepts_frame()

    def is_running(self) -> bool:
        """Return whether an inference has reserved the single-worker gate."""
        return self._gate.is_running()

    def try_start(self, now_monotonic: float) -> bool:
        """Reserve the inference gate using a steady-clock timestamp."""
        return self._gate.try_start(now_monotonic)

    def mark_start_failed(self) -> None:
        """Release a reserved gate when input decoding starts no worker."""
        self._gate.mark_done()

    def current_generation(self) -> int:
        """Return the ordered-output generation for a newly queued image."""
        return self._ordered_output.current_generation()

    def reset_for_clock_jump(self) -> None:
        """Invalidate queued/late results after a backwards ROS-time jump."""
        self._ordered_output.reset()

    def requires_depth_before_inference(self) -> bool:
        """Return whether the selected MegaPose model requires synchronized depth."""
        if self._depth_disabled() or self._uses_gigapose():
            return False
        return model_uses_depth(self._select_model(has_depth=False))

    def start(self, snapshot: Any, generation: int) -> None:
        """Validate a decoded input snapshot and start the owned worker."""
        model_name = self._select_model(snapshot.depth is not None)
        needs_depth = model_uses_depth(model_name) and not self._depth_disabled()
        if needs_depth and snapshot.depth is None:
            self._on_drop_or_skip(1)
            self._gate.mark_done()
            self.publish_invalidation(snapshot.image_header, generation)
            self._logger.warn(
                "MegaPose ожидает stereo depth для выбранной RGB-D/ICP модели.",
                throttle_duration_sec=3.0,
            )
            return

        use_depth = needs_depth and snapshot.depth is not None
        if model_name != self._last_logged_model:
            depth_status = "on" if use_depth else "off"
            self._logger.info(
                f"MegaPose inference model={model_name}, depth={depth_status}"
            )
            self._last_logged_model = model_name

        try:
            started = self._worker.start(
                target=self._run_worker,
                args=(
                    snapshot.rgb.copy(),
                    snapshot.mask.copy() if snapshot.mask is not None else None,
                    snapshot.depth.copy() if snapshot.depth is not None else None,
                    use_depth,
                    model_name,
                    snapshot.camera_info,
                    snapshot.image_header,
                    snapshot.bbox,
                    generation,
                ),
                name="megapose_inference_worker",
            )
        except RuntimeError:
            self._gate.mark_done()
            self._on_error()
            raise
        if not started:
            self._gate.mark_done()

    def publish_invalidation(self, image_header: Any, generation: int) -> None:
        """Publish an empty stamped detection unless shutdown/generation rejects it."""
        if self.shutdown_requested():
            return
        published = self._ordered_output.publish(
            build_empty_pose_detection_array(image_header, self._output_frame_id),
            invalidation=True,
            generation=generation,
        )
        if published:
            self._on_output(image_header, False)
        else:
            self._on_drop_or_skip(1)

    def shutdown(self, timeout_sec: float = 2.0) -> bool:
        """Stop backend processes and join the owned worker within `timeout_sec`."""
        self._worker.request_shutdown()
        self._megapose_backend.stop()
        prewarm_joined = self._gigapose_pipeline.stop()
        worker_joined = self._worker.join(timeout_sec=timeout_sec)
        if worker_joined and prewarm_joined is not False:
            return self._cleanup_runtime() is not False
        return False

    def _run_worker(
        self,
        rgb: np.ndarray,
        mask: np.ndarray | None,
        depth: np.ndarray | None,
        use_depth: bool,
        model_name: str,
        camera_info: CameraInfo,
        image_header: Any,
        bbox: tuple[int, int, int, int],
        generation: int,
    ) -> None:
        started_at = self._begin_processing()
        try:
            if self.shutdown_requested():
                return
            if self._uses_gigapose():
                pose_data = self._gigapose_pipeline.run_refined(
                    rgb, mask, camera_info.k, bbox
                )
            else:
                pose_data = self._megapose_backend.run_with_runtime_example(
                    rgb,
                    depth,
                    use_depth,
                    model_name,
                    camera_info.k,
                    bbox,
                )
            if self.shutdown_requested():
                return
            if pose_data is None:
                self._on_drop_or_skip(1)
                self._logger.warn(
                    "MegaPose не вернул pose для объекта.",
                    throttle_duration_sec=3.0,
                )
                self.publish_invalidation(image_header, generation)
                return
            self._publish_detection(pose_data, image_header, generation)
        except subprocess.TimeoutExpired:
            if not self.shutdown_requested():
                self._on_worker_timeout()
                self._on_error()
                self._logger.error(
                    f"MegaPose inference превысил timeout {self._process_timeout_sec:.1f} сек."
                )
                self.publish_invalidation(image_header, generation)
        except Exception as error:  # pragma: no cover - runtime guard
            if not self.shutdown_requested():
                self._on_error()
                self._logger.error(f"Ошибка MegaPose inference: {error}")
                self.publish_invalidation(image_header, generation)
        finally:
            self._gate.mark_done()
            self._record_processed(started_at)

    def _publish_detection(
        self,
        pose_data: dict,
        image_header: Any,
        generation: int,
    ) -> None:
        if self.shutdown_requested():
            return
        detection_array = build_pose_detection_array(
            pose_data=pose_data,
            image_header=image_header,
            output_frame_id=self._output_frame_id,
            object_label=self._object_label,
            object_size_m=self._object_size_m,
            score=self._score,
        )
        published = self._ordered_output.publish(
            detection_array,
            invalidation=False,
            generation=generation,
        )
        if published:
            self._on_output(image_header, True)
        else:
            self._on_drop_or_skip(1)

    def _uses_gigapose(self) -> bool:
        return self._coarse_estimator == "gigapose"

    def _depth_disabled(self) -> bool:
        return self._depth_mode in {"off", "false", "0"}

    def _select_model(self, has_depth: bool) -> str:
        return select_megapose_model(
            coarse_estimator=self._coarse_estimator,
            megapose_model=self._megapose_model,
            depth_mode=self._depth_mode,
            has_depth=has_depth,
        )


__all__ = ["MegaPoseInferenceRunner"]
