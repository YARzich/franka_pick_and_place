"""
Optional OpenCV visualization and runtime-control session for YOLO.

Responsibility:
    Own debug-window lifecycle, key handling, and control-panel synchronization
    while leaving headless detection behavior unchanged.

Does not own:
    Model inference, ROS transport, or detection post-processing.
"""

from __future__ import annotations

import os
import time
from typing import Any

import cv2
import numpy as np

from manipulation_perception.common.detection_candidate import DetectionCandidate
from manipulation_perception.yolo.yolo_debug_rendering import draw_detections, draw_fps
from manipulation_perception.yolo.yolo_debug_view import YoloDebugControlPanel


class YoloDebugSession:
    """
    Own an optional OpenCV window, control panel and FPS estimator.

    OpenCV resources are created only when `DISPLAY` is available. The class is
    callback-serialized by the node's single-threaded executor and must be
    closed before the node releases its publishers/subscriptions.
    """

    def __init__(
        self,
        *,
        window_name: str,
        enable_window: bool,
        enable_runtime_controls: bool,
        show_fps: bool,
        runtime_controller: Any,
        log_info: Any,
    ) -> None:
        self._window_name = window_name
        self._runtime_controller = runtime_controller
        self._show_fps = show_fps
        self._fps_ema = 0.0
        self._last_frame_time = time.perf_counter()
        self._display_available = enable_window and bool(os.environ.get("DISPLAY"))
        self._controls_enabled = enable_runtime_controls and self._display_available
        self._control_panel: YoloDebugControlPanel | None = None

        if self._display_available:
            cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self._window_name, 1280, 900)
            if self._controls_enabled:
                self._control_panel = YoloDebugControlPanel(
                    self._runtime_controller.create_controls()
                )
                cv2.setMouseCallback(self._window_name, self._control_panel.on_mouse)
        elif not enable_runtime_controls:
            log_info("Окно YOLO отключено, детекция работает в фоне без UI.")

    @property
    def controls_enabled(self) -> bool:
        """Return whether the interactive control panel is active."""
        return self._controls_enabled

    def sync_controls(self, detector: Any, logger: Any) -> None:
        """Apply UI changes and report detector state transitions."""
        if not self._controls_enabled or self._control_panel is None:
            return
        update = self._runtime_controller.apply_controls(self._control_panel.controls)
        if not update.controls_applied:
            return

        state = update.state
        if update.model_changed:
            detector.reset_temporal_tracks()
            model_path = detector.active_model_path(state.use_mask_model)
            logger.info(
                "YOLO model switched to "
                f"{'mask' if state.use_mask_model else 'bbox'}: {model_path}"
            )

        if update.state_changed:
            config = self._runtime_controller.build_detector_config()
            logger.info(
                "Runtime YOLO controls: "
                f"conf={state.confidence:.2f}, iou={state.iou:.2f}, imgsz={state.imgsz}, "
                f"max_det={state.max_det}, run_every_n_frames={state.run_every_n_frames}, "
                f"agnostic_nms={state.agnostic_nms}, augment={state.augment}, "
                f"whitelist={state.whitelist_enabled}, "
                f"sahi_slice={state.sahi_slice_width}x{state.sahi_slice_height}, "
                f"sahi_overlap={state.sahi_overlap_ratio:.2f}, "
                f"sahi_postprocess={config.sahi_postprocess_type}/"
                f"{config.sahi_postprocess_match_metric}:"
                f"{self._runtime_controller.sahi_postprocess_match_threshold:.2f}, "
                f"box_area=[{state.min_box_area_ratio:.4f}, {state.max_box_area_ratio:.4f}], "
                f"temporal_confirm_frames={state.temporal_confirm_frames}, "
                f"temporal_iou={state.temporal_iou_threshold:.2f}, "
                f"model={'mask' if state.use_mask_model else 'bbox'}, "
                f"show_masks={state.show_masks}, mask_alpha={state.mask_overlay_alpha:.2f}"
            )

    def render(self, frame_bgr: np.ndarray, detections: list[DetectionCandidate]) -> None:
        """Draw detections/FPS and display the frame when a window is active."""
        state = self._runtime_controller.state
        draw_detections(
            frame_bgr,
            detections,
            show_masks=state.show_masks,
            mask_overlay_alpha=state.mask_overlay_alpha,
        )
        if self._show_fps:
            now = time.perf_counter()
            fps = 1.0 / max(1.0e-6, now - self._last_frame_time)
            self._last_frame_time = now
            self._fps_ema = fps if self._fps_ema <= 0.0 else 0.9 * self._fps_ema + 0.1 * fps
            draw_fps(frame_bgr, self._fps_ema)
        if self._display_available:
            display_frame = frame_bgr
            if self._controls_enabled and self._control_panel is not None:
                display_frame = self._control_panel.render(frame_bgr)
            cv2.imshow(self._window_name, display_frame)
            cv2.waitKey(1)

    def close(self) -> None:
        """Release the owned OpenCV window; safe to call more than once."""
        if not self._display_available:
            return
        self._display_available = False
        try:
            cv2.destroyWindow(self._window_name)
        except cv2.error:
            pass


__all__ = ["YoloDebugSession"]
