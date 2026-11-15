"""
Headless YOLO/SAHI detector orchestration for table-camera frames.

Responsibility:
- Choose full-frame, SAHI sliced or legacy tiled inference for one BGR frame.
- Pass normalized runtime settings into Ultralytics/SAHI model calls.
- Normalize results and apply box-area/temporal filtering to typed candidates.

Does not own:
- ROS subscriptions, publishers, CvBridge or message headers.
- Runtime control widgets, OpenCV windows or bbox/mask ROS output payloads.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from manipulation_perception.common.detection_candidate import DetectionCandidate
from manipulation_perception.yolo.yolo_model_runtime import (
    active_yolo_model,
    active_yolo_model_path,
)
from manipulation_perception.yolo.yolo_postprocess import (
    apply_temporal_filter,
    detections_from_sahi_result,
    detections_from_yolo_result,
    filter_by_box_area,
    nms_detections,
    TemporalDetectionTrack,
    tile_starts,
)


@dataclass(frozen=True)
class YoloDetectorRuntimeConfig:
    """Immutable inference, filtering and temporal-confirmation settings."""

    confidence: float
    iou: float
    device: str
    imgsz: int
    max_det: int
    active_class_ids_filter: list[int] | None
    agnostic_nms: bool
    augment: bool
    use_yolo_world: bool
    use_mask_model: bool
    sahi_slice_width: int
    sahi_slice_height: int
    sahi_overlap_ratio: float
    sahi_perform_standard_pred: bool
    sahi_auto_slice_resolution: bool
    sahi_postprocess_type: str
    sahi_postprocess_match_metric: str
    sahi_postprocess_match_threshold: float
    sahi_postprocess_class_agnostic: bool
    min_box_area_ratio: float
    max_box_area_ratio: float
    temporal_confirm_frames: int
    temporal_iou_threshold: float
    track_max_age: int


class HeadlessYoloDetector:
    """Own model handles and temporal state for typed candidate inference."""

    def __init__(
        self,
        *,
        model: Any,
        model_path: str,
        mask_model: Any | None,
        mask_model_path: str,
        sahi_model: Any | None,
        get_sliced_prediction: Any | None,
        logger: Any | None = None,
    ) -> None:
        self._model = model
        self._model_path = model_path
        self._mask_model = mask_model
        self._mask_model_path = mask_model_path
        self._sahi_model = sahi_model
        self._get_sliced_prediction = get_sliced_prediction
        self._logger = logger
        self._last_infer_shape_log: tuple[int, int, int, int, int] | None = None
        self._temporal_tracks: list[TemporalDetectionTrack] = []

    @property
    def has_sahi_model(self) -> bool:
        return self._sahi_model is not None

    def active_model_path(self, use_mask_model: bool) -> str:
        return active_yolo_model_path(
            self._model_path,
            self._mask_model_path,
            self._mask_model,
            use_mask_model,
        )

    def reset_temporal_tracks(self) -> None:
        """Forget cross-frame candidates after a model or runtime-policy change."""
        self._temporal_tracks = []

    def detect(
        self, frame_bgr: np.ndarray, config: YoloDetectorRuntimeConfig,
    ) -> list[DetectionCandidate]:
        """Infer and confirm typed candidates for one source BGR image."""
        detections = self.predict(frame_bgr, config)
        detections = filter_by_box_area(
            detections,
            frame_bgr.shape,
            config.min_box_area_ratio,
            config.max_box_area_ratio,
        )
        self._temporal_tracks, confirmed = apply_temporal_filter(
            self._temporal_tracks,
            detections,
            confirm_frames=config.temporal_confirm_frames,
            iou_threshold=config.temporal_iou_threshold,
            max_age=config.track_max_age,
            max_det=config.max_det,
        )
        return confirmed

    def predict(
        self,
        frame_bgr: np.ndarray,
        config: YoloDetectorRuntimeConfig,
    ) -> list[DetectionCandidate]:
        """Run the configured inference path without temporal confirmation."""
        if self._active_model_is_segment(config):
            return self._predict_full(frame_bgr, config)
        if (
            config.use_yolo_world
            or self._sahi_model is None
            or self._get_sliced_prediction is None
        ):
            return self._predict_sahi_legacy(frame_bgr, config)
        return self._predict_sahi(frame_bgr, config)

    def _active_model(self, config: YoloDetectorRuntimeConfig) -> Any:
        return active_yolo_model(self._model, self._mask_model, config.use_mask_model)

    def _active_model_is_segment(self, config: YoloDetectorRuntimeConfig) -> bool:
        return getattr(self._active_model(config), "task", "") == "segment"

    def _predict_full(
        self,
        frame_bgr: np.ndarray,
        config: YoloDetectorRuntimeConfig,
    ) -> list[DetectionCandidate]:
        results = self._active_model(config).predict(
            source=frame_bgr,
            conf=config.confidence,
            iou=config.iou,
            imgsz=config.imgsz,
            device=config.device,
            max_det=config.max_det,
            classes=config.active_class_ids_filter,
            agnostic_nms=config.agnostic_nms,
            augment=config.augment,
            verbose=False,
        )
        if not results:
            return []
        return detections_from_yolo_result(results[0], offset_x=0, offset_y=0)

    def _predict_sahi(
        self,
        frame_bgr: np.ndarray,
        config: YoloDetectorRuntimeConfig,
    ) -> list[DetectionCandidate]:
        h, w = frame_bgr.shape[:2]
        slice_w = max(64, min(w, config.sahi_slice_width))
        slice_h = max(64, min(h, config.sahi_slice_height))
        self._log_sahi_geometry_once(w, h, config.imgsz, slice_w, slice_h)

        try:
            self._sahi_model.confidence_threshold = config.confidence
            self._sahi_model.image_size = config.imgsz
            prediction_result = self._get_sliced_prediction(
                image=frame_bgr,
                detection_model=self._sahi_model,
                slice_height=slice_h,
                slice_width=slice_w,
                overlap_height_ratio=config.sahi_overlap_ratio,
                overlap_width_ratio=config.sahi_overlap_ratio,
                perform_standard_pred=config.sahi_perform_standard_pred,
                postprocess_type=config.sahi_postprocess_type,
                postprocess_match_metric=config.sahi_postprocess_match_metric,
                postprocess_match_threshold=(
                    config.sahi_postprocess_match_threshold
                ),
                postprocess_class_agnostic=(
                    config.sahi_postprocess_class_agnostic
                    or config.agnostic_nms
                ),
                auto_slice_resolution=config.sahi_auto_slice_resolution,
                verbose=0,
            )
        except Exception as exc:  # noqa: BLE001
            self._warn(
                "SAHI sliced prediction завершился с ошибкой, "
                f"fallback на legacy tiled loop: {exc}"
            )
            return self._predict_sahi_legacy(frame_bgr, config)

        return detections_from_sahi_result(
            prediction_result,
            active_class_ids_filter=config.active_class_ids_filter,
            max_det=config.max_det,
        )

    def _predict_sahi_legacy(
        self,
        frame_bgr: np.ndarray,
        config: YoloDetectorRuntimeConfig,
    ) -> list[DetectionCandidate]:
        h, w = frame_bgr.shape[:2]
        slice_w = max(64, min(w, config.sahi_slice_width))
        slice_h = max(64, min(h, config.sahi_slice_height))
        step_x = max(1, int(round(slice_w * (1.0 - config.sahi_overlap_ratio))))
        step_y = max(1, int(round(slice_h * (1.0 - config.sahi_overlap_ratio))))

        x_starts = tile_starts(w, slice_w, step_x)
        y_starts = tile_starts(h, slice_h, step_y)

        all_detections: list[DetectionCandidate] = []
        for y0 in y_starts:
            for x0 in x_starts:
                tile = frame_bgr[y0:y0 + slice_h, x0:x0 + slice_w]
                results = self._model.predict(
                    source=tile,
                    conf=config.confidence,
                    iou=config.iou,
                    imgsz=config.imgsz,
                    device=config.device,
                    max_det=config.max_det,
                    classes=config.active_class_ids_filter,
                    agnostic_nms=config.agnostic_nms,
                    augment=config.augment,
                    verbose=False,
                )
                if results:
                    all_detections.extend(
                        detections_from_yolo_result(results[0], offset_x=x0, offset_y=y0)
                    )

        if not all_detections:
            return []
        return nms_detections(
            all_detections,
            iou_threshold=config.iou,
            max_det=config.max_det,
            agnostic_nms=config.agnostic_nms,
        )

    def _log_sahi_geometry_once(
        self,
        frame_w: int,
        frame_h: int,
        imgsz: int,
        slice_w: int,
        slice_h: int,
    ) -> None:
        infer_shape_state = (frame_w, frame_h, imgsz, slice_w, slice_h)
        if infer_shape_state == self._last_infer_shape_log:
            return
        self._last_infer_shape_log = infer_shape_state
        if self._logger is not None:
            self._logger.info(
                "YOLO/SAHI geometry: "
                f"frame={frame_w}x{frame_h}, imgsz={imgsz}, slice={slice_w}x{slice_h}"
            )

    def _warn(self, message: str) -> None:
        if self._logger is not None:
            self._logger.warn(message)
