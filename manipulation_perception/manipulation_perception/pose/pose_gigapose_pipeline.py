"""
GigaPose coarse/refiner pipeline runner.

Responsibility:
- Prepare GigaPose runtime image/mask input.
- Run GigaPose coarse pose hypotheses.
- Pass coarse hypotheses to the refiner worker and select the final pose.

Does not own:
- ROS image or mask subscriptions.
- Worker subprocess mechanics; those belong to `pose_worker_client`.
- Generic MegaPose CLI fallback behavior.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math
from pathlib import Path
import threading

import numpy as np

from manipulation_perception.pose.pose_backend_common import LogCallback, RequestIdCounter
from manipulation_perception.pose.pose_estimation_io import select_pose_from_object_data
from manipulation_perception.pose.pose_gigapose_prewarm import GigaPosePrewarmCoordinator
from manipulation_perception.pose.pose_input_state import select_gigapose_bbox
from manipulation_perception.pose.pose_runtime_files import GigaPoseRuntimeInput
from manipulation_perception.pose.pose_worker_client import JsonWorkerClient
from manipulation_perception.pose.pose_worker_requests import (
    build_gigapose_prewarm_request,
    build_gigapose_refiner_prewarm_request,
    build_gigapose_refiner_worker_request,
    build_gigapose_worker_request,
)


@dataclass(frozen=True)
class GigaPoseWorkerRequestConfig:
    """Immutable coarse-worker dataset, model and hypothesis contract."""

    repo_dir: Path
    dinov2_repo_dir: Path | None
    root_dir: Path
    dataset_name: str
    object_id: int
    checkpoint_path: Path
    template_count: int
    max_num_dets_per_forward: int
    translation_scale: float
    num_hypotheses: int


@dataclass(frozen=True)
class GigaPoseRefinerRequestConfig:
    """Immutable MegaPose refinement model and batching contract."""

    repo_dir: Path
    root_dir: Path
    dataset_name: str
    object_id: int
    output_label: str
    megapose_models_root: Path
    bsz_images: int
    bsz_objects: int
    n_iterations: int


class GigaPosePipelineRunner:
    """
    Coordinate coarse and refinement workers for one serialized stream.

    The runner owns both ``JsonWorkerClient`` instances, their optional startup
    prewarm job and temporary runtime input files. ``stop`` is idempotent and
    prevents new requests before terminating workers; calls to inference
    methods must not be concurrent. Once prewarm starts, the first inference
    waits until both GPU backends have loaded or prewarm has failed.
    Returned 4x4 poses follow the worker contract and are interpreted by the ROS
    output adapter, which owns frame IDs and sample stamps.
    """

    def __init__(
        self,
        *,
        runtime_input: GigaPoseRuntimeInput,
        worker_client: JsonWorkerClient,
        refiner_worker_client: JsonWorkerClient,
        worker_config: GigaPoseWorkerRequestConfig,
        refiner_config: GigaPoseRefinerRequestConfig,
        min_matching_score: float = 0.0,
        verbose: bool = False,
        log_info: LogCallback | None = None,
        log_warn: LogCallback | None = None,
    ) -> None:
        if not math.isfinite(min_matching_score) or not 0.0 <= min_matching_score <= 1.0:
            raise ValueError(
                "min_matching_score должен быть конечным числом в диапазоне [0, 1]"
            )
        self.runtime_input = runtime_input
        self.worker_client = worker_client
        self.refiner_worker_client = refiner_worker_client
        self.worker_config = worker_config
        self.refiner_config = refiner_config
        self.min_matching_score = min_matching_score
        self.verbose = verbose
        self.log_info = log_info
        self.log_warn = log_warn
        self._worker_request_ids = RequestIdCounter()
        self._refiner_request_ids = RequestIdCounter()
        self._stopped = threading.Event()
        self._prewarm = GigaPosePrewarmCoordinator(
            coarse_client=worker_client,
            refiner_client=refiner_worker_client,
            log_info=log_info,
            log_warn=log_warn,
        )

    def start_prewarm(self) -> bool:
        """Load both persistent GPU backends before admitting inference."""
        if self._stopped.is_set():
            return False
        return self._prewarm.start(
            self._build_coarse_prewarm_request,
            self._build_refiner_prewarm_request,
        )

    def stop(self) -> bool:
        """Stop owned clients and report whether the prewarm writer joined."""
        self._stopped.set()
        self._prewarm.request_shutdown()
        self.worker_client.stop()
        self.refiner_worker_client.stop()
        return self._prewarm.join(timeout_sec=0.5)

    def run_coarse(
        self,
        rgb: np.ndarray,
        mask: np.ndarray | None,
        camera_k: Sequence[float],
        bbox: tuple[int, int, int, int],
    ) -> dict | None:
        """Run coarse retrieval and return hypotheses, or ``None`` on no pose."""
        if not self._wait_for_prewarm():
            return None
        image_path, mask_path = self.runtime_input.prepare(rgb, mask)
        if self._stopped.is_set():
            return None
        response = self._run_coarse_backend(
            image_path,
            mask_path,
            camera_k,
            self._select_bbox(bbox, mask, rgb.shape[:2]),
        )
        poses = response.get("poses", [])
        if not poses:
            return None
        return {
            "label": self.refiner_config.output_label,
            "poses": poses,
            "scores": response.get("scores", []),
        }

    def run_refined(
        self,
        rgb: np.ndarray,
        mask: np.ndarray | None,
        camera_k: Sequence[float],
        bbox: tuple[int, int, int, int],
    ) -> dict | None:
        """Run coarse retrieval plus refinement and return the selected pose."""
        if not self._wait_for_prewarm():
            return None
        image_path, mask_path = self.runtime_input.prepare(rgb, mask)
        if self._stopped.is_set():
            return None
        response = self._run_coarse_backend(
            image_path,
            mask_path,
            camera_k,
            self._select_bbox(bbox, mask, rgb.shape[:2]),
        )
        poses = response.get("poses", [])
        if not poses:
            if self.log_warn is not None:
                self.log_warn("GigaPose coarse-stage не вернул гипотезы, пропускаю кадр.")
            return None
        matching_score = self._matching_score_or_none(
            response.get("scores"),
            pose_count=len(poses),
        )
        if matching_score is None:
            return None
        if matching_score < self.min_matching_score:
            self._log_quality_rejection(
                actual=f"{matching_score:.6f}",
                reason="max matching score ниже порога",
            )
            return None

        coarse_estimates = {
            "poses": poses,
            "scores": response.get("scores", []),
            "elapsed_sec": response.get("elapsed_sec", 0.0),
        }
        if self._stopped.is_set():
            return None
        refiner_response = self._request_refiner_inference(image_path, camera_k, coarse_estimates)
        pose_data = select_pose_from_object_data(
            refiner_response.get("object_data", []),
            self.refiner_config.output_label,
        )
        if pose_data is not None and self.verbose and self.log_info is not None:
            self.log_info(f"GigaPose refiner timing: {refiner_response.get('timing', {})}")
        return pose_data

    def _matching_score_or_none(
        self,
        scores: object,
        *,
        pose_count: int,
    ) -> float | None:
        if (
            not isinstance(scores, Sequence)
            or isinstance(scores, (str, bytes))
            or len(scores) == 0
        ):
            self._log_quality_rejection(
                actual="unavailable",
                reason="coarse scores отсутствуют",
            )
            return None
        if len(scores) != pose_count:
            self._log_quality_rejection(
                actual=f"count={len(scores)}",
                reason=f"число coarse scores не совпадает с poses ({pose_count})",
            )
            return None
        try:
            score_values = tuple(float(score) for score in scores)
        except (TypeError, ValueError, OverflowError):
            self._log_quality_rejection(
                actual="malformed",
                reason="coarse scores не являются числами",
            )
            return None
        if not all(math.isfinite(score) for score in score_values):
            self._log_quality_rejection(
                actual="non-finite",
                reason="coarse scores содержат NaN или infinity",
            )
            return None
        return max(score_values)

    def _log_quality_rejection(self, *, actual: str, reason: str) -> None:
        if self.log_warn is None:
            return
        self.log_warn(
            "GigaPose coarse quality gate отклонил кадр: "
            f"actual={actual}, threshold={self.min_matching_score:.6f}, "
            f"reason={reason}."
        )

    def _select_bbox(
        self,
        detector_bbox: tuple[int, int, int, int],
        mask: np.ndarray | None,
        image_shape: tuple[int, int],
    ) -> tuple[int, int, int, int]:
        selected_bbox = select_gigapose_bbox(detector_bbox, mask, image_shape)
        if (
            self.verbose
            and selected_bbox != detector_bbox
            and self.log_info is not None
        ):
            self.log_info(
                f"GigaPose использует bbox из mask: {selected_bbox}, "
                f"detector_bbox={detector_bbox}"
            )
        return selected_bbox

    def _run_coarse_backend(
        self,
        image_path: Path,
        mask_path: Path | None,
        camera_k: Sequence[float],
        bbox: tuple[int, int, int, int],
    ) -> dict:
        request = build_gigapose_worker_request(
            request_id=self._worker_request_ids.next_id(),
            image_path=image_path,
            mask_path=mask_path,
            camera_k=camera_k,
            bbox=bbox,
            repo_dir=self.worker_config.repo_dir,
            dinov2_repo_dir=self.worker_config.dinov2_repo_dir,
            root_dir=self.worker_config.root_dir,
            dataset_name=self.worker_config.dataset_name,
            object_id=self.worker_config.object_id,
            checkpoint_path=self.worker_config.checkpoint_path,
            template_count=self.worker_config.template_count,
            max_num_dets_per_forward=self.worker_config.max_num_dets_per_forward,
            translation_scale=self.worker_config.translation_scale,
            num_hypotheses=self.worker_config.num_hypotheses,
        )
        return self.worker_client.request(request)

    def _build_coarse_prewarm_request(self) -> dict:
        return build_gigapose_prewarm_request(
            request_id=self._worker_request_ids.next_id(),
            repo_dir=self.worker_config.repo_dir,
            dinov2_repo_dir=self.worker_config.dinov2_repo_dir,
            root_dir=self.worker_config.root_dir,
            dataset_name=self.worker_config.dataset_name,
            object_id=self.worker_config.object_id,
            checkpoint_path=self.worker_config.checkpoint_path,
            template_count=self.worker_config.template_count,
            max_num_dets_per_forward=self.worker_config.max_num_dets_per_forward,
            translation_scale=self.worker_config.translation_scale,
        )

    def _build_refiner_prewarm_request(self) -> dict:
        return build_gigapose_refiner_prewarm_request(
            request_id=self._refiner_request_ids.next_id(),
            repo_dir=self.refiner_config.repo_dir,
            root_dir=self.refiner_config.root_dir,
            dataset_name=self.refiner_config.dataset_name,
            object_id=self.refiner_config.object_id,
            megapose_models_root=self.refiner_config.megapose_models_root,
            bsz_images=self.refiner_config.bsz_images,
            bsz_objects=self.refiner_config.bsz_objects,
            n_iterations=self.refiner_config.n_iterations,
        )

    def _wait_for_prewarm(self) -> bool:
        """Block inference until startup loading finishes or shutdown begins."""
        return self._prewarm.wait_until_ready() and not self._stopped.is_set()

    def _request_refiner_inference(
        self,
        image_path: Path,
        camera_k: Sequence[float],
        coarse_estimates: dict,
    ) -> dict:
        request = build_gigapose_refiner_worker_request(
            request_id=self._refiner_request_ids.next_id(),
            image_path=image_path,
            camera_k=camera_k,
            repo_dir=self.refiner_config.repo_dir,
            root_dir=self.refiner_config.root_dir,
            dataset_name=self.refiner_config.dataset_name,
            object_id=self.refiner_config.object_id,
            output_label=self.refiner_config.output_label,
            megapose_models_root=self.refiner_config.megapose_models_root,
            bsz_images=self.refiner_config.bsz_images,
            bsz_objects=self.refiner_config.bsz_objects,
            n_iterations=self.refiner_config.n_iterations,
            coarse_estimates=coarse_estimates,
        )
        return self.refiner_worker_client.request(request)
