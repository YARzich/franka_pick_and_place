"""
Runtime backend for the MegaPose JSON worker.

Responsibility:
- Lazily load MegaPose/PyTorch dependencies.
- Keep MegaPose estimators cached across requests.
- Execute one normalized MegaPose worker request and return the JSON response payload.

Does not own:
- ROS subscriptions, publishers or parameters.
- Runtime example file creation.
- The stdin/stdout JSON-lines loop.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import time
import traceback
from typing import Any

from manipulation_perception.pose.pose_megapose_worker_inputs import (
    build_megapose_coarse_estimates,
    parse_megapose_worker_request,
)
from manipulation_perception.pose.pose_megapose_worker_outputs import (
    serialize_and_write_megapose_predictions,
)


@dataclass(frozen=True)
class MegaPoseRuntimeDependencies:
    """Late-imported MegaPose/PyTorch types injected into the runtime backend."""

    pandas: Any
    torch: Any
    local_data_dir: Path
    object_data_type: Any
    add_instance_id: Any
    transform_type: Any
    load_detections: Any
    load_observation_tensor: Any
    make_object_dataset: Any
    pandas_tensor_collection_type: Any
    named_models: dict[str, Any]
    load_named_model: Any


def load_megapose_runtime_dependencies() -> MegaPoseRuntimeDependencies:
    """Import heavy inference dependencies only inside the worker process."""
    import pandas as pd
    import torch
    from megapose.config import LOCAL_DATA_DIR
    from megapose.datasets.scene_dataset import ObjectData
    from megapose.inference.utils import add_instance_id
    from megapose.lib3d.transform import Transform
    from megapose.scripts.run_inference_on_example import (
        load_detections,
        load_observation_tensor,
        make_object_dataset,
    )
    from megapose.utils.load_model import NAMED_MODELS, load_named_model
    from megapose.utils.tensor_collection import PandasTensorCollection

    return MegaPoseRuntimeDependencies(
        pandas=pd,
        torch=torch,
        local_data_dir=LOCAL_DATA_DIR,
        object_data_type=ObjectData,
        add_instance_id=add_instance_id,
        transform_type=Transform,
        load_detections=load_detections,
        load_observation_tensor=load_observation_tensor,
        make_object_dataset=make_object_dataset,
        pandas_tensor_collection_type=PandasTensorCollection,
        named_models=NAMED_MODELS,
        load_named_model=load_named_model,
    )


def configure_megapose_worker_logging() -> None:
    """Reduce third-party worker logs so stdout remains valid JSON-lines protocol."""
    from megapose.utils.logging import set_logging_level

    set_logging_level("warning")


class MegaPoseRuntimeBackend:
    """
    Cache CUDA estimators and execute serialized JSON worker requests.

    Cache identity includes model, mesh dataset, image batch size and renderer
    worker count. Renderer/CUDA resources are process-owned and released at
    worker exit. ``run_request`` catches all inference failures because no Python
    exception may corrupt the stdout request/response protocol.
    """

    def __init__(self, dependencies: MegaPoseRuntimeDependencies | None = None) -> None:
        self._dependencies = dependencies
        self._estimators: dict[tuple[str, str, int, int], Any] = {}

    @property
    def dependencies(self) -> MegaPoseRuntimeDependencies:
        """Return injected dependencies, importing them on first use."""
        if self._dependencies is None:
            self._dependencies = load_megapose_runtime_dependencies()
        return self._dependencies

    def run_request(self, request: dict) -> dict:
        """Execute one request and return a success or serialized failure payload."""
        request_id = request.get("id")
        started_at = time.monotonic()

        try:
            deps = self.dependencies
            parsed = parse_megapose_worker_request(
                request,
                local_data_dir=deps.local_data_dir,
                named_models=deps.named_models,
                env=os.environ,
            )
            self._debug(parsed.debug, started_at, "load_observation")
            observation = deps.load_observation_tensor(
                parsed.example_dir,
                load_depth=bool(parsed.model_info["requires_depth"]),
            ).cuda()

            self._debug(parsed.debug, started_at, "build_inputs")
            coarse_estimates = build_megapose_coarse_estimates(
                parsed.coarse_estimates,
                object_label=parsed.object_label,
                torch_module=deps.torch,
                pandas_module=deps.pandas,
                tensor_collection_type=deps.pandas_tensor_collection_type,
                add_instance_id_fn=deps.add_instance_id,
            )
            detections = None
            if coarse_estimates is None:
                detections = deps.load_detections(parsed.example_dir).cuda()

            self._debug(parsed.debug, started_at, "load_model")
            pose_estimator = self._get_pose_estimator(
                parsed.model_name,
                parsed.example_dir,
                parsed.bsz_images,
                parsed.renderer_workers,
            )
            self._debug(parsed.debug, started_at, "run_pipeline")

            output, extra_data = pose_estimator.run_inference_pipeline(
                observation,
                detections=detections,
                coarse_estimates=coarse_estimates,
                **parsed.model_info["inference_parameters"],
            )
            self._debug(parsed.debug, started_at, "serialize")
            object_data = serialize_and_write_megapose_predictions(
                parsed.example_dir,
                output,
                object_data_type=deps.object_data_type,
                transform_type=deps.transform_type,
            )
            self._debug(parsed.debug, started_at, "done")

            return {
                "id": request_id,
                "ok": True,
                "object_data": object_data,
                "timing": extra_data.get("timing_str", ""),
                "elapsed_sec": time.monotonic() - started_at,
            }
        except Exception as exc:  # pragma: no cover - runtime protocol guard
            self._try_release_cuda_cache()
            return {
                "id": request_id,
                "ok": False,
                "error": str(exc),
                "traceback": traceback.format_exc(),
            }

    @staticmethod
    def _debug(enabled: bool, started_at: float, stage: str) -> None:
        if enabled:
            print(f"debug: {stage} +{time.monotonic() - started_at:.3f}s", flush=True)

    def _get_pose_estimator(
        self,
        model_name: str,
        example_dir: Path,
        bsz_images: int,
        renderer_workers: int,
    ):
        deps = self.dependencies
        cache_key = (
            model_name,
            str((example_dir / "meshes").resolve()),
            bsz_images,
            renderer_workers,
        )
        pose_estimator = self._estimators.get(cache_key)
        if pose_estimator is not None:
            return pose_estimator

        object_dataset = deps.make_object_dataset(example_dir)
        pose_estimator = deps.load_named_model(
            model_name,
            object_dataset,
            n_workers=renderer_workers,
            bsz_images=bsz_images,
        ).cuda()
        self._estimators[cache_key] = pose_estimator
        return pose_estimator

    def _try_release_cuda_cache(self) -> None:
        try:
            deps = self._dependencies
            if deps is not None and deps.torch.cuda.is_available():
                deps.torch.cuda.empty_cache()
        except Exception:
            pass
