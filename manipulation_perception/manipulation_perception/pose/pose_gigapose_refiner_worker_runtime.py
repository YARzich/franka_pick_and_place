"""
Runtime backend for the GigaPose refiner JSON worker.

Responsibility:
- Lazily load MegaPose/GigaPose refiner dependencies.
- Accept model-only prewarm requests without running refinement or scoring.
- Keep refiner backends cached across requests.
- Run refinement/scoring and return the existing JSON response payload.

Does not own:
- Coarse GigaPose retrieval.
- ROS subscriptions, publishers or parameters.
- stdin/stdout worker protocol.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys
import time
import traceback
from typing import Any

from manipulation_perception.pose.pose_gigapose_refiner_models import (
    prepare_gigapose_refiner_models_dir,
    validate_gigapose_refiner_runtime_files,
)
from manipulation_perception.pose.pose_gigapose_refiner_worker_inputs import (
    gigapose_refiner_backend_cache_key,
    GigaPoseRefinerBackendConfig,
    GigaPoseRefinerWorkerRequest,
    load_refiner_rgb,
    make_gigapose_refiner_pose_estimates,
    parse_gigapose_refiner_backend_config,
    parse_gigapose_refiner_worker_request,
)
from manipulation_perception.pose.pose_gigapose_refiner_worker_outputs import (
    serialize_gigapose_refiner_prediction,
)


@dataclass(frozen=True)
class GigaPoseRefinerRuntimeDependencies:
    """Late-imported GPU/runtime types injected into the refiner backend."""

    pandas: Any
    torch: Any
    omega_conf: Any
    bop_object_dataset: Any
    object_data_type: Any
    observation_tensor_type: Any
    transform_type: Any
    tensor_collection_type: Any


def load_gigapose_refiner_runtime_dependencies() -> GigaPoseRefinerRuntimeDependencies:
    """Import heavy refiner dependencies only inside the configured worker."""
    import pandas as pd
    import torch
    from omegaconf import OmegaConf
    from megapose.datasets.bop_object_datasets import BOPObjectDataset
    from src.megapose.datasets.scene_dataset import ObjectData
    from src.megapose.inference.types import ObservationTensor
    from src.megapose.lib3d.transform import Transform
    from src.megapose.utils.tensor_collection import PandasTensorCollection

    return GigaPoseRefinerRuntimeDependencies(
        pandas=pd,
        torch=torch,
        omega_conf=OmegaConf,
        bop_object_dataset=BOPObjectDataset,
        object_data_type=ObjectData,
        observation_tensor_type=ObservationTensor,
        transform_type=Transform,
        tensor_collection_type=PandasTensorCollection,
    )


class GigaPoseRefinerBackend:
    """
    Own one cached CUDA refiner/scorer stack for a stable model config.

    Construction mutates ``sys.path`` for the external GigaPose repository and
    loads renderer workers plus CUDA models. ``run`` is serialized by the JSON
    worker protocol. ``close`` stops the owned renderer process pool on a
    best-effort basis and must be called before worker exit.
    """

    def __init__(
        self,
        request: GigaPoseRefinerBackendConfig,
        dependencies: GigaPoseRefinerRuntimeDependencies | None = None,
    ) -> None:
        self._request = request
        self._dependencies = dependencies
        self._pose_estimator = None

        self._configure_imports()
        validate_gigapose_refiner_runtime_files(request)
        self._load_runtime()

    @property
    def dependencies(self) -> GigaPoseRefinerRuntimeDependencies:
        """Return the injected dependencies, loading them on first access."""
        if self._dependencies is None:
            self._dependencies = load_gigapose_refiner_runtime_dependencies()
        return self._dependencies

    def run(self, request: GigaPoseRefinerWorkerRequest) -> dict[str, Any]:
        """Refine coarse object-to-camera poses and serialize the best result."""
        started_at = time.monotonic()
        rgb = load_refiner_rgb(request.image_path)
        observation = self.dependencies.observation_tensor_type.from_numpy(
            rgb=rgb,
            K=request.camera_matrix,
        ).cuda()
        data_tco = make_gigapose_refiner_pose_estimates(
            request.coarse_estimates,
            object_id=self._request.object_id,
            torch_module=self.dependencies.torch,
            pandas_module=self.dependencies.pandas,
            tensor_collection_type=self.dependencies.tensor_collection_type,
        )

        self.dependencies.torch.cuda.empty_cache()
        with self.dependencies.torch.no_grad():
            preds, refiner_extra_data = self._pose_estimator.forward_refiner(
                observation=observation,
                data_TCO_input=data_tco,
                n_iterations=self._request.n_iterations,
                keep_all_outputs=False,
                cuda_timer=None,
            )
            data_tco_refined = preds[f"iteration={self._request.n_iterations}"]
            data_tco_scored, scoring_extra_data = self._pose_estimator.forward_scoring_model(
                observation,
                data_tco_refined,
            )
            data_tco_final = self._pose_estimator.filter_pose_estimates(
                data_tco_scored,
                top_K=1,
                filter_field="pose_logit",
            )

        if "matching_score" in data_tco_final.infos and "pose_score" in data_tco_final.infos:
            data_tco_final.infos.pose_score = (
                data_tco_final.infos.matching_score + data_tco_final.infos.pose_score
            ) / 2.0

        object_data = serialize_gigapose_refiner_prediction(
            data_tco_final,
            output_label=request.output_label,
            object_data_type=self.dependencies.object_data_type,
            transform_type=self.dependencies.transform_type,
        )
        return {
            "id": request.request_id,
            "ok": True,
            "object_data": [object_data],
            "timing": {
                "refiner_time": refiner_extra_data.get("time"),
                "scoring_time": scoring_extra_data.get("time"),
                "scoring": scoring_extra_data.get("timing_str", ""),
            },
            "elapsed_sec": time.monotonic() - started_at,
        }

    def close(self) -> None:
        """Best-effort stop the renderer pool owned by the loaded estimator."""
        try:
            refiner_model = getattr(self._pose_estimator, "refiner_model", None)
            renderer = getattr(refiner_model, "renderer", None)
            if renderer is not None:
                renderer.stop()
        except Exception:
            pass

    def _configure_imports(self) -> None:
        for path in (str(self._request.repo_dir), str(self._request.repo_dir / "src")):
            if path not in sys.path:
                sys.path.insert(0, path)

    def _load_runtime(self) -> None:
        cfg = self.dependencies.omega_conf.load(
            self._request.repo_dir / "configs" / "model" / "refiner" / "rgb.yaml"
        )
        self.dependencies.omega_conf.set_struct(cfg, False)
        cfg.model_name = "megapose-1.0-RGB-multi-hypothesis"
        cfg.depth_refiner = None
        cfg.models_root = str(self._request.megapose_models_root)
        cfg.num_workers = self._request.num_workers
        cfg.batch_size_images = self._request.bsz_images
        cfg.batch_size_objects = self._request.bsz_objects
        cfg.n_iterations = self._request.n_iterations

        models_dir = prepare_gigapose_refiner_models_dir(self._request)
        object_dataset = self.dependencies.bop_object_dataset(Path(models_dir), format=".obj")
        self._pose_estimator = self._load_pretrained_refiner(cfg, object_dataset)
        self._move_pose_estimator_to_cuda()

    def _load_pretrained_refiner(self, cfg_refiner, object_dataset):
        import torch
        from omegaconf import OmegaConf
        from src.megapose.inference.pose_estimator import PoseEstimator
        from src.megapose.lib3d.rigid_mesh_database import MeshDataBase
        from src.megapose.panda3d_renderer.panda3d_batch_renderer import Panda3dBatchRenderer
        from src.megapose.training.pose_models_cfg import check_update_config, create_model_pose
        from src.megapose.training.training_config import TrainingConfig
        from src.megapose.utils.models_compat import change_keys_of_older_models

        model_info = {
            "coarse_run_id": "coarse-rgb-906902141",
            "refiner_run_id": "refiner-rgb-653307694",
        }
        mesh_database = MeshDataBase.from_object_ds(object_dataset)
        mesh_database_batched = mesh_database.batched()
        models_root = Path(cfg_refiner.models_root)

        def make_renderer() -> Panda3dBatchRenderer:
            return Panda3dBatchRenderer(
                object_dataset=object_dataset,
                preload_cache=False,
                split_objects=False,
                n_workers=cfg_refiner.num_workers,
            )

        def load_model(run_id: str, renderer: Panda3dBatchRenderer):
            run_dir = models_root / run_id
            cfg: TrainingConfig = OmegaConf.load(run_dir / "config.yaml")
            cfg = check_update_config(cfg)
            model = create_model_pose(cfg, renderer=renderer, mesh_db=mesh_database_batched)
            checkpoint = torch.load(run_dir / "checkpoint.pth.tar")
            state_dict = change_keys_of_older_models(checkpoint["state_dict"])
            model.load_state_dict(state_dict)
            model.cfg = cfg
            model.config = cfg
            return model

        renderer = make_renderer()
        coarse_model = load_model(model_info["coarse_run_id"], renderer)
        refiner_model = load_model(model_info["refiner_run_id"], renderer)
        return PoseEstimator(
            refiner_model=refiner_model,
            coarse_model=coarse_model,
            detector_model=None,
            depth_refiner=None,
            bsz_objects=cfg_refiner.batch_size_objects,
            bsz_images=cfg_refiner.batch_size_images,
        )

    def _move_pose_estimator_to_cuda(self) -> None:
        for model in (self._pose_estimator.coarse_model, self._pose_estimator.refiner_model):
            model.mesh_db.to("cuda")
            model.to("cuda")
            model.eval()
        self._pose_estimator.to("cuda")
        self._pose_estimator.eval()


class GigaPoseRefinerWorker:
    """
    Cache refiner backends and adapt exceptions to the JSON worker contract.

    Backend cache keys include every model-affecting request field. Requests are
    expected to arrive serially on stdin; ``close`` releases every cached GPU
    backend before process shutdown.
    """

    def __init__(
        self,
        *,
        backend_type: type[GigaPoseRefinerBackend] = GigaPoseRefinerBackend,
        dependencies: GigaPoseRefinerRuntimeDependencies | None = None,
    ) -> None:
        self._backend_type = backend_type
        self._dependencies = dependencies
        self._backends: dict[tuple[str, ...], GigaPoseRefinerBackend] = {}

    def run_request(self, request: dict[str, Any]) -> dict[str, Any]:
        """Run one request and return a response that never leaks exceptions."""
        request_id = request.get("id")
        try:
            if request.get("command") == "prewarm":
                config = parse_gigapose_refiner_backend_config(request)
                self._get_backend(config)
                return {"id": request_id, "ok": True, "prewarmed": True}
            parsed = parse_gigapose_refiner_worker_request(request)
            backend = self._get_backend(parsed)
            return backend.run(parsed)
        except Exception as exc:  # pragma: no cover - runtime protocol guard
            return {
                "id": request_id,
                "ok": False,
                "error": str(exc),
                "traceback": traceback.format_exc(),
            }

    def close(self) -> None:
        """Close and forget every cached backend owned by this worker."""
        for backend in self._backends.values():
            backend.close()
        self._backends.clear()

    def _get_backend(
        self,
        request: GigaPoseRefinerBackendConfig,
    ) -> GigaPoseRefinerBackend:
        cache_key = gigapose_refiner_backend_cache_key(request)
        backend = self._backends.get(cache_key)
        if backend is not None:
            return backend
        backend = self._backend_type(request, self._dependencies)
        self._backends[cache_key] = backend
        return backend
