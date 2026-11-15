"""
Runtime backend for the GigaPose coarse JSON worker.

Responsibility:
- Lazily load GigaPose retrieval dependencies.
- Accept model-only prewarm requests without running retrieval.
- Keep retrieval backends cached across requests.
- Execute coarse retrieval and return the existing JSON response payload.

Does not own:
- ROS subscriptions, publishers or parameters.
- Runtime RGB/mask file creation.
- Refiner scoring or final pose selection.
- stdin/stdout worker protocol.
"""

from __future__ import annotations

from dataclasses import dataclass
import sys
import time
import traceback
from typing import Any

from manipulation_perception.pose.pose_gigapose_worker_inputs import (
    build_gigapose_batch,
    gigapose_backend_cache_key,
    GigaPoseBackendConfig,
    GigaPoseWorkerRequest,
    load_gigapose_mask,
    load_gigapose_rgb,
    parse_gigapose_backend_config,
    parse_gigapose_worker_request,
    validate_gigapose_runtime_files,
)
from manipulation_perception.pose.pose_gigapose_worker_outputs import load_gigapose_prediction


@dataclass(frozen=True)
class GigaPoseRuntimeDependencies:
    """Late-imported Hydra, tensor and CUDA dependencies for coarse retrieval."""

    pandas: Any
    torch: Any
    tensor_collection_module: Any
    instantiate: Any
    compose: Any
    initialize_config_dir: Any
    global_hydra: Any
    omega_conf: Any


def load_gigapose_runtime_dependencies() -> GigaPoseRuntimeDependencies:
    """Import heavy GigaPose dependencies only inside the worker process."""
    import pandas as pd
    import torch
    from hydra import compose, initialize_config_dir
    from hydra.core.global_hydra import GlobalHydra
    from hydra.utils import instantiate
    from omegaconf import OmegaConf
    import src.megapose.utils.tensor_collection as tc

    return GigaPoseRuntimeDependencies(
        pandas=pd,
        torch=torch,
        tensor_collection_module=tc,
        instantiate=instantiate,
        compose=compose,
        initialize_config_dir=initialize_config_dir,
        global_hydra=GlobalHydra,
        omega_conf=OmegaConf,
    )


class GigaPoseBackend:
    """
    Own one cached CUDA retrieval model for a stable model configuration.

    Construction mutates ``sys.path`` and Hydra global state, then loads weights
    and templates onto CUDA. The stdin worker serializes ``run`` calls. Model
    memory is process-owned and is released when the worker exits.
    """

    def __init__(
        self,
        request: GigaPoseBackendConfig,
        dependencies: GigaPoseRuntimeDependencies | None = None,
    ) -> None:
        self._request = request
        self._dependencies = dependencies
        self._crop_transform = None
        self._normalize = None
        self._model = None

        self._configure_imports()
        validate_gigapose_runtime_files(request)
        self._load_runtime()

    @property
    def dependencies(self) -> GigaPoseRuntimeDependencies:
        """Return injected dependencies, importing them on first access."""
        if self._dependencies is None:
            self._dependencies = load_gigapose_runtime_dependencies()
        return self._dependencies

    def run(self, request: GigaPoseWorkerRequest) -> dict[str, Any]:
        """Run coarse retrieval and return metre-scaled pose hypotheses."""
        started_at = time.monotonic()
        rgb = load_gigapose_rgb(request.image_path)
        mask = load_gigapose_mask(request.mask_path, rgb.shape[:2])
        batch = build_gigapose_batch(
            rgb=rgb,
            mask=mask,
            bbox=request.bbox,
            camera_matrix=request.camera_matrix,
            crop_transform=self._crop_transform,
            normalize=self._normalize,
            torch_module=self.dependencies.torch,
            pandas_module=self.dependencies.pandas,
            tensor_collection_module=self.dependencies.tensor_collection_module,
            object_id=self._request.object_id,
        )
        prediction_path = request.log_dir / "predictions" / f"{request.request_id}.npz"
        prediction_path.unlink(missing_ok=True)

        self.dependencies.torch.cuda.empty_cache()
        with self.dependencies.torch.no_grad():
            self._model.eval_retrieval(
                batch=batch,
                idx_batch=request.request_id,
                dataset_name=self._request.dataset_name,
            )

        coarse_estimates = load_gigapose_prediction(
            prediction_path,
            num_hypotheses=request.num_hypotheses,
            translation_scale=self._request.translation_scale,
        )
        coarse_estimates["elapsed_sec"] = time.monotonic() - started_at
        return coarse_estimates

    def _configure_imports(self) -> None:
        for path in (str(self._request.repo_dir), str(self._request.repo_dir / "src")):
            if path not in sys.path:
                sys.path.insert(0, path)

    def _load_runtime(self) -> None:
        cfg = self._load_config()
        self._crop_transform = self.dependencies.instantiate(cfg.data.transform.crop_transform)
        self._normalize = self.dependencies.instantiate(cfg.data.transform.normalize)

        self._model = self.dependencies.instantiate(cfg.model)
        checkpoint = self.dependencies.torch.load(
            str(self._request.checkpoint_path),
            map_location="cpu",
        )
        state_dict = checkpoint.get("state_dict", checkpoint)
        self._model.load_state_dict(state_dict, strict=False)
        self._model.cuda()
        self._model.eval()

        cfg.data.test.dataloader.dataset_name = self._request.dataset_name
        cfg.data.test.dataloader._target_ = "src.dataloader.template.TemplateSet"
        template_dataset = self.dependencies.instantiate(cfg.data.test.dataloader)
        self._model.template_datasets = {self._request.dataset_name: template_dataset}
        self._model.test_dataset_name = self._request.dataset_name
        self._model.max_num_dets_per_forward = self._request.max_num_dets_per_forward
        self._model.run_id = "ros_runtime"
        self._model.log_interval = 1_000_000

    def _load_config(self):
        if self.dependencies.global_hydra.instance().is_initialized():
            self.dependencies.global_hydra.instance().clear()

        overrides = [
            f"user.local_root_dir={self._request.root_dir}",
            f"test_dataset_name={self._request.dataset_name}",
            "test_setting=detection",
            "machine.batch_size=1",
            "machine.num_workers=0",
            f"max_num_dets_per_forward={self._request.max_num_dets_per_forward}",
            "run_id=ros_runtime",
            f"save_dir={self._request.log_dir}",
            f"model.checkpoint_path={self._request.checkpoint_path}",
            f"data.test.dataloader.template_config.num_templates={self._request.template_count}",
        ]
        if self._request.dinov2_repo_dir is not None:
            overrides.append(
                "model.ae_net.dinov2_model.repo_or_dir="
                f"{self._request.dinov2_repo_dir}"
            )
            overrides.append("+model.ae_net.dinov2_model.source=local")
        with self.dependencies.initialize_config_dir(
            version_base=None,
            config_dir=str(self._request.repo_dir / "configs"),
        ):
            cfg = self.dependencies.compose(config_name="test", overrides=overrides)
        self.dependencies.omega_conf.set_struct(cfg, False)
        return cfg


class GigaPoseWorker:
    """
    Cache CUDA retrieval backends and adapt failures to JSON responses.

    Requests are expected to arrive serially from stdin. Cache keys cover every
    model-affecting field; image, bbox and camera data remain request-local.
    """

    def __init__(
        self,
        *,
        backend_type: type[GigaPoseBackend] = GigaPoseBackend,
        dependencies: GigaPoseRuntimeDependencies | None = None,
    ) -> None:
        self._backend_type = backend_type
        self._dependencies = dependencies
        self._backends: dict[tuple[str, ...], GigaPoseBackend] = {}

    def run_request(self, request: dict[str, Any]) -> dict[str, Any]:
        """Run one request and convert every exception into a typed failure payload."""
        request_id = request.get("id")
        try:
            if request.get("command") == "prewarm":
                config = parse_gigapose_backend_config(request)
                self._get_backend(config)
                return {"id": request_id, "ok": True, "prewarmed": True}
            parsed = parse_gigapose_worker_request(request)
            backend = self._get_backend(parsed)
            result = backend.run(parsed)
            return {
                "id": request_id,
                "ok": True,
                **result,
            }
        except Exception as exc:  # pragma: no cover - runtime protocol guard
            return {
                "id": request_id,
                "ok": False,
                "error": str(exc),
                "traceback": traceback.format_exc(),
            }

    def _get_backend(self, request: GigaPoseBackendConfig) -> GigaPoseBackend:
        cache_key = gigapose_backend_cache_key(request)
        backend = self._backends.get(cache_key)
        if backend is not None:
            return backend

        backend = self._backend_type(request, self._dependencies)
        self._backends[cache_key] = backend
        return backend
