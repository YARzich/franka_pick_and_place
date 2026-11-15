"""
Input parsing helpers for the MegaPose JSON worker.

Responsibility:
- Normalize JSON worker requests into a typed request object.
- Resolve runtime example directories.
- Convert optional coarse estimates into the tensor collection expected by MegaPose.

Does not own:
- Loading MegaPose models or running inference.
- Writing output JSON files.
- stdin/stdout worker protocol.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class MegaPoseWorkerRequest:
    """
    Validated example, model and optional coarse hypotheses for one request.

    Coarse 4x4 transforms are object-to-camera poses. Frame IDs and ROS stamps
    remain outside the worker and are restored by the publishing adapter.
    """

    request_id: Any
    debug: bool
    model_name: str
    model_info: Mapping[str, Any]
    example_dir: Path
    object_label: str
    bsz_images: int
    bsz_objects: int
    renderer_workers: int
    coarse_estimates: Mapping[str, Any] | None


def resolve_megapose_example_dir(request: Mapping[str, Any], *, local_data_dir: Path) -> Path:
    """Resolve an explicit example path or MegaPose's named-example default."""
    example_dir = request.get("example_dir")
    if example_dir:
        return Path(str(example_dir)).expanduser()
    return local_data_dir / "examples" / str(request["example_name"])


def parse_megapose_worker_request(
    request: Mapping[str, Any],
    *,
    local_data_dir: Path,
    named_models: Mapping[str, Mapping[str, Any]],
    env: Mapping[str, str] | None = None,
) -> MegaPoseWorkerRequest:
    """Validate model selection and normalize a JSON-compatible request."""
    model_name = str(request["model"])
    if model_name not in named_models:
        raise ValueError(f"unknown MegaPose model: {model_name}")

    debug = bool(request.get("debug", False) or (env or {}).get("MEGAPOSE_WORKER_DEBUG"))
    object_label = str(request.get("object_label") or "")
    coarse_estimates = request.get("coarse_estimates")
    if coarse_estimates is not None and not isinstance(coarse_estimates, Mapping):
        raise ValueError("coarse_estimates должен быть JSON object")

    return MegaPoseWorkerRequest(
        request_id=request.get("id"),
        debug=debug,
        model_name=model_name,
        model_info=named_models[model_name],
        example_dir=resolve_megapose_example_dir(request, local_data_dir=local_data_dir),
        object_label=object_label,
        bsz_images=max(1, int(request.get("bsz_images", 64))),
        bsz_objects=max(1, int(request.get("bsz_objects", 8))),
        renderer_workers=max(1, int(request.get("renderer_workers", 1))),
        coarse_estimates=coarse_estimates,
    )


def build_megapose_coarse_estimates(
    coarse_data: Mapping[str, Any] | None,
    *,
    object_label: str,
    torch_module: Any,
    pandas_module: Any,
    tensor_collection_type: Callable[..., Any],
    add_instance_id_fn: Callable[[Any], Any],
) -> Any | None:
    """
    Convert coarse 4x4 camera poses into MegaPose's CUDA collection.

    ``None`` selects MegaPose's detector path. A present collection must carry
    a label and valid ``N x 4 x 4`` poses; optional scores are retained as
    diagnostic input metadata.
    """
    if not coarse_data:
        return None

    poses = torch_module.as_tensor(coarse_data.get("poses", []), dtype=torch_module.float32)
    if poses.ndim != 3 or poses.shape[1:] != (4, 4):
        raise ValueError("coarse_estimates.poses должен иметь размер [N,4,4]")

    label = str(coarse_data.get("label") or object_label or "")
    if not label:
        raise ValueError("Для coarse_estimates нужен object label")

    num_hypotheses = int(poses.shape[0])
    infos = pandas_module.DataFrame(
        {
            "label": [label] * num_hypotheses,
            "batch_im_id": [0] * num_hypotheses,
            "instance_id": [0] * num_hypotheses,
            "hypothesis_id": list(range(num_hypotheses)),
        }
    )

    scores = coarse_data.get("scores", [])
    if isinstance(scores, list) and len(scores) == num_hypotheses:
        infos["coarse_input_score"] = [float(value) for value in scores]

    coarse_estimates = tensor_collection_type(
        infos=infos,
        poses=poses,
    ).cuda()
    return add_instance_id_fn(coarse_estimates)
