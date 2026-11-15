"""
Output serialization helpers for the MegaPose JSON worker.

Responsibility:
- Convert MegaPose pose estimates into the existing object_data JSON contract.
- Persist `outputs/object_data.json` under the runtime example directory.

Does not own:
- Running MegaPose inference.
- Selecting poses on the ROS-node side.
- stdin/stdout worker protocol.
"""

from __future__ import annotations

from collections.abc import Callable
import json
from pathlib import Path
from typing import Any


def serialize_megapose_predictions(
    pose_estimates: Any,
    *,
    object_data_type: Callable[..., Any],
    transform_type: Callable[[Any], Any],
) -> list[dict]:
    """Serialize labelled object-to-camera estimates into object-data records."""
    labels = pose_estimates.infos["label"]
    poses = pose_estimates.poses.cpu().numpy()
    return [
        object_data_type(label=label, TWO=transform_type(pose)).to_json()
        for label, pose in zip(labels, poses)
    ]


def write_megapose_object_data(example_dir: Path, object_data: list[dict]) -> None:
    """Create the example output directory and replace ``object_data.json``."""
    output_dir = example_dir / "outputs"
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "object_data.json").write_text(json.dumps(object_data))


def serialize_and_write_megapose_predictions(
    example_dir: Path,
    pose_estimates: Any,
    *,
    object_data_type: Callable[..., Any],
    transform_type: Callable[[Any], Any],
) -> list[dict]:
    """Serialize estimates, persist the worker artifact and return the same data."""
    object_data = serialize_megapose_predictions(
        pose_estimates,
        object_data_type=object_data_type,
        transform_type=transform_type,
    )
    write_megapose_object_data(example_dir, object_data)
    return object_data
