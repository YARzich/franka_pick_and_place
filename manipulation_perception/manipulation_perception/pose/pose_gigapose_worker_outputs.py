"""
Output helpers for the GigaPose coarse JSON worker.

Responsibility:
- Read GigaPose prediction files.
- Preserve the existing coarse-estimates JSON payload and translation scaling.

Does not own:
- Running retrieval inference.
- Refiner scoring or final object_data serialization.
- stdin/stdout worker protocol.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np


def load_gigapose_prediction(
    prediction_path: Path,
    *,
    num_hypotheses: int,
    translation_scale: float,
) -> dict[str, Any]:
    """Load top coarse poses and convert prediction translations to metres."""
    if not prediction_path.is_file():
        raise FileNotFoundError(f"GigaPose prediction file не создан: {prediction_path}")

    prediction = np.load(prediction_path)
    poses = prediction["poses"]
    scores = prediction["scores"]
    if poses.ndim == 3:
        poses = poses[None, ...]
    if scores.ndim == 1:
        scores = scores[None, ...]

    num_selected = min(num_hypotheses, poses.shape[1], scores.shape[1])
    coarse_poses = []
    coarse_scores = []
    for index in range(num_selected):
        pose = poses[0, index].astype(np.float64, copy=True)
        pose[:3, 3] *= translation_scale
        coarse_poses.append(pose.tolist())
        coarse_scores.append(float(scores[0, index]))

    return {
        "poses": coarse_poses,
        "scores": coarse_scores,
    }
