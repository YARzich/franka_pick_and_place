"""
Output helpers for the GigaPose refiner JSON worker.

Responsibility:
- Convert final refiner pose estimates into the existing object_data payload.
- Preserve score forwarding from pose estimates.

Does not own:
- Running refiner/scoring inference.
- Selecting final pose on the ROS-node side.
- stdin/stdout worker protocol.
"""

from __future__ import annotations

from typing import Any


def serialize_gigapose_refiner_prediction(
    pose_estimates: Any,
    *,
    output_label: str,
    object_data_type: Any,
    transform_type: Any,
) -> dict[str, Any]:
    """Serialize the highest-ranked object-to-camera pose and optional score."""
    if len(pose_estimates) < 1:
        raise ValueError("GigaPose refiner не вернул pose")
    pose = pose_estimates.poses[0].detach().cpu().numpy()
    object_data = object_data_type(
        label=output_label,
        TWO=transform_type(pose),
    ).to_json()
    if "pose_score" in pose_estimates.infos:
        object_data["score"] = float(pose_estimates.infos.pose_score.iloc[0])
    return object_data
