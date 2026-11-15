"""
Pose-estimation output loading and selection helpers.

Responsibility:
- Select a pose payload from backend output object data.
- Load pose candidates from known output JSON files in priority order.

Does not own:
- Backend execution or subprocess lifecycle.
- ROS Detection3D message construction.
- Runtime example/dataset file creation.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def select_pose_from_object_data(
    object_datas: list[dict[str, Any]],
    object_label: str,
) -> dict[str, Any] | None:
    """Prefer the requested labelled pose, then the first valid fallback pose."""
    for object_data in object_datas:
        if object_data.get("label") == object_label and "TWO" in object_data:
            return object_data
    if object_datas and "TWO" in object_datas[0]:
        return object_datas[0]
    return None


def load_pose_from_output_files(
    output_paths: list[Path],
    object_label: str,
) -> dict[str, Any] | None:
    """Read candidate JSON files in order and return the first selected pose."""
    for output_path in output_paths:
        if not output_path.is_file():
            continue
        object_datas = json.loads(output_path.read_text())
        if not object_datas:
            continue
        pose_data = select_pose_from_object_data(object_datas, object_label)
        if pose_data is not None:
            return pose_data
    return None
