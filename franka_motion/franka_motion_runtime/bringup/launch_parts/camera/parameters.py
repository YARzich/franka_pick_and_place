"""
Parameter-file helpers for table-camera launch nodes.

Responsibility:
- Combine shared table-camera YAML parameters with launch-time overrides.
- Attach use_sim_time consistently to perception and camera-driven motion nodes.

Does not own:
- Declaring launch arguments or validating parameter file contents.
- Backend-specific node factory decisions.
"""

from __future__ import annotations

from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile


def table_camera_parameters(
    configs: dict[str, LaunchConfiguration],
    *overrides: dict[str, object],
) -> list[object]:
    """Build the common parameter overlay for table-camera ROS nodes."""
    parameters: list[object] = [
        ParameterFile(configs["table_camera_params_file"], allow_substs=True),
    ]
    parameters.extend(overrides)
    parameters.append({"use_sim_time": configs["use_sim_time"]})
    return parameters
