"""
Table-camera mode validation messages and launch conditions.

Responsibility:
- Convert the public mode argument into launch conditions for each pipeline branch.
- Keep accepted mode spellings and user-facing invalid-mode diagnostics consistent.

Does not own:
- Dependency validation for mode-specific packages or files.
- Construction of the nodes gated by these conditions.
"""

from __future__ import annotations

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from franka_motion_runtime.bringup.launch_parts.camera.constants import (
    ESS_MODES,
    MEGAPOSE_MODES,
    POSE_ESTIMATOR_INPUT_MODES,
    SGBM_MODES,
    VALID_MODES_TEXT,
    YOLO_MODES,
)


def _invalid_mode_message(mode_value: str) -> str:
    hint = (
        " Возможно, имелось в виду mode:=foundationPose."
        if mode_value == "foudationpose"
        else ""
    )
    return (
        f"Неизвестный mode: {mode_value!r}. "
        f"Допустимые значения: {VALID_MODES_TEXT}.{hint}"
    )


def _mode_expression(mode: LaunchConfiguration, mode_values: tuple[str, ...]) -> PythonExpression:
    expression = []
    for index, mode_value in enumerate(mode_values):
        if index:
            expression.append(" or ")
        expression.extend(["'", mode, f"'.lower() == '{mode_value}'"])
    return PythonExpression(expression)


def _mode_condition(mode: LaunchConfiguration, mode_values: tuple[str, ...]) -> IfCondition:
    return IfCondition(_mode_expression(mode, mode_values))


def create_pipeline_conditions(configs: dict[str, LaunchConfiguration]) -> dict[str, IfCondition]:
    """Map the public mode selector to compatible launch conditions."""
    mode = configs["mode"]
    return {
        "megapose": _mode_condition(mode, MEGAPOSE_MODES),
        "foundationpose": _mode_condition(mode, ("foundationpose",)),
        "pose_estimator_input": _mode_condition(mode, POSE_ESTIMATOR_INPUT_MODES),
        "ess": _mode_condition(mode, ESS_MODES),
        "sgbm": _mode_condition(mode, SGBM_MODES),
        "yolo": _mode_condition(mode, YOLO_MODES),
    }
