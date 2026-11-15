"""
Typed ROS parameter contract for perception pipeline diagnostics.

Responsibility:
    Declare, normalize, and validate diagnostic retention, publication, and
    optional threshold parameters for perception adapters.

Does not own:
    Metric storage or diagnostic publication.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from manipulation_perception.common.parameter_utils import bool_value


@dataclass(frozen=True)
class PipelineDiagnosticsConfig:
    """
    Configure metrics retention, publication and optional health thresholds.

    A non-positive threshold disables that check. This keeps diagnostics
    observational by default until deployment-specific budgets are measured.
    """

    enabled: bool
    require_success: bool
    window_size: int
    update_period_sec: float
    minimum_samples: int
    warn_processing_latency_p95_sec: float
    error_processing_latency_p95_sec: float
    warn_last_success_age_sec: float
    error_last_success_age_sec: float
    warn_drop_or_skip_ratio: float
    error_drop_or_skip_ratio: float
    warn_error_count: int
    error_error_count: int


def declare_pipeline_diagnostics_parameters(node: Any) -> None:
    """Declare the shared `diagnostics.*` parameter API on a ROS node-like object."""
    node.declare_parameter("diagnostics.enabled", True)
    node.declare_parameter("diagnostics.require_success", False)
    node.declare_parameter("diagnostics.window_size", 120)
    node.declare_parameter("diagnostics.update_period_sec", 1.0)
    node.declare_parameter("diagnostics.minimum_samples", 10)
    node.declare_parameter("diagnostics.warn_processing_latency_p95_sec", 0.0)
    node.declare_parameter("diagnostics.error_processing_latency_p95_sec", 0.0)
    node.declare_parameter("diagnostics.warn_last_success_age_sec", 0.0)
    node.declare_parameter("diagnostics.error_last_success_age_sec", 0.0)
    node.declare_parameter("diagnostics.warn_drop_or_skip_ratio", 0.0)
    node.declare_parameter("diagnostics.error_drop_or_skip_ratio", 0.0)
    node.declare_parameter("diagnostics.warn_error_count", 0)
    node.declare_parameter("diagnostics.error_error_count", 0)


def read_pipeline_diagnostics_config(node: Any) -> PipelineDiagnosticsConfig:
    """Read and normalize the typed diagnostics contract from a ROS node-like object."""

    def value(name: str) -> Any:
        return node.get_parameter(name).value

    warn_latency = _threshold_float(value("diagnostics.warn_processing_latency_p95_sec"))
    error_latency = _ordered_error_threshold(
        warn_latency,
        value("diagnostics.error_processing_latency_p95_sec"),
    )
    warn_age = _threshold_float(value("diagnostics.warn_last_success_age_sec"))
    error_age = _ordered_error_threshold(
        warn_age,
        value("diagnostics.error_last_success_age_sec"),
    )
    warn_drop = _ratio_threshold(value("diagnostics.warn_drop_or_skip_ratio"))
    error_drop = _ordered_error_threshold(
        warn_drop,
        _ratio_threshold(value("diagnostics.error_drop_or_skip_ratio")),
    )
    warn_errors = _threshold_int(value("diagnostics.warn_error_count"))
    error_errors = _ordered_error_count(
        warn_errors,
        value("diagnostics.error_error_count"),
    )
    return PipelineDiagnosticsConfig(
        enabled=bool_value(value("diagnostics.enabled"), "diagnostics.enabled"),
        require_success=bool_value(
            value("diagnostics.require_success"),
            "diagnostics.require_success",
        ),
        window_size=max(1, int(value("diagnostics.window_size"))),
        update_period_sec=max(0.1, float(value("diagnostics.update_period_sec"))),
        minimum_samples=max(1, int(value("diagnostics.minimum_samples"))),
        warn_processing_latency_p95_sec=warn_latency,
        error_processing_latency_p95_sec=error_latency,
        warn_last_success_age_sec=warn_age,
        error_last_success_age_sec=error_age,
        warn_drop_or_skip_ratio=warn_drop,
        error_drop_or_skip_ratio=error_drop,
        warn_error_count=warn_errors,
        error_error_count=error_errors,
    )


def _threshold_float(value: Any) -> float:
    return max(0.0, float(value))


def _ratio_threshold(value: Any) -> float:
    return min(1.0, _threshold_float(value))


def _threshold_int(value: Any) -> int:
    return max(0, int(value))


def _ordered_error_threshold(warn_threshold: float, value: Any) -> float:
    threshold = _threshold_float(value)
    if threshold > 0.0 and warn_threshold > 0.0 and threshold < warn_threshold:
        raise ValueError("diagnostics ERROR threshold must not be lower than WARN")
    return threshold


def _ordered_error_count(warn_threshold: int, value: Any) -> int:
    threshold = _threshold_int(value)
    if threshold > 0 and warn_threshold > 0 and threshold < warn_threshold:
        raise ValueError("diagnostics ERROR count must not be lower than WARN")
    return threshold


__all__ = [
    "PipelineDiagnosticsConfig",
    "declare_pipeline_diagnostics_parameters",
    "read_pipeline_diagnostics_config",
]
