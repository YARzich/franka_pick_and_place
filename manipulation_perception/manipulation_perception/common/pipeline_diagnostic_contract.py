"""
Machine-readable wire contract for perception pipeline diagnostics.

Responsibility:
    Define the stable task prefix, KeyValue names, unavailable marker, and
    source-correlation label shared by diagnostic producers and validators.

Does not own:
    ROS publication, metric collection, threshold policy, or recovery.
"""

from __future__ import annotations

from typing import NamedTuple


PIPELINE_DIAGNOSTIC_TASK_PREFIX = "perception/"
PIPELINE_DIAGNOSTIC_UNAVAILABLE = "n/a"


class PipelineDiagnosticValueKeys(NamedTuple):
    """Ordered names published in each perception DiagnosticStatus."""

    input_count: str
    processed_count: str
    output_count: str
    error_count: str
    drop_or_skip_count: str
    drop_or_skip_ratio: str
    worker_timeout_count: str
    worker_restart_count: str
    latency_sample_count: str
    processing_latency_p50_sec: str
    processing_latency_p95_sec: str
    processing_latency_max_sec: str
    last_success_age_sec: str
    last_success_source: str
    steady_uptime_sec: str


PIPELINE_DIAGNOSTIC_KEYS = PipelineDiagnosticValueKeys(
    input_count="input_count",
    processed_count="processed_count",
    output_count="output_count",
    error_count="error_count",
    drop_or_skip_count="drop_or_skip_count",
    drop_or_skip_ratio="drop_or_skip_ratio",
    worker_timeout_count="worker_timeout_count",
    worker_restart_count="worker_restart_count",
    latency_sample_count="latency_sample_count",
    processing_latency_p50_sec="processing_latency_p50_sec",
    processing_latency_p95_sec="processing_latency_p95_sec",
    processing_latency_max_sec="processing_latency_max_sec",
    last_success_age_sec="last_success_age_sec",
    last_success_source="last_success_source",
    steady_uptime_sec="steady_uptime_sec",
)


def pipeline_diagnostic_task_name(pipeline_name: str) -> str:
    """Return the stable task suffix used by diagnostic_updater."""
    return f"{PIPELINE_DIAGNOSTIC_TASK_PREFIX}{pipeline_name}"


def format_source_correlation(stamp_sec: int, stamp_nanosec: int, frame_id: str) -> str:
    """Format one source header exactly as it appears on the diagnostic wire."""
    return f"{int(stamp_sec)}.{int(stamp_nanosec):09d}@{frame_id}"


__all__ = [
    "PIPELINE_DIAGNOSTIC_KEYS",
    "PIPELINE_DIAGNOSTIC_TASK_PREFIX",
    "PIPELINE_DIAGNOSTIC_UNAVAILABLE",
    "PipelineDiagnosticValueKeys",
    "format_source_correlation",
    "pipeline_diagnostic_task_name",
]
