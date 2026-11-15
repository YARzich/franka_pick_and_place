"""
Read-only `diagnostic_updater` adapter for perception pipeline metrics.

Responsibility:
    Snapshot backend-neutral pipeline metrics and expose them as ROS diagnostic
    tasks without changing pipeline state or initiating recovery.

Does not own:
    Metric collection, health recovery, or sensor lifecycle.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater

from manipulation_perception.common.pipeline_diagnostic_contract import (
    PIPELINE_DIAGNOSTIC_KEYS,
    pipeline_diagnostic_task_name,
    PIPELINE_DIAGNOSTIC_UNAVAILABLE,
)
from manipulation_perception.common.pipeline_diagnostics_config import (
    declare_pipeline_diagnostics_parameters,
    PipelineDiagnosticsConfig,
    read_pipeline_diagnostics_config,
)
from manipulation_perception.common.pipeline_metrics import (
    PipelineMetrics,
    PipelineMetricsSnapshot,
)


@dataclass(frozen=True)
class PipelineHealth:
    """Severity and summary derived from one immutable metrics snapshot."""

    level: int
    message: str


class PipelineDiagnostics:
    """
    Publish one standard diagnostic task without mutating pipeline state.

    The adapter owns only the updater timer/task registration. It never resets,
    restarts or recovers a backend; recovery policy remains with the owning node.
    """

    def __init__(
        self,
        *,
        node: Any,
        pipeline_name: str,
        metrics: PipelineMetrics,
        config: PipelineDiagnosticsConfig,
        updater_factory: Callable[..., Any] = Updater,
    ) -> None:
        self._metrics = metrics
        self._config = config
        self._updater: Any | None = None
        if not config.enabled:
            return
        self._updater = updater_factory(node, period=config.update_period_sec)
        self._updater.setHardwareID("none")
        self._updater.add(pipeline_diagnostic_task_name(pipeline_name), self._produce_status)

    def close(self) -> None:
        """Cancel the diagnostic timer before the owning node destroys resources."""
        if self._updater is None:
            return
        timer = getattr(self._updater, "timer", None)
        if timer is not None:
            timer.cancel()
        self._updater = None

    def _produce_status(self, status: Any) -> Any:
        snapshot = self._metrics.snapshot()
        health = evaluate_pipeline_health(snapshot, self._config)
        status.summary(health.level, health.message)
        _add_snapshot_values(status, snapshot)
        return status


class PipelineObserver:
    """Bind pure metrics and standard diagnostics to one ROS pipeline adapter."""

    def __init__(
        self,
        *,
        metrics: PipelineMetrics,
        diagnostics: PipelineDiagnostics,
    ) -> None:
        self.metrics = metrics
        self._diagnostics = diagnostics

    @classmethod
    def create(cls, node: Any, pipeline_name: str) -> "PipelineObserver":
        """Declare/read shared parameters and register one 1 Hz diagnostic task."""
        declare_pipeline_diagnostics_parameters(node)
        config = read_pipeline_diagnostics_config(node)
        metrics = PipelineMetrics(window_size=config.window_size)
        return cls(
            metrics=metrics,
            diagnostics=PipelineDiagnostics(
                node=node,
                pipeline_name=pipeline_name,
                metrics=metrics,
                config=config,
            ),
        )

    def record_output_header(self, header: Any, successful: bool = True) -> None:
        """Record output using the source header only as a correlation label."""
        stamp = getattr(header, "stamp", None)
        self.metrics.record_output(
            successful=successful,
            stamp_sec=getattr(stamp, "sec", None),
            stamp_nanosec=getattr(stamp, "nanosec", None),
            frame_id=str(getattr(header, "frame_id", "")),
        )

    def close(self) -> None:
        """Stop diagnostic publication while leaving the snapshot readable."""
        self._diagnostics.close()


def evaluate_pipeline_health(
    snapshot: PipelineMetricsSnapshot,
    config: PipelineDiagnosticsConfig,
) -> PipelineHealth:
    """Evaluate only explicitly enabled thresholds, with ERROR taking precedence."""
    error_reasons = _threshold_reasons(snapshot, config, error=True)
    if error_reasons:
        return PipelineHealth(DiagnosticStatus.ERROR, "; ".join(error_reasons))
    warn_reasons = _threshold_reasons(snapshot, config, error=False)
    if warn_reasons:
        return PipelineHealth(DiagnosticStatus.WARN, "; ".join(warn_reasons))
    if config.require_success and snapshot.last_success_age_sec is None:
        return PipelineHealth(
            DiagnosticStatus.WARN,
            "waiting for first successful output",
        )
    if snapshot.input_count == 0:
        return PipelineHealth(DiagnosticStatus.OK, "waiting for input")
    return PipelineHealth(DiagnosticStatus.OK, "pipeline observed")


def _threshold_reasons(
    snapshot: PipelineMetricsSnapshot,
    config: PipelineDiagnosticsConfig,
    *,
    error: bool,
) -> list[str]:
    prefix = "error" if error else "warn"
    latency_limit = getattr(config, f"{prefix}_processing_latency_p95_sec")
    age_limit = getattr(config, f"{prefix}_last_success_age_sec")
    drop_limit = getattr(config, f"{prefix}_drop_or_skip_ratio")
    error_count_limit = getattr(config, f"{prefix}_error_count")
    reasons: list[str] = []
    if (
        latency_limit > 0.0
        and snapshot.latency_sample_count >= config.minimum_samples
        and snapshot.processing_latency_p95_sec is not None
        and snapshot.processing_latency_p95_sec >= latency_limit
    ):
        reasons.append(f"processing p95 {snapshot.processing_latency_p95_sec:.3f}s")
    if age_limit > 0.0:
        if snapshot.last_success_age_sec is not None:
            if snapshot.last_success_age_sec >= age_limit:
                reasons.append(f"last success age {snapshot.last_success_age_sec:.3f}s")
        elif snapshot.uptime_sec >= age_limit:
            reasons.append("no successful output")
    if (
        drop_limit > 0.0
        and snapshot.input_count >= config.minimum_samples
        and snapshot.drop_or_skip_ratio >= drop_limit
    ):
        reasons.append(f"drop/skip ratio {snapshot.drop_or_skip_ratio:.3f}")
    if error_count_limit > 0 and snapshot.error_count >= error_count_limit:
        reasons.append(f"errors {snapshot.error_count}")
    return reasons


def _add_snapshot_values(status: Any, snapshot: PipelineMetricsSnapshot) -> None:
    # diagnostic_msgs/KeyValue requires strings. Some test doubles accept
    # numbers, but the generated Python conversion asserts at runtime.
    keys = PIPELINE_DIAGNOSTIC_KEYS
    status.add(keys.input_count, str(snapshot.input_count))
    status.add(keys.processed_count, str(snapshot.processed_count))
    status.add(keys.output_count, str(snapshot.output_count))
    status.add(keys.error_count, str(snapshot.error_count))
    status.add(keys.drop_or_skip_count, str(snapshot.drop_or_skip_count))
    status.add(keys.drop_or_skip_ratio, f"{snapshot.drop_or_skip_ratio:.6f}")
    status.add(keys.worker_timeout_count, str(snapshot.worker_timeout_count))
    status.add(keys.worker_restart_count, str(snapshot.worker_restart_count))
    status.add(keys.latency_sample_count, str(snapshot.latency_sample_count))
    status.add(keys.processing_latency_p50_sec, _format_optional(snapshot.processing_latency_p50_sec))
    status.add(keys.processing_latency_p95_sec, _format_optional(snapshot.processing_latency_p95_sec))
    status.add(keys.processing_latency_max_sec, _format_optional(snapshot.processing_latency_max_sec))
    status.add(keys.last_success_age_sec, _format_optional(snapshot.last_success_age_sec))
    correlation = snapshot.last_success_correlation
    status.add(
        keys.last_success_source,
        correlation.label if correlation is not None else PIPELINE_DIAGNOSTIC_UNAVAILABLE,
    )
    status.add(keys.steady_uptime_sec, f"{snapshot.uptime_sec:.3f}")


def _format_optional(value: float | None) -> str:
    return PIPELINE_DIAGNOSTIC_UNAVAILABLE if value is None else f"{value:.6f}"


__all__ = [
    "PipelineDiagnostics",
    "PipelineHealth",
    "PipelineObserver",
    "evaluate_pipeline_health",
]
