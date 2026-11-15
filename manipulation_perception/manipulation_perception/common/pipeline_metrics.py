"""
Thread-safe, ROS-independent observability state for perception pipelines.

Responsibility:
    Record bounded counters, stage timings, drop reasons, and source-sample
    correlations and expose immutable snapshots to diagnostics adapters.

Does not own:
    ROS publishers, diagnostic policy, or pipeline execution.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
import math
import threading
import time

from manipulation_perception.common.pipeline_diagnostic_contract import format_source_correlation


@dataclass(frozen=True)
class RosCorrelation:
    """Identify a source sample without treating its ROS stamp as a wall clock."""

    stamp_sec: int
    stamp_nanosec: int
    frame_id: str

    @property
    def label(self) -> str:
        """Return a stable human-readable `(stamp, frame_id)` correlation label."""
        return format_source_correlation(self.stamp_sec, self.stamp_nanosec, self.frame_id)


@dataclass(frozen=True)
class PipelineMetricsSnapshot:
    """Immutable, internally consistent view of counters and latency samples."""

    input_count: int
    processed_count: int
    output_count: int
    error_count: int
    drop_or_skip_count: int
    worker_timeout_count: int
    worker_restart_count: int
    latency_sample_count: int
    processing_latency_p50_sec: float | None
    processing_latency_p95_sec: float | None
    processing_latency_max_sec: float | None
    last_success_age_sec: float | None
    last_success_correlation: RosCorrelation | None
    uptime_sec: float

    @property
    def drop_or_skip_ratio(self) -> float:
        """Return cumulative dropped-or-skipped inputs divided by all inputs."""
        if self.input_count == 0:
            return 0.0
        return min(1.0, self.drop_or_skip_count / self.input_count)


class PipelineMetrics:
    """
    Collect bounded latency samples and cumulative pipeline event counters.

    All durations are computed from the injected monotonic clock. ROS stamps are
    retained only as source-correlation labels, so simulation clock jumps cannot
    create negative or misleading transport/processing latency measurements.
    """

    def __init__(
        self,
        *,
        window_size: int = 120,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        if window_size <= 0:
            raise ValueError("window_size must be positive")
        self._monotonic = monotonic
        self._lock = threading.Lock()
        self._latencies_sec: deque[float] = deque(maxlen=window_size)
        self._reset_locked(self._now())

    def begin_processing(self) -> float:
        """Return a steady-clock token for a later `record_processed` call."""
        return self._now()

    def record_input(self, count: int = 1) -> None:
        """Count source samples offered to this pipeline."""
        self._increment("_input_count", count)

    def record_processed(self, started_at: float, count: int = 1) -> None:
        """Count completed attempts and retain their non-negative steady latency."""
        count = self._validated_count(count)
        latency_sec = max(0.0, self._now() - float(started_at))
        if not math.isfinite(latency_sec):
            raise ValueError("processing latency must be finite")
        with self._lock:
            self._processed_count += count
            for _ in range(count):
                self._latencies_sec.append(latency_sec)

    def record_output(
        self,
        *,
        successful: bool,
        stamp_sec: int | None = None,
        stamp_nanosec: int | None = None,
        frame_id: str = "",
        count: int = 1,
    ) -> None:
        """Count publications and optionally mark the latest successful sample."""
        count = self._validated_count(count)
        correlation = _optional_correlation(stamp_sec, stamp_nanosec, frame_id)
        now = self._now() if successful else None
        with self._lock:
            self._output_count += count
            if successful:
                self._last_success_at = now
                self._last_success_correlation = correlation

    def record_error(self, count: int = 1) -> None:
        """Count failures that prevented normal processing or publication."""
        self._increment("_error_count", count)

    def record_drop_or_skip(self, count: int = 1) -> None:
        """Count intentional decimation, overload drops and stale/sync skips."""
        self._increment("_drop_or_skip_count", count)

    def record_worker_timeout(self, count: int = 1) -> None:
        """Count backend subprocess timeouts separately from generic errors."""
        self._increment("_worker_timeout_count", count)

    def record_worker_restart(self, count: int = 1) -> None:
        """Count replacement starts of persistent backend subprocesses."""
        self._increment("_worker_restart_count", count)

    def snapshot(self) -> PipelineMetricsSnapshot:
        """Return one lock-consistent snapshot using steady time for ages."""
        now = self._now()
        with self._lock:
            latencies = sorted(self._latencies_sec)
            last_success_age_sec = None
            if self._last_success_at is not None:
                last_success_age_sec = max(0.0, now - self._last_success_at)
            return PipelineMetricsSnapshot(
                input_count=self._input_count,
                processed_count=self._processed_count,
                output_count=self._output_count,
                error_count=self._error_count,
                drop_or_skip_count=self._drop_or_skip_count,
                worker_timeout_count=self._worker_timeout_count,
                worker_restart_count=self._worker_restart_count,
                latency_sample_count=len(latencies),
                processing_latency_p50_sec=_percentile(latencies, 0.50),
                processing_latency_p95_sec=_percentile(latencies, 0.95),
                processing_latency_max_sec=latencies[-1] if latencies else None,
                last_success_age_sec=last_success_age_sec,
                last_success_correlation=self._last_success_correlation,
                uptime_sec=max(0.0, now - self._reset_at),
            )

    def reset(self) -> None:
        """Atomically clear the window, counters and last-success state."""
        now = self._now()
        with self._lock:
            self._reset_locked(now)

    def _increment(self, attribute: str, count: int) -> None:
        count = self._validated_count(count)
        with self._lock:
            setattr(self, attribute, getattr(self, attribute) + count)

    def _reset_locked(self, now: float) -> None:
        self._latencies_sec.clear()
        self._input_count = 0
        self._processed_count = 0
        self._output_count = 0
        self._error_count = 0
        self._drop_or_skip_count = 0
        self._worker_timeout_count = 0
        self._worker_restart_count = 0
        self._last_success_at: float | None = None
        self._last_success_correlation: RosCorrelation | None = None
        self._reset_at = now

    def _now(self) -> float:
        now = float(self._monotonic())
        if not math.isfinite(now):
            raise ValueError("monotonic clock must return a finite value")
        return now

    @staticmethod
    def _validated_count(count: int) -> int:
        count = int(count)
        if count <= 0:
            raise ValueError("event count must be positive")
        return count


def _percentile(sorted_values: list[float], quantile: float) -> float | None:
    if not sorted_values:
        return None
    rank = (len(sorted_values) - 1) * quantile
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return sorted_values[lower]
    weight = rank - lower
    return sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight


def _optional_correlation(
    stamp_sec: int | None,
    stamp_nanosec: int | None,
    frame_id: str,
) -> RosCorrelation | None:
    if stamp_sec is None or stamp_nanosec is None:
        return None
    nanosec = int(stamp_nanosec)
    if not 0 <= nanosec < 1_000_000_000:
        raise ValueError("stamp_nanosec must be in [0, 1e9)")
    return RosCorrelation(int(stamp_sec), nanosec, str(frame_id))


__all__ = ["PipelineMetrics", "PipelineMetricsSnapshot", "RosCorrelation"]
