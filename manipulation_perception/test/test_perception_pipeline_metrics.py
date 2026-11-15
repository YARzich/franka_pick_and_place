"""
Test the pure perception-pipeline metrics state.

Responsibility: Verify latency statistics, success freshness, reset, and concurrent counters.
"""

from concurrent.futures import ThreadPoolExecutor
import threading

import pytest

from manipulation_perception.common.pipeline_metrics import PipelineMetrics


class _FakeMonotonic:
    def __init__(self) -> None:
        self.value = 100.0

    def __call__(self) -> float:
        return self.value

    def advance(self, seconds: float) -> None:
        self.value += seconds


def test_latency_window_percentiles_and_source_correlation() -> None:
    clock = _FakeMonotonic()
    metrics = PipelineMetrics(window_size=3, monotonic=clock)

    for latency_sec in (1.0, 2.0, 3.0, 4.0):
        started_at = metrics.begin_processing()
        clock.advance(latency_sec)
        metrics.record_processed(started_at)
    metrics.record_input(4)
    metrics.record_output(
        successful=True,
        stamp_sec=7,
        stamp_nanosec=11,
        frame_id="camera_optical_frame",
    )
    clock.advance(0.5)

    snapshot = metrics.snapshot()
    assert snapshot.latency_sample_count == 3
    assert snapshot.processing_latency_p50_sec == pytest.approx(3.0)
    assert snapshot.processing_latency_p95_sec == pytest.approx(3.9)
    assert snapshot.processing_latency_max_sec == pytest.approx(4.0)
    assert snapshot.last_success_age_sec == pytest.approx(0.5)
    assert snapshot.last_success_correlation is not None
    assert snapshot.last_success_correlation.label == "7.000000011@camera_optical_frame"


def test_unsuccessful_output_does_not_replace_last_success() -> None:
    clock = _FakeMonotonic()
    metrics = PipelineMetrics(monotonic=clock)
    metrics.record_output(successful=True, stamp_sec=1, stamp_nanosec=2, frame_id="camera")
    clock.advance(1.0)
    metrics.record_output(successful=False, stamp_sec=3, stamp_nanosec=4, frame_id="camera")

    snapshot = metrics.snapshot()
    assert snapshot.output_count == 2
    assert snapshot.last_success_age_sec == pytest.approx(1.0)
    assert snapshot.last_success_correlation is not None
    assert snapshot.last_success_correlation.stamp_sec == 1


def test_reset_atomically_clears_window_counters_and_success() -> None:
    clock = _FakeMonotonic()
    metrics = PipelineMetrics(monotonic=clock)
    started_at = metrics.begin_processing()
    clock.advance(0.25)
    metrics.record_input()
    metrics.record_processed(started_at)
    metrics.record_output(successful=True)
    metrics.record_error()
    metrics.record_drop_or_skip()
    metrics.record_worker_timeout()
    metrics.record_worker_restart()

    metrics.reset()
    snapshot = metrics.snapshot()

    assert snapshot.input_count == 0
    assert snapshot.processed_count == 0
    assert snapshot.output_count == 0
    assert snapshot.error_count == 0
    assert snapshot.drop_or_skip_count == 0
    assert snapshot.worker_timeout_count == 0
    assert snapshot.worker_restart_count == 0
    assert snapshot.processing_latency_p50_sec is None
    assert snapshot.last_success_age_sec is None
    assert snapshot.uptime_sec == pytest.approx(0.0)


def test_concurrent_counter_updates_are_not_lost() -> None:
    metrics = PipelineMetrics(window_size=32)
    start_gate = threading.Barrier(8)

    def update_metrics() -> None:
        start_gate.wait()
        for _ in range(500):
            started_at = metrics.begin_processing()
            metrics.record_input()
            metrics.record_processed(started_at)
            metrics.record_output(successful=True)
            metrics.record_error()
            metrics.record_drop_or_skip()

    with ThreadPoolExecutor(max_workers=8) as executor:
        futures = [executor.submit(update_metrics) for _ in range(8)]
        for future in futures:
            future.result()

    snapshot = metrics.snapshot()
    assert snapshot.input_count == 4000
    assert snapshot.processed_count == 4000
    assert snapshot.output_count == 4000
    assert snapshot.error_count == 4000
    assert snapshot.drop_or_skip_count == 4000
    assert snapshot.latency_sample_count == 32


@pytest.mark.parametrize("count", [0, -1])
def test_non_positive_event_counts_are_rejected(count: int) -> None:
    metrics = PipelineMetrics()
    with pytest.raises(ValueError, match="positive"):
        metrics.record_input(count)
