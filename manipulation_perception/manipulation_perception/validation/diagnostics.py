"""
Pure parser and delta model for perception DiagnosticStatus values.

Responsibility:
    Convert the shared 15-key wire contract into typed snapshots, reject
    duplicate/missing/malformed values, and calculate cumulative counter deltas.

Does not own:
    ROS subscriptions, health thresholds, operator messages, or recovery.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
import math
import re

from manipulation_perception.common.config_validation import is_valid_frame_id
from manipulation_perception.common.pipeline_diagnostic_contract import (
    format_source_correlation,
    PIPELINE_DIAGNOSTIC_KEYS,
    PIPELINE_DIAGNOSTIC_UNAVAILABLE,
)


_COUNTER_PATTERN = re.compile(r"^(0|[1-9][0-9]*)$")
_CORRELATION_PATTERN = re.compile(r"^(-?[0-9]+)\.([0-9]{9})@(.+)$")


class DiagnosticContractError(ValueError):
    """Report malformed machine-readable diagnostic values."""


@dataclass(frozen=True)
class SourceCorrelation:
    """Positive source message timestamp and canonical relative frame."""

    stamp_sec: int
    stamp_nanosec: int
    frame_id: str

    @property
    def label(self) -> str:
        """Return the exact diagnostic wire representation."""
        return format_source_correlation(
            self.stamp_sec, self.stamp_nanosec, self.frame_id
        )


@dataclass(frozen=True)
class PipelineDiagnosticSnapshot:
    """Typed values from one complete perception DiagnosticStatus."""

    input_count: int
    processed_count: int
    output_count: int
    error_count: int
    drop_or_skip_count: int
    drop_or_skip_ratio: float
    worker_timeout_count: int
    worker_restart_count: int
    latency_sample_count: int
    processing_latency_p50_sec: float | None
    processing_latency_p95_sec: float | None
    processing_latency_max_sec: float | None
    last_success_age_sec: float | None
    last_success_source: SourceCorrelation | None
    steady_uptime_sec: float

    def as_dict(self) -> dict[str, object]:
        """Return artifact-ready data without ROS message objects."""
        return {
            "input_count": self.input_count,
            "processed_count": self.processed_count,
            "output_count": self.output_count,
            "error_count": self.error_count,
            "drop_or_skip_count": self.drop_or_skip_count,
            "drop_or_skip_ratio": self.drop_or_skip_ratio,
            "worker_timeout_count": self.worker_timeout_count,
            "worker_restart_count": self.worker_restart_count,
            "latency_sample_count": self.latency_sample_count,
            "processing_latency_p50_sec": self.processing_latency_p50_sec,
            "processing_latency_p95_sec": self.processing_latency_p95_sec,
            "processing_latency_max_sec": self.processing_latency_max_sec,
            "last_success_age_sec": self.last_success_age_sec,
            "last_success_source": (
                self.last_success_source.label
                if self.last_success_source is not None
                else None
            ),
            "steady_uptime_sec": self.steady_uptime_sec,
        }


@dataclass(frozen=True)
class PipelineDiagnosticDelta:
    """Non-negative cumulative changes over one observation window."""

    input_count: int
    processed_count: int
    output_count: int
    error_count: int
    drop_or_skip_count: int
    worker_timeout_count: int
    worker_restart_count: int
    latency_observation_count: int

    def as_dict(self) -> dict[str, int]:
        """Return the stable counter-delta artifact shape."""
        return {
            "input_count": self.input_count,
            "processed_count": self.processed_count,
            "output_count": self.output_count,
            "error_count": self.error_count,
            "drop_or_skip_count": self.drop_or_skip_count,
            "worker_timeout_count": self.worker_timeout_count,
            "worker_restart_count": self.worker_restart_count,
            "latency_observation_count": self.latency_observation_count,
        }


def key_values_to_mapping(values: Iterable[tuple[str, str]]) -> dict[str, str]:
    """Convert unordered KeyValue pairs while rejecting duplicate wire keys."""
    result: dict[str, str] = {}
    for key, value in values:
        key_text = str(key)
        if key_text in result:
            raise DiagnosticContractError(
                f"duplicate DiagnosticStatus KeyValue {key_text!r}"
            )
        result[key_text] = str(value)
    return result


def require_unique_status_names(status_names: Iterable[str]) -> tuple[str, ...]:
    """Reject duplicate required statuses within one DiagnosticArray sample."""
    result = tuple(str(name) for name in status_names)
    duplicate = sorted(
        name for name in set(result) if result.count(name) > 1
    )
    if duplicate:
        raise DiagnosticContractError(
            "duplicate DiagnosticStatus names in one array: " + ", ".join(duplicate)
        )
    return result


def parse_pipeline_snapshot(values: Mapping[str, str]) -> PipelineDiagnosticSnapshot:
    """Parse one snapshot using shared producer constants, never guessed labels."""
    keys = PIPELINE_DIAGNOSTIC_KEYS
    missing = sorted(set(keys) - set(values))
    if missing:
        raise DiagnosticContractError(
            "missing perception diagnostic keys: " + ", ".join(missing)
        )
    snapshot = PipelineDiagnosticSnapshot(
        input_count=_counter(values[keys.input_count], keys.input_count),
        processed_count=_counter(values[keys.processed_count], keys.processed_count),
        output_count=_counter(values[keys.output_count], keys.output_count),
        error_count=_counter(values[keys.error_count], keys.error_count),
        drop_or_skip_count=_counter(
            values[keys.drop_or_skip_count], keys.drop_or_skip_count
        ),
        drop_or_skip_ratio=_non_negative_float(
            values[keys.drop_or_skip_ratio], keys.drop_or_skip_ratio
        ),
        worker_timeout_count=_counter(
            values[keys.worker_timeout_count], keys.worker_timeout_count
        ),
        worker_restart_count=_counter(
            values[keys.worker_restart_count], keys.worker_restart_count
        ),
        latency_sample_count=_counter(
            values[keys.latency_sample_count], keys.latency_sample_count
        ),
        processing_latency_p50_sec=_optional_non_negative_float(
            values[keys.processing_latency_p50_sec],
            keys.processing_latency_p50_sec,
        ),
        processing_latency_p95_sec=_optional_non_negative_float(
            values[keys.processing_latency_p95_sec],
            keys.processing_latency_p95_sec,
        ),
        processing_latency_max_sec=_optional_non_negative_float(
            values[keys.processing_latency_max_sec],
            keys.processing_latency_max_sec,
        ),
        last_success_age_sec=_optional_non_negative_float(
            values[keys.last_success_age_sec], keys.last_success_age_sec
        ),
        last_success_source=_optional_correlation(
            values[keys.last_success_source]
        ),
        steady_uptime_sec=_non_negative_float(
            values[keys.steady_uptime_sec], keys.steady_uptime_sec
        ),
    )
    if snapshot.drop_or_skip_ratio > 1.0:
        raise DiagnosticContractError("drop_or_skip_ratio must be in [0, 1]")
    _validate_latency_order(snapshot)
    return snapshot


def calculate_pipeline_delta(
    baseline: PipelineDiagnosticSnapshot,
    latest: PipelineDiagnosticSnapshot,
) -> PipelineDiagnosticDelta:
    """Calculate deltas and fail when a cumulative counter moved backwards."""
    names = (
        "input_count",
        "processed_count",
        "output_count",
        "error_count",
        "drop_or_skip_count",
        "worker_timeout_count",
        "worker_restart_count",
    )
    deltas = {}
    for name in names:
        value = int(getattr(latest, name)) - int(getattr(baseline, name))
        if value < 0:
            raise DiagnosticContractError(
                f"cumulative diagnostic counter {name} moved backwards"
            )
        deltas[name] = value
    return PipelineDiagnosticDelta(
        **deltas,
        # record_processed increments processed_count and appends exactly the
        # same count of latency observations; latency_sample_count itself is a
        # bounded window size and cannot provide an honest cumulative delta.
        latency_observation_count=deltas["processed_count"],
    )


def _counter(value: str, name: str) -> int:
    if _COUNTER_PATTERN.fullmatch(str(value)) is None:
        raise DiagnosticContractError(f"{name} must be a canonical non-negative integer")
    return int(value)


def _non_negative_float(value: str, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise DiagnosticContractError(f"{name} must be numeric") from error
    if not math.isfinite(number) or number < 0.0:
        raise DiagnosticContractError(f"{name} must be finite and non-negative")
    return number


def _optional_non_negative_float(value: str, name: str) -> float | None:
    if value == PIPELINE_DIAGNOSTIC_UNAVAILABLE:
        return None
    return _non_negative_float(value, name)


def _optional_correlation(value: str) -> SourceCorrelation | None:
    if value == PIPELINE_DIAGNOSTIC_UNAVAILABLE:
        return None
    match = _CORRELATION_PATTERN.fullmatch(value)
    if match is None:
        raise DiagnosticContractError("last_success_source has an invalid format")
    stamp_sec = int(match.group(1))
    stamp_nanosec = int(match.group(2))
    frame_id = match.group(3)
    if stamp_sec * 1_000_000_000 + stamp_nanosec <= 0:
        raise DiagnosticContractError("last_success_source stamp must be positive")
    if not is_valid_frame_id(frame_id):
        raise DiagnosticContractError(
            "last_success_source frame must be canonical and relative"
        )
    return SourceCorrelation(stamp_sec, stamp_nanosec, frame_id)


def _validate_latency_order(snapshot: PipelineDiagnosticSnapshot) -> None:
    latencies = (
        snapshot.processing_latency_p50_sec,
        snapshot.processing_latency_p95_sec,
        snapshot.processing_latency_max_sec,
    )
    available = tuple(value is not None for value in latencies)
    if any(available) and not all(available):
        raise DiagnosticContractError(
            "p50, p95, and max latency must be available together"
        )
    if all(available):
        p50, p95, maximum = latencies
        assert p50 is not None and p95 is not None and maximum is not None
        if not p50 <= p95 <= maximum:
            raise DiagnosticContractError("latency values must satisfy p50 <= p95 <= max")


__all__ = [
    "DiagnosticContractError",
    "PipelineDiagnosticDelta",
    "PipelineDiagnosticSnapshot",
    "SourceCorrelation",
    "calculate_pipeline_delta",
    "key_values_to_mapping",
    "parse_pipeline_snapshot",
    "require_unique_status_names",
]
