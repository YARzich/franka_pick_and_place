"""
Bounded diagnostic-window evaluation for GPU perception runtime evidence.

Responsibility:
    Retain baseline/latest exact-name statuses, calculate cumulative deltas and
    throughput, and enforce freshness/error/worker/latency evidence policy.

Does not own:
    Topic correlation, ROS subscriptions, producer thresholds, or recovery.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping

from manipulation_perception.validation.contracts import GpuPerceptionContract
from manipulation_perception.validation.diagnostics import (
    calculate_pipeline_delta,
    DiagnosticContractError,
    parse_pipeline_snapshot,
    PipelineDiagnosticDelta,
)
from manipulation_perception.validation.error_log import BoundedErrorLog
from manipulation_perception.validation.evidence_model import (
    DiagnosticEvidence,
    DiagnosticObservation,
)


_DIAGNOSTIC_ERROR_LEVEL = 2


@dataclass
class _DiagnosticHistory:
    baseline: DiagnosticObservation
    latest: DiagnosticObservation
    update_count: int
    maximum_level: int


@dataclass(frozen=True)
class DiagnosticEvidenceEvaluation:
    """Current per-role diagnostic evidence and diagnostic-specific errors."""

    diagnostics: tuple[DiagnosticEvidence, ...]
    errors: tuple[str, ...]


class DiagnosticEvidenceStore:
    """Retain two endpoints and severity history for required exact statuses."""

    def __init__(self, contract: GpuPerceptionContract) -> None:
        self._contract = contract
        self._contracts = {item.status_name: item for item in contract.diagnostics}
        self._histories: dict[str, _DiagnosticHistory] = {}
        self._errors = BoundedErrorLog()

    def record(
        self,
        *,
        status_name: str,
        level: int,
        message: str,
        values: Mapping[str, str],
        received_elapsed_sec: float,
    ) -> None:
        """Parse one required status and update its baseline/latest window."""
        contract = self._contracts.get(status_name)
        if contract is None:
            raise KeyError(f"unconfigured diagnostic status: {status_name}")
        numeric_level = int(level)
        if not 0 <= numeric_level <= 3:
            self._errors.add(
                f"diagnostic {contract.role} has invalid level {numeric_level}"
            )
            return
        try:
            snapshot = parse_pipeline_snapshot(values)
        except DiagnosticContractError as error:
            self._errors.add(f"diagnostic {contract.role}: {error}")
            return
        observation = DiagnosticObservation(
            level=numeric_level,
            message=str(message),
            received_elapsed_sec=_non_negative_finite(
                received_elapsed_sec, "received_elapsed_sec"
            ),
            snapshot=snapshot,
        )
        history = self._histories.get(contract.role)
        if history is None:
            self._histories[contract.role] = _DiagnosticHistory(
                baseline=observation,
                latest=observation,
                update_count=1,
                maximum_level=numeric_level,
            )
            return
        history.latest = observation
        history.update_count += 1
        history.maximum_level = max(history.maximum_level, numeric_level)

    def evaluate(self, *, elapsed_sec: float) -> DiagnosticEvidenceEvaluation:
        """Build complete role evidence and apply the dedicated-profile gates."""
        elapsed = _non_negative_finite(elapsed_sec, "elapsed_sec")
        errors = list(self._errors.as_tuple())
        result = []
        for contract in self._contract.diagnostics:
            history = self._histories.get(contract.role)
            if history is None:
                errors.append(f"diagnostic role {contract.role} was not observed")
                continue
            result.append(self._evaluate_role(contract, history, elapsed, errors))
        return DiagnosticEvidenceEvaluation(
            diagnostics=tuple(result),
            errors=tuple(dict.fromkeys(errors)),
        )

    def _evaluate_role(
        self,
        contract,
        history: _DiagnosticHistory,
        elapsed: float,
        errors: list[str],
    ) -> DiagnosticEvidence:
        if history.update_count < 2:
            errors.append(
                f"diagnostic role {contract.role} needs baseline and latest updates"
            )
        window_sec = max(
            0.0,
            history.latest.received_elapsed_sec
            - history.baseline.received_elapsed_sec,
        )
        latest_age = max(0.0, elapsed - history.latest.received_elapsed_sec)
        self._evaluate_health(contract.role, history, latest_age, errors)
        try:
            delta = calculate_pipeline_delta(
                history.baseline.snapshot, history.latest.snapshot
            )
        except DiagnosticContractError as error:
            errors.append(f"diagnostic role {contract.role}: {error}")
            delta = PipelineDiagnosticDelta(0, 0, 0, 0, 0, 0, 0, 0)
        self._evaluate_metrics(contract.role, history, delta, errors)
        rate_window = window_sec if window_sec > 0.0 else 1.0
        return DiagnosticEvidence(
            role=contract.role,
            status_name=contract.status_name,
            source_topic_role=contract.source_topic_role,
            latest_source_sample=None,
            update_count=history.update_count,
            maximum_level=history.maximum_level,
            baseline_level=history.baseline.level,
            latest_level=history.latest.level,
            baseline_message=history.baseline.message,
            latest_message=history.latest.message,
            window_sec=window_sec,
            latest_age_sec=latest_age,
            input_rate_hz=delta.input_count / rate_window,
            output_rate_hz=delta.output_count / rate_window,
            baseline=history.baseline.snapshot,
            latest=history.latest.snapshot,
            delta=delta,
        )

    def _evaluate_health(
        self,
        role: str,
        history: _DiagnosticHistory,
        latest_age: float,
        errors: list[str],
    ) -> None:
        if latest_age > self._contract.diagnostic_freshness_sec:
            errors.append(
                f"diagnostic role {role} age {latest_age:.3f}s exceeds "
                f"{self._contract.diagnostic_freshness_sec:.3f}s"
            )
        if history.maximum_level >= _DIAGNOSTIC_ERROR_LEVEL:
            errors.append(
                f"diagnostic role {role} reached ERROR/STALE level "
                f"{history.maximum_level}"
            )

    def _evaluate_metrics(
        self,
        role: str,
        history: _DiagnosticHistory,
        delta: PipelineDiagnosticDelta,
        errors: list[str],
    ) -> None:
        latest = history.latest.snapshot
        if delta.input_count < self._contract.minimum_input_delta:
            errors.append(
                f"diagnostic role {role} input delta {delta.input_count} is below "
                f"{self._contract.minimum_input_delta}"
            )
        if delta.output_count < self._contract.minimum_output_delta:
            errors.append(
                f"diagnostic role {role} output delta {delta.output_count} is below "
                f"{self._contract.minimum_output_delta}"
            )
        if delta.latency_observation_count < self._contract.minimum_latency_samples:
            errors.append(
                f"diagnostic role {role} has {delta.latency_observation_count} new "
                f"latency observations; expected at least "
                f"{self._contract.minimum_latency_samples}"
            )
        if (
            latest.processing_latency_p50_sec is None
            or latest.processing_latency_p95_sec is None
            or latest.processing_latency_max_sec is None
        ):
            errors.append(f"diagnostic role {role} has no latency metrics")
        if latest.last_success_source is None or latest.last_success_age_sec is None:
            errors.append(
                f"diagnostic role {role} has no successful output correlation"
            )
        elif latest.last_success_age_sec > self._contract.diagnostic_freshness_sec:
            errors.append(
                f"diagnostic role {role} last-success age "
                f"{latest.last_success_age_sec:.3f}s exceeds "
                f"{self._contract.diagnostic_freshness_sec:.3f}s"
            )
        if latest.worker_timeout_count != 0:
            errors.append(
                f"diagnostic role {role} reports "
                f"{latest.worker_timeout_count} worker timeouts"
            )
        if latest.worker_restart_count != 0:
            errors.append(
                f"diagnostic role {role} reports "
                f"{latest.worker_restart_count} worker restarts"
            )
        if delta.error_count != 0:
            errors.append(
                f"diagnostic role {role} observed {delta.error_count} processing errors"
            )
        if latest.steady_uptime_sec < history.baseline.snapshot.steady_uptime_sec:
            errors.append(f"diagnostic role {role} steady uptime moved backwards")
        if latest.latency_sample_count < history.baseline.snapshot.latency_sample_count:
            errors.append(f"diagnostic role {role} latency window moved backwards")


def _non_negative_finite(value: float, name: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return number


__all__ = ["DiagnosticEvidenceEvaluation", "DiagnosticEvidenceStore"]
