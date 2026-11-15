"""
Pure coordinator for bounded GPU perception runtime evidence.

Responsibility:
    Convert steady timestamps, delegate stamped-topic and diagnostic state to
    focused stores, and expose one immutable combined pass/fail evaluation.

Does not own:
    ROS subscriptions, GPU commands, file hashing, persistence, or recovery.
"""

from __future__ import annotations

from dataclasses import replace
import math
from typing import Mapping

from manipulation_perception.validation.contracts import GpuPerceptionContract
from manipulation_perception.validation.diagnostic_evidence import (
    DiagnosticEvidenceStore,
)
from manipulation_perception.validation.error_log import BoundedErrorLog
from manipulation_perception.validation.evidence_model import (
    CorrelatedSampleEvidence,
    DiagnosticEvidence,
    DiagnosticObservation,
    EvidenceEvaluation,
    TopicSample,
)
from manipulation_perception.validation.topic_evidence import TopicEvidenceStore


class EvidenceSession:
    """
    Combine two bounded read-only stores under one steady observation window.

    Invalid observations remain visible as evidence errors. The session never
    resets a producer, publishes ROS data, or treats ROS time as wall time.
    """

    def __init__(
        self,
        contract: GpuPerceptionContract,
        *,
        started_monotonic: float | None = None,
    ) -> None:
        self._contract = contract
        self._started_monotonic = (
            None
            if started_monotonic is None
            else _finite(started_monotonic, "started_monotonic")
        )
        self._topics = TopicEvidenceStore(contract)
        self._diagnostics = DiagnosticEvidenceStore(contract)
        self._errors = BoundedErrorLog()

    @property
    def contract(self) -> GpuPerceptionContract:
        """Expose the immutable scenario policy to the thin ROS adapter."""
        return self._contract

    @property
    def observation_started(self) -> bool:
        """Return whether the steady observation window has an established origin."""
        return self._started_monotonic is not None

    def begin_observation(self, started_monotonic: float) -> None:
        """Start timing once the ROS adapter has created all subscriptions."""
        if self._started_monotonic is not None:
            raise RuntimeError("evidence observation has already started")
        self._started_monotonic = _finite(started_monotonic, "started_monotonic")

    def elapsed(self, now_monotonic: float) -> float:
        """Return non-negative steady time since observation began."""
        if self._started_monotonic is None:
            raise RuntimeError("evidence observation has not started")
        return max(
            0.0,
            _finite(now_monotonic, "now_monotonic") - self._started_monotonic,
        )

    def record_error(self, message: str) -> None:
        """Retain one adapter/provenance-independent observation failure."""
        text = str(message).strip()
        if text:
            self._errors.add(text)

    def record_advertised_types(self, role: str, message_types: set[str]) -> None:
        """Store the exact publisher type set observed for a configured topic."""
        self._topics.record_advertised_types(role, message_types)

    def record_topic_sample(
        self,
        *,
        role: str,
        message_type: str,
        frame_id: str,
        stamp_sec: int,
        stamp_nanosec: int,
        received_monotonic: float,
        serialized_size_bytes: int | None,
        serialized_sha256: str | None,
        content_count: int | None,
    ) -> None:
        """Validate and retain one bounded source-message identity."""
        self._topics.record_sample(
            role=role,
            message_type=message_type,
            frame_id=frame_id,
            stamp_sec=stamp_sec,
            stamp_nanosec=stamp_nanosec,
            received_elapsed_sec=self.elapsed(received_monotonic),
            serialized_size_bytes=serialized_size_bytes,
            serialized_sha256=serialized_sha256,
            content_count=content_count,
        )

    def record_diagnostic(
        self,
        *,
        status_name: str,
        level: int,
        message: str,
        values: Mapping[str, str],
        received_monotonic: float,
    ) -> None:
        """Parse one required exact-name DiagnosticStatus into its bounded window."""
        self._diagnostics.record(
            status_name=status_name,
            level=level,
            message=message,
            values=values,
            received_elapsed_sec=self.elapsed(received_monotonic),
        )

    def evaluate(self, *, now_monotonic: float) -> EvidenceEvaluation:
        """Evaluate current bounded state without mutating or recovering producers."""
        elapsed = self.elapsed(now_monotonic)
        errors = list(self._errors.as_tuple())
        if elapsed < self._contract.minimum_observation_sec:
            errors.append(
                "minimum observation window has not elapsed: "
                f"{elapsed:.3f}s < {self._contract.minimum_observation_sec:.3f}s"
            )
        topic_evidence = self._topics.evaluate(elapsed_sec=elapsed)
        diagnostic_evidence = self._diagnostics.evaluate(elapsed_sec=elapsed)
        errors.extend(topic_evidence.errors)
        errors.extend(diagnostic_evidence.errors)
        diagnostics = self._bind_diagnostic_sources(
            diagnostic_evidence.diagnostics,
            errors,
        )
        errors = list(dict.fromkeys(errors))
        return EvidenceEvaluation(
            evidence_status="pass" if not errors else "fail",
            elapsed_sec=elapsed,
            errors=tuple(errors),
            correlated_samples=topic_evidence.correlated_samples,
            terminal_pose=topic_evidence.terminal_pose,
            terminal_pose_age_sec=topic_evidence.terminal_pose_age_sec,
            diagnostics=diagnostics,
        )

    def _bind_diagnostic_sources(
        self,
        diagnostics: tuple[DiagnosticEvidence, ...],
        errors: list[str],
    ) -> tuple[DiagnosticEvidence, ...]:
        """Bind each latest diagnostic success label to an observed valid topic."""
        result = []
        for evidence in diagnostics:
            correlation = evidence.latest.last_success_source
            sample = None
            if correlation is not None:
                sample = self._topics.find_sample(
                    role=evidence.source_topic_role,
                    stamp_sec=correlation.stamp_sec,
                    stamp_nanosec=correlation.stamp_nanosec,
                    frame_id=correlation.frame_id,
                )
                if sample is None:
                    errors.append(
                        f"diagnostic role {evidence.role} source does not match "
                        f"observed topic role {evidence.source_topic_role}"
                    )
            result.append(replace(evidence, latest_source_sample=sample))
        return tuple(result)


def _finite(value: float, name: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


__all__ = [
    "CorrelatedSampleEvidence",
    "DiagnosticEvidence",
    "DiagnosticObservation",
    "EvidenceEvaluation",
    "EvidenceSession",
    "TopicSample",
]
