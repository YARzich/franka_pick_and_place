"""
Immutable value objects emitted by GPU perception evidence evaluation.

Responsibility:
    Represent bounded topic samples, exact correlation groups, diagnostic
    windows, and final pass/fail results in artifact-ready form.

Does not own:
    Observation state, validation policy, ROS callbacks, or persistence.
"""

from __future__ import annotations

from dataclasses import dataclass

from manipulation_perception.validation.diagnostics import (
    PipelineDiagnosticDelta,
    PipelineDiagnosticSnapshot,
)


@dataclass(frozen=True)
class TopicSample:
    """Bounded metadata and optional digest for one source-stamped ROS message."""

    role: str
    topic: str
    message_type: str
    frame_id: str
    stamp_sec: int
    stamp_nanosec: int
    received_elapsed_sec: float
    serialized_size_bytes: int | None
    serialized_sha256: str | None
    content_count: int | None

    @property
    def stamp_ns(self) -> int:
        """Return positive ROS timestamp as a correlation key."""
        return self.stamp_sec * 1_000_000_000 + self.stamp_nanosec

    def as_dict(self) -> dict[str, object]:
        """Return artifact metadata; payload bytes are deliberately not retained."""
        return {
            "role": self.role,
            "topic": self.topic,
            "message_type": self.message_type,
            "frame_id": self.frame_id,
            "stamp_sec": self.stamp_sec,
            "stamp_nanosec": self.stamp_nanosec,
            "received_elapsed_sec": self.received_elapsed_sec,
            "serialized_size_bytes": self.serialized_size_bytes,
            "serialized_sha256": self.serialized_sha256,
            "content_count": self.content_count,
        }


@dataclass(frozen=True)
class DiagnosticObservation:
    """One parsed required status received at a steady-clock offset."""

    level: int
    message: str
    received_elapsed_sec: float
    snapshot: PipelineDiagnosticSnapshot


@dataclass(frozen=True)
class CorrelatedSampleEvidence:
    """One exact timestamp observed on every role in a correlation group."""

    group: str
    stamp_sec: int
    stamp_nanosec: int
    topics: tuple[TopicSample, ...]

    def as_dict(self) -> dict[str, object]:
        """Return a traceable sample claim without claiming replayability."""
        return {
            "group": self.group,
            "stamp_sec": self.stamp_sec,
            "stamp_nanosec": self.stamp_nanosec,
            "topics": [topic.as_dict() for topic in self.topics],
        }


@dataclass(frozen=True)
class DiagnosticEvidence:
    """Baseline/latest snapshots and cumulative deltas for one pipeline role."""

    role: str
    status_name: str
    source_topic_role: str
    latest_source_sample: TopicSample | None
    update_count: int
    maximum_level: int
    baseline_level: int
    latest_level: int
    baseline_message: str
    latest_message: str
    window_sec: float
    latest_age_sec: float
    input_rate_hz: float
    output_rate_hz: float
    baseline: PipelineDiagnosticSnapshot
    latest: PipelineDiagnosticSnapshot
    delta: PipelineDiagnosticDelta

    def as_dict(self) -> dict[str, object]:
        """Return complete metrics while keeping operator text non-authoritative."""
        return {
            "role": self.role,
            "status_name": self.status_name,
            "source_topic_role": self.source_topic_role,
            "latest_source_sample": (
                self.latest_source_sample.as_dict()
                if self.latest_source_sample is not None
                else None
            ),
            "update_count": self.update_count,
            "maximum_level": self.maximum_level,
            "baseline_level": self.baseline_level,
            "latest_level": self.latest_level,
            "baseline_message": self.baseline_message,
            "latest_message": self.latest_message,
            "window_sec": self.window_sec,
            "latest_age_sec": self.latest_age_sec,
            "input_rate_hz": self.input_rate_hz,
            "output_rate_hz": self.output_rate_hz,
            "baseline": self.baseline.as_dict(),
            "latest": self.latest.as_dict(),
            "delta": self.delta.as_dict(),
        }


@dataclass(frozen=True)
class EvidenceEvaluation:
    """Immutable pass/fail view used by both the ROS loop and artifact writer."""

    evidence_status: str
    elapsed_sec: float
    errors: tuple[str, ...]
    correlated_samples: tuple[CorrelatedSampleEvidence, ...]
    terminal_pose: TopicSample | None
    terminal_pose_age_sec: float | None
    diagnostics: tuple[DiagnosticEvidence, ...]


__all__ = [
    "CorrelatedSampleEvidence",
    "DiagnosticEvidence",
    "DiagnosticObservation",
    "EvidenceEvaluation",
    "TopicSample",
]
