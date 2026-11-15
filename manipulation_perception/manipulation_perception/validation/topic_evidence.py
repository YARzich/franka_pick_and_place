"""
Bounded stamped-topic state for GPU perception runtime evidence.

Responsibility:
    Validate topic type/frame/content metadata, retain a rate-derived stamp
    window, form exact correlation groups, and evaluate terminal-pose freshness.

Does not own:
    Diagnostics, ROS callbacks, payload persistence, provenance, or artifacts.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
import math
import re

from manipulation_perception.validation.contracts import GpuPerceptionContract
from manipulation_perception.validation.error_log import BoundedErrorLog
from manipulation_perception.validation.evidence_model import (
    CorrelatedSampleEvidence,
    TopicSample,
)


_SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")


@dataclass(frozen=True)
class TopicEvidenceEvaluation:
    """Current correlated samples, terminal pose, and topic-specific errors."""

    correlated_samples: tuple[CorrelatedSampleEvidence, ...]
    terminal_pose: TopicSample | None
    terminal_pose_age_sec: float | None
    errors: tuple[str, ...]


class TopicEvidenceStore:
    """Retain bounded topic metadata without retaining serialized payloads."""

    def __init__(self, contract: GpuPerceptionContract) -> None:
        self._contract = contract
        self._topics = {topic.role: topic for topic in contract.topics}
        self._samples: dict[str, OrderedDict[int, TopicSample]] = {
            topic.role: OrderedDict() for topic in contract.topics
        }
        self._limits = {
            topic.role: math.ceil(
                topic.maximum_rate_hz * contract.observation_timeout_sec
            ) + 2
            for topic in contract.topics
        }
        self._advertised_types: dict[str, tuple[str, ...]] = {}
        self._errors = BoundedErrorLog()
        self._retention_exhausted: set[str] = set()

    def record_advertised_types(self, role: str, message_types: set[str]) -> None:
        """Store the exact publisher type set observed for a configured topic."""
        self._require_topic(role)
        self._advertised_types[role] = tuple(
            sorted(str(value) for value in message_types)
        )

    def record_sample(
        self,
        *,
        role: str,
        message_type: str,
        frame_id: str,
        stamp_sec: int,
        stamp_nanosec: int,
        received_elapsed_sec: float,
        serialized_size_bytes: int | None,
        serialized_sha256: str | None,
        content_count: int | None,
    ) -> None:
        """Validate and retain one bounded source-message identity."""
        topic = self._require_topic(role)
        elapsed = _non_negative_finite(received_elapsed_sec, "received_elapsed_sec")
        errors = []
        if message_type != topic.message_type:
            errors.append(
                f"topic role {role} received type {message_type!r}; "
                f"expected {topic.message_type!r}"
            )
        if frame_id != topic.frame_id:
            errors.append(
                f"topic role {role} received frame {frame_id!r}; "
                f"expected {topic.frame_id!r}"
            )
        if not 0 <= int(stamp_nanosec) < 1_000_000_000:
            errors.append(f"topic role {role} has invalid stamp nanoseconds")
        stamp_ns = int(stamp_sec) * 1_000_000_000 + int(stamp_nanosec)
        if stamp_ns <= 0:
            errors.append(f"topic role {role} has a non-positive ROS stamp")
        self._validate_digest(
            role,
            topic.capture_payload_digest,
            serialized_size_bytes,
            serialized_sha256,
            errors,
        )
        if topic.nonempty_field is not None:
            if content_count is None:
                errors.append(
                    f"topic role {role} did not expose configured field "
                    f"{topic.nonempty_field!r}"
                )
            elif int(content_count) <= 0:
                errors.append(
                    f"topic role {role} has empty field {topic.nonempty_field!r}"
                )
        if errors:
            self._errors.extend(errors)
            return

        sample = TopicSample(
            role=role,
            topic=topic.topic,
            message_type=message_type,
            frame_id=frame_id,
            stamp_sec=int(stamp_sec),
            stamp_nanosec=int(stamp_nanosec),
            received_elapsed_sec=elapsed,
            serialized_size_bytes=(
                None if serialized_size_bytes is None else int(serialized_size_bytes)
            ),
            serialized_sha256=(
                None if serialized_sha256 is None else str(serialized_sha256)
            ),
            content_count=None if content_count is None else int(content_count),
        )
        samples = self._samples[role]
        previous = samples.get(sample.stamp_ns)
        if (
            previous is not None
            and previous.serialized_sha256 is not None
            and previous.serialized_sha256 != sample.serialized_sha256
        ):
            self._errors.add(
                f"topic role {role} published different payloads with one ROS stamp"
            )
            return
        samples[sample.stamp_ns] = sample
        samples.move_to_end(sample.stamp_ns)
        while len(samples) > self._limits[role]:
            samples.popitem(last=False)
            if role not in self._retention_exhausted:
                self._retention_exhausted.add(role)
                self._errors.add(
                    f"topic role {role} exceeded its declared retention rate budget"
                )

    def evaluate(self, *, elapsed_sec: float) -> TopicEvidenceEvaluation:
        """Evaluate graph types, exact joins, and terminal freshness."""
        elapsed = _non_negative_finite(elapsed_sec, "elapsed_sec")
        errors = list(self._errors.as_tuple())
        self._evaluate_graph_types(errors)
        correlated_samples = self._correlated_samples(errors)
        terminal, terminal_age = self._terminal_pose(
            correlated_samples, elapsed, errors
        )
        return TopicEvidenceEvaluation(
            correlated_samples=tuple(correlated_samples),
            terminal_pose=terminal,
            terminal_pose_age_sec=terminal_age,
            errors=tuple(dict.fromkeys(errors)),
        )

    def find_sample(
        self,
        *,
        role: str,
        stamp_sec: int,
        stamp_nanosec: int,
        frame_id: str,
    ) -> TopicSample | None:
        """Return an observed valid sample matching one diagnostic source label."""
        self._require_topic(role)
        stamp_ns = int(stamp_sec) * 1_000_000_000 + int(stamp_nanosec)
        sample = self._samples[role].get(stamp_ns)
        if sample is None or sample.frame_id != frame_id:
            return None
        return sample

    @staticmethod
    def _validate_digest(
        role: str,
        capture: bool,
        size_bytes: int | None,
        sha256: str | None,
        errors: list[str],
    ) -> None:
        if capture:
            if (
                size_bytes is None
                or isinstance(size_bytes, bool)
                or int(size_bytes) <= 0
            ):
                errors.append(f"topic role {role} has no serialized payload")
            if sha256 is None or _SHA256_PATTERN.fullmatch(str(sha256)) is None:
                errors.append(f"topic role {role} has an invalid serialized SHA256")
        elif size_bytes is not None or sha256 is not None:
            errors.append(
                f"topic role {role} supplied a payload digest although capture is disabled"
            )

    def _require_topic(self, role: str):
        topic = self._topics.get(role)
        if topic is None:
            raise KeyError(f"unconfigured topic role: {role}")
        return topic

    def _evaluate_graph_types(self, errors: list[str]) -> None:
        for topic in self._contract.topics:
            actual = self._advertised_types.get(topic.role, ())
            expected = (topic.message_type,)
            if actual != expected:
                errors.append(
                    f"topic role {topic.role} advertised types {list(actual)!r}; "
                    f"expected exactly {list(expected)!r}"
                )

    def _correlated_samples(
        self,
        errors: list[str],
    ) -> list[CorrelatedSampleEvidence]:
        result = []
        for group in self._contract.correlation_groups:
            stamp_sets = [set(self._samples[role]) for role in group.roles]
            common = set.intersection(*stamp_sets) if stamp_sets else set()
            if not common:
                errors.append(
                    f"correlation group {group.name} has no exact common ROS stamp"
                )
                continue
            stamp_ns = max(
                common,
                key=lambda value: max(
                    self._samples[role][value].received_elapsed_sec
                    for role in group.roles
                ),
            )
            topics = tuple(self._samples[role][stamp_ns] for role in group.roles)
            result.append(
                CorrelatedSampleEvidence(
                    group=group.name,
                    stamp_sec=stamp_ns // 1_000_000_000,
                    stamp_nanosec=stamp_ns % 1_000_000_000,
                    topics=topics,
                )
            )
        return result

    def _terminal_pose(
        self,
        samples: list[CorrelatedSampleEvidence],
        elapsed: float,
        errors: list[str],
    ) -> tuple[TopicSample | None, float | None]:
        candidates = [
            topic
            for sample in samples
            for topic in sample.topics
            if topic.role == self._contract.terminal_pose_role
        ]
        if not candidates:
            errors.append("no correlated terminal PoseStamped was observed")
            return None, None
        terminal = max(candidates, key=lambda sample: sample.received_elapsed_sec)
        age = max(0.0, elapsed - terminal.received_elapsed_sec)
        if age > self._contract.terminal_freshness_sec:
            errors.append(
                f"terminal pose age {age:.3f}s exceeds "
                f"{self._contract.terminal_freshness_sec:.3f}s"
            )
        return terminal, age


def _non_negative_finite(value: float, name: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return number


__all__ = ["TopicEvidenceEvaluation", "TopicEvidenceStore"]
