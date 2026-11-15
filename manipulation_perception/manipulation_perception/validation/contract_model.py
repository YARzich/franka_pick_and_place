"""
Immutable policy model for GPU perception evidence contracts.

Responsibility:
    Define scenario-neutral contract values, cross-field ownership invariants,
    and canonical content identity independently of JSON transport.

Does not own:
    JSON/file parsing, ROS graph observation, provenance collection, or artifacts.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import hashlib
import json
from typing import Any


CONTRACT_SCHEMA_VERSION = 1
GPU_IDENTITY_EXECUTABLE = "/usr/bin/nvidia-smi"
GPU_IDENTITY_QUERY_ARGUMENT = "--query-gpu=name,uuid,driver_version,memory.total"
GPU_IDENTITY_FORMAT_ARGUMENT = "--format=csv,noheader,nounits"
GPU_IDENTITY_FIELDS = ("name", "uuid", "driver_version", "memory_total")
REQUIRED_DIAGNOSTIC_ROLES = frozenset(
    {"stereo", "detector", "pose_backend", "pose_adapter"}
)


class ContractError(ValueError):
    """Report an invalid or ambiguous scenario evidence contract."""


@dataclass(frozen=True)
class TopicContract:
    """Expected ROS type, frame, and optional positive-content rule."""

    role: str
    topic: str
    message_type: str
    frame_id: str
    nonempty_field: str | None
    capture_payload_digest: bool
    maximum_rate_hz: float


@dataclass(frozen=True)
class CorrelationGroupContract:
    """Topic roles which must expose one identical positive ROS timestamp."""

    name: str
    roles: tuple[str, ...]


@dataclass(frozen=True)
class DiagnosticContract:
    """Exact, scenario-owned DiagnosticStatus wire name for one pipeline role."""

    role: str
    status_name: str
    source_topic_role: str


@dataclass(frozen=True)
class AssetContract:
    """Logical model or engine identity requiring an explicit runtime path."""

    name: str


@dataclass(frozen=True)
class GpuIdentityContract:
    """Explicit argv-only nvidia-smi query and its ordered CSV field names."""

    argv: tuple[str, ...]
    fields: tuple[str, ...]
    timeout_sec: float


@dataclass(frozen=True)
class GpuPerceptionContract:
    """Complete robot-independent policy for one bounded evidence observation."""

    contract_id: str
    observation_timeout_sec: float
    minimum_observation_sec: float
    terminal_freshness_sec: float
    diagnostic_freshness_sec: float
    minimum_input_delta: int
    minimum_output_delta: int
    minimum_latency_samples: int
    topics: tuple[TopicContract, ...]
    correlation_groups: tuple[CorrelationGroupContract, ...]
    terminal_pose_role: str
    diagnostics: tuple[DiagnosticContract, ...]
    assets: tuple[AssetContract, ...]
    gpu_identity: GpuIdentityContract

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "GpuPerceptionContract":
        """Parse through the strict transport adapter without coupling the model."""
        from manipulation_perception.validation.contract_io import contract_from_mapping

        return contract_from_mapping(value)

    def validate_relationships(self) -> None:
        """Reject duplicate identities and incomplete evidence ownership."""
        if self.minimum_observation_sec >= self.observation_timeout_sec:
            raise ContractError(
                "minimum_observation_sec must be lower than observation_timeout_sec"
            )
        if not self.topics:
            raise ContractError("topics must not be empty")
        if not self.correlation_groups:
            raise ContractError("correlation_groups must not be empty")
        if not self.assets:
            raise ContractError("assets must not be empty")

        _require_unique((topic.role for topic in self.topics), "topic roles")
        _require_unique((topic.topic for topic in self.topics), "topic names")
        topic_by_role = {topic.role: topic for topic in self.topics}
        terminal = topic_by_role.get(self.terminal_pose_role)
        if terminal is None:
            raise ContractError("terminal_pose_role must reference a topic role")
        if terminal.message_type != "geometry_msgs/msg/PoseStamped":
            raise ContractError(
                "terminal_pose_role must use geometry_msgs/msg/PoseStamped"
            )

        _require_unique(
            (group.name for group in self.correlation_groups), "group names"
        )
        terminal_groups = 0
        grouped_roles: set[str] = set()
        for group in self.correlation_groups:
            missing = sorted(set(group.roles) - topic_by_role.keys())
            if missing:
                raise ContractError(
                    f"correlation group {group.name!r} references unknown roles: "
                    f"{', '.join(missing)}"
                )
            if self.terminal_pose_role in group.roles:
                terminal_groups += 1
            grouped_roles.update(group.roles)
        unreferenced = sorted(topic_by_role.keys() - grouped_roles)
        if unreferenced:
            raise ContractError(
                "every topic role must belong to a correlation group; missing: "
                + ", ".join(unreferenced)
            )
        if terminal_groups != 1:
            raise ContractError(
                "terminal_pose_role must belong to exactly one correlation group"
            )

        _require_unique((item.role for item in self.diagnostics), "diagnostic roles")
        _require_unique(
            (item.status_name for item in self.diagnostics),
            "diagnostic status names",
        )
        diagnostic_roles = {item.role for item in self.diagnostics}
        missing_diagnostics = sorted(REQUIRED_DIAGNOSTIC_ROLES - diagnostic_roles)
        if missing_diagnostics:
            raise ContractError(
                "diagnostics must explicitly cover: "
                + ", ".join(missing_diagnostics)
            )
        unknown_sources = sorted(
            {
                item.source_topic_role
                for item in self.diagnostics
                if item.source_topic_role not in topic_by_role
            }
        )
        if unknown_sources:
            raise ContractError(
                "diagnostic source_topic_role references unknown topic roles: "
                + ", ".join(unknown_sources)
            )
        _require_unique((asset.name for asset in self.assets), "asset names")

    def as_dict(self) -> dict[str, Any]:
        """Return canonical JSON data used for artifact traceability."""
        return {
            "schema_version": CONTRACT_SCHEMA_VERSION,
            "contract_id": self.contract_id,
            "observation_timeout_sec": self.observation_timeout_sec,
            "minimum_observation_sec": self.minimum_observation_sec,
            "terminal_freshness_sec": self.terminal_freshness_sec,
            "diagnostic_freshness_sec": self.diagnostic_freshness_sec,
            "minimum_input_delta": self.minimum_input_delta,
            "minimum_output_delta": self.minimum_output_delta,
            "minimum_latency_samples": self.minimum_latency_samples,
            "topics": [
                {
                    "role": topic.role,
                    "topic": topic.topic,
                    "message_type": topic.message_type,
                    "frame_id": topic.frame_id,
                    "nonempty_field": topic.nonempty_field,
                    "capture_payload_digest": topic.capture_payload_digest,
                    "maximum_rate_hz": topic.maximum_rate_hz,
                }
                for topic in self.topics
            ],
            "correlation_groups": [
                {"name": group.name, "roles": list(group.roles)}
                for group in self.correlation_groups
            ],
            "terminal_pose_role": self.terminal_pose_role,
            "diagnostics": [
                {
                    "role": item.role,
                    "status_name": item.status_name,
                    "source_topic_role": item.source_topic_role,
                }
                for item in self.diagnostics
            ],
            "assets": [{"name": asset.name} for asset in self.assets],
            "gpu_identity": {
                "argv": list(self.gpu_identity.argv),
                "fields": list(self.gpu_identity.fields),
                "timeout_sec": self.gpu_identity.timeout_sec,
            },
        }

    @property
    def sha256(self) -> str:
        """Identify normalized contract content independently of key ordering."""
        payload = json.dumps(
            self.as_dict(), separators=(",", ":"), sort_keys=True
        ).encode("utf-8")
        return hashlib.sha256(payload).hexdigest()


def _require_unique(values: Any, name: str) -> None:
    materialized = tuple(values)
    if len(materialized) != len(set(materialized)):
        raise ContractError(f"{name} must be unique")


__all__ = [
    "CONTRACT_SCHEMA_VERSION",
    "GPU_IDENTITY_EXECUTABLE",
    "GPU_IDENTITY_FIELDS",
    "GPU_IDENTITY_FORMAT_ARGUMENT",
    "GPU_IDENTITY_QUERY_ARGUMENT",
    "REQUIRED_DIAGNOSTIC_ROLES",
    "AssetContract",
    "ContractError",
    "CorrelationGroupContract",
    "DiagnosticContract",
    "GpuIdentityContract",
    "GpuPerceptionContract",
    "TopicContract",
]
