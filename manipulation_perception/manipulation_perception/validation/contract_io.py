"""
Strict JSON/file transport for GPU perception evidence contracts.

Responsibility:
    Decode explicit JSON values into the immutable contract model and reject
    unknown keys, invalid scalar syntax, unsafe GPU argv, and implicit paths.

Does not own:
    Cross-field ownership rules, ROS observation, command execution, or artifacts.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import json
import math
from pathlib import Path
import re
from typing import Any

from manipulation_perception.common.config_validation import is_valid_frame_id
from manipulation_perception.validation.contract_model import (
    AssetContract,
    CONTRACT_SCHEMA_VERSION,
    ContractError,
    CorrelationGroupContract,
    DiagnosticContract,
    GPU_IDENTITY_EXECUTABLE,
    GPU_IDENTITY_FIELDS,
    GPU_IDENTITY_FORMAT_ARGUMENT,
    GPU_IDENTITY_QUERY_ARGUMENT,
    GpuIdentityContract,
    GpuPerceptionContract,
    TopicContract,
)


_IDENTIFIER_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_MESSAGE_TYPE_PATTERN = re.compile(
    r"^[A-Za-z][A-Za-z0-9_]*/msg/[A-Za-z][A-Za-z0-9_]*$"
)


def contract_from_mapping(value: Mapping[str, Any]) -> GpuPerceptionContract:
    """Build a fail-closed contract and reject unknown or implicit values."""
    data = _mapping(value, "contract")
    _require_exact_keys(
        data,
        {
            "schema_version",
            "contract_id",
            "observation_timeout_sec",
            "minimum_observation_sec",
            "terminal_freshness_sec",
            "diagnostic_freshness_sec",
            "minimum_input_delta",
            "minimum_output_delta",
            "minimum_latency_samples",
            "topics",
            "correlation_groups",
            "terminal_pose_role",
            "diagnostics",
            "assets",
            "gpu_identity",
        },
        "contract",
    )
    if _integer(data["schema_version"], "schema_version") != CONTRACT_SCHEMA_VERSION:
        raise ContractError(f"schema_version must be {CONTRACT_SCHEMA_VERSION}")
    contract = GpuPerceptionContract(
        contract_id=_identifier(data["contract_id"], "contract_id"),
        observation_timeout_sec=_positive_float(
            data["observation_timeout_sec"], "observation_timeout_sec"
        ),
        minimum_observation_sec=_positive_float(
            data["minimum_observation_sec"], "minimum_observation_sec"
        ),
        terminal_freshness_sec=_positive_float(
            data["terminal_freshness_sec"], "terminal_freshness_sec"
        ),
        diagnostic_freshness_sec=_positive_float(
            data["diagnostic_freshness_sec"], "diagnostic_freshness_sec"
        ),
        minimum_input_delta=_positive_integer(
            data["minimum_input_delta"], "minimum_input_delta"
        ),
        minimum_output_delta=_positive_integer(
            data["minimum_output_delta"], "minimum_output_delta"
        ),
        minimum_latency_samples=_positive_integer(
            data["minimum_latency_samples"], "minimum_latency_samples"
        ),
        topics=tuple(
            _parse_topic(item, index)
            for index, item in enumerate(_sequence(data["topics"], "topics"))
        ),
        correlation_groups=tuple(
            _parse_group(item, index)
            for index, item in enumerate(
                _sequence(data["correlation_groups"], "correlation_groups")
            )
        ),
        terminal_pose_role=_identifier(
            data["terminal_pose_role"], "terminal_pose_role"
        ),
        diagnostics=tuple(
            _parse_diagnostic(item, index)
            for index, item in enumerate(
                _sequence(data["diagnostics"], "diagnostics")
            )
        ),
        assets=tuple(
            _parse_asset(item, index)
            for index, item in enumerate(_sequence(data["assets"], "assets"))
        ),
        gpu_identity=_parse_gpu_identity(data["gpu_identity"]),
    )
    contract.validate_relationships()
    return contract


def load_contract(path: Path) -> GpuPerceptionContract:
    """Load one UTF-8 JSON contract from an explicitly supplied file."""
    source = Path(path)
    if not source.is_file():
        raise ContractError(f"contract file does not exist: {source}")
    try:
        value = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ContractError(f"failed to read contract {source}: {error}") from error
    return contract_from_mapping(_mapping(value, "contract"))


def _parse_topic(value: object, index: int) -> TopicContract:
    data = _mapping(value, f"topics[{index}]")
    _require_exact_keys(
        data,
        {
            "role",
            "topic",
            "message_type",
            "frame_id",
            "nonempty_field",
            "capture_payload_digest",
            "maximum_rate_hz",
        },
        f"topics[{index}]",
    )
    topic = _text(data["topic"], f"topics[{index}].topic")
    if not topic.startswith("/") or topic == "/" or "//" in topic:
        raise ContractError(f"topics[{index}].topic must be an absolute ROS topic")
    message_type = _text(data["message_type"], f"topics[{index}].message_type")
    if _MESSAGE_TYPE_PATTERN.fullmatch(message_type) is None:
        raise ContractError(
            f"topics[{index}].message_type must use package/msg/Type syntax"
        )
    frame_id = _text(data["frame_id"], f"topics[{index}].frame_id")
    if not is_valid_frame_id(frame_id):
        raise ContractError(
            f"topics[{index}].frame_id must be a canonical relative frame"
        )
    nonempty_field = None
    if data["nonempty_field"] is not None:
        nonempty_field = _identifier(
            data["nonempty_field"], f"topics[{index}].nonempty_field"
        )
    return TopicContract(
        role=_identifier(data["role"], f"topics[{index}].role"),
        topic=topic,
        message_type=message_type,
        frame_id=frame_id,
        nonempty_field=nonempty_field,
        capture_payload_digest=_boolean(
            data["capture_payload_digest"],
            f"topics[{index}].capture_payload_digest",
        ),
        maximum_rate_hz=_positive_float(
            data["maximum_rate_hz"], f"topics[{index}].maximum_rate_hz"
        ),
    )


def _parse_group(value: object, index: int) -> CorrelationGroupContract:
    data = _mapping(value, f"correlation_groups[{index}]")
    _require_exact_keys(data, {"name", "roles"}, f"correlation_groups[{index}]")
    roles = tuple(
        _identifier(item, f"correlation_groups[{index}].roles[{role_index}]")
        for role_index, item in enumerate(
            _sequence(data["roles"], f"correlation_groups[{index}].roles")
        )
    )
    if len(roles) < 2:
        raise ContractError(
            f"correlation_groups[{index}].roles needs at least two roles"
        )
    _require_unique(roles, f"correlation_groups[{index}] roles")
    return CorrelationGroupContract(
        name=_identifier(data["name"], f"correlation_groups[{index}].name"),
        roles=roles,
    )


def _parse_diagnostic(value: object, index: int) -> DiagnosticContract:
    data = _mapping(value, f"diagnostics[{index}]")
    _require_exact_keys(
        data,
        {"role", "status_name", "source_topic_role"},
        f"diagnostics[{index}]",
    )
    return DiagnosticContract(
        role=_identifier(data["role"], f"diagnostics[{index}].role"),
        status_name=_text(data["status_name"], f"diagnostics[{index}].status_name"),
        source_topic_role=_identifier(
            data["source_topic_role"],
            f"diagnostics[{index}].source_topic_role",
        ),
    )


def _parse_asset(value: object, index: int) -> AssetContract:
    data = _mapping(value, f"assets[{index}]")
    _require_exact_keys(data, {"name"}, f"assets[{index}]")
    return AssetContract(name=_identifier(data["name"], f"assets[{index}].name"))


def _parse_gpu_identity(value: object) -> GpuIdentityContract:
    data = _mapping(value, "gpu_identity")
    _require_exact_keys(data, {"argv", "fields", "timeout_sec"}, "gpu_identity")
    argv = tuple(
        _text(item, f"gpu_identity.argv[{index}]")
        for index, item in enumerate(_sequence(data["argv"], "gpu_identity.argv"))
    )
    if not argv or argv[0] != GPU_IDENTITY_EXECUTABLE:
        raise ContractError(
            f"gpu_identity.argv[0] must be {GPU_IDENTITY_EXECUTABLE!r}"
        )
    if argv[1:] != (GPU_IDENTITY_QUERY_ARGUMENT, GPU_IDENTITY_FORMAT_ARGUMENT):
        raise ContractError(
            "gpu_identity.argv permits only the fixed read-only GPU identity query"
        )
    fields = tuple(
        _identifier(item, f"gpu_identity.fields[{index}]")
        for index, item in enumerate(
            _sequence(data["fields"], "gpu_identity.fields")
        )
    )
    _require_unique(fields, "gpu identity fields")
    if fields != GPU_IDENTITY_FIELDS:
        raise ContractError(
            "gpu_identity.fields must match the fixed identity query fields"
        )
    return GpuIdentityContract(
        argv=argv,
        fields=fields,
        timeout_sec=_positive_float(data["timeout_sec"], "gpu_identity.timeout_sec"),
    )


def _mapping(value: object, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
        raise ContractError(f"{name} must be a JSON object with string keys")
    return value


def _sequence(value: object, name: str) -> Sequence[Any]:
    if isinstance(value, (str, bytes, bytearray)) or not isinstance(value, Sequence):
        raise ContractError(f"{name} must be a JSON array")
    return value


def _require_exact_keys(data: Mapping[str, Any], expected: set[str], name: str) -> None:
    actual = set(data)
    missing = sorted(expected - actual)
    unknown = sorted(actual - expected)
    if missing or unknown:
        details = []
        if missing:
            details.append("missing=" + ",".join(missing))
        if unknown:
            details.append("unknown=" + ",".join(unknown))
        raise ContractError(f"{name} has invalid keys ({'; '.join(details)})")


def _text(value: object, name: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ContractError(f"{name} must be a non-empty trimmed string")
    if any(ord(character) < 32 for character in value):
        raise ContractError(f"{name} must not contain control characters")
    return value


def _identifier(value: object, name: str) -> str:
    text = _text(value, name)
    if _IDENTIFIER_PATTERN.fullmatch(text) is None:
        raise ContractError(
            f"{name} must contain only letters, digits, '.', '-', or '_'"
        )
    return text


def _integer(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ContractError(f"{name} must be an integer")
    return value


def _boolean(value: object, name: str) -> bool:
    if not isinstance(value, bool):
        raise ContractError(f"{name} must be a boolean")
    return value


def _positive_integer(value: object, name: str) -> int:
    number = _integer(value, name)
    if number <= 0:
        raise ContractError(f"{name} must be greater than zero")
    return number


def _positive_float(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ContractError(f"{name} must be numeric")
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ContractError(f"{name} must be finite and greater than zero")
    return number


def _require_unique(values: Sequence[str], name: str) -> None:
    materialized = tuple(values)
    if len(materialized) != len(set(materialized)):
        raise ContractError(f"{name} must be unique")


__all__ = ["contract_from_mapping", "load_contract"]
