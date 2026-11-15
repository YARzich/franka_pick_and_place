"""
Strict JSON loader for the versioned runtime validation matrix.

Responsibility:
- Parse JSON into immutable runtime types and reject unknown fields.
- Preserve one explicit schema-version migration boundary.

Does not own:
- Cross-field profile invariants, process execution or runtime artifacts.
"""

from __future__ import annotations

import json
from pathlib import Path
import re
from types import MappingProxyType
from typing import Mapping

from franka_motion_runtime.validation.runtime_json import (
    array_value as _array,
    boolean as _boolean,
    check_keys as _check_keys,
    integer as _integer,
    non_empty_string as _non_empty_string,
    number as _number,
    object_value as _object,
    string_tuple as _string_tuple,
)
from franka_motion_runtime.validation.runtime_types import (
    CommandSpec,
    ConfigError,
    EvidenceKind,
    FaultProfile,
    Platform,
    RUNTIME_MATRIX_SCHEMA_VERSION,
    RuntimeMatrix,
    RuntimeProfile,
)

_ENVIRONMENT_NAME = re.compile(r"^[A-Z][A-Z0-9_]*$")

__all__ = [
    "CommandSpec",
    "ConfigError",
    "EvidenceKind",
    "FaultProfile",
    "Platform",
    "RUNTIME_MATRIX_SCHEMA_VERSION",
    "RuntimeMatrix",
    "RuntimeProfile",
    "load_runtime_matrix",
]


def load_runtime_matrix(path: Path) -> RuntimeMatrix:
    """Load and strictly validate one versioned JSON runtime matrix."""
    try:
        raw_value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ConfigError(f"cannot read runtime matrix {path}: {error}") from error

    root = _object(raw_value, "matrix")
    _check_keys(root, {"schema_version", "matrix_id", "profiles"}, set(), "matrix")
    schema_version = _integer(root["schema_version"], "matrix.schema_version")
    if schema_version != RUNTIME_MATRIX_SCHEMA_VERSION:
        raise ConfigError(
            f"unsupported runtime matrix schema_version={schema_version}; "
            f"expected {RUNTIME_MATRIX_SCHEMA_VERSION}"
        )
    matrix_id = _non_empty_string(root["matrix_id"], "matrix.matrix_id")
    profile_values = _object(root["profiles"], "matrix.profiles")
    if not profile_values:
        raise ConfigError("matrix.profiles must not be empty")

    profiles = {
        profile_id: _parse_profile(profile_id, profile_value)
        for profile_id, profile_value in profile_values.items()
    }
    return RuntimeMatrix(
        matrix_id=matrix_id,
        profiles=MappingProxyType(profiles),
        schema_version=schema_version,
    )


def _parse_profile(profile_id: str, raw_value: object) -> RuntimeProfile:
    """Parse one named profile after validating its exact top-level keys."""
    profile = _object(raw_value, f"profiles.{profile_id}")
    _check_keys(
        profile,
        {
            "platform",
            "evidence_kind",
            "deadline_sec",
            "requirements",
            "environment",
            "preflight",
            "background",
            "readiness",
            "validation",
        },
        {"faults"},
        f"profiles.{profile_id}",
    )
    try:
        platform = Platform(_non_empty_string(profile["platform"], "platform"))
        evidence_kind = EvidenceKind(
            _non_empty_string(profile["evidence_kind"], "evidence_kind")
        )
    except ValueError as error:
        raise ConfigError(f"{profile_id}: unsupported enum value: {error}") from error

    environment = _parse_environment(
        profile["environment"],
        f"{profile_id}.environment",
    )
    faults = _parse_faults(profile, profile_id)

    return RuntimeProfile(
        profile_id=profile_id,
        platform=platform,
        evidence_kind=evidence_kind,
        deadline_sec=_number(profile["deadline_sec"], f"{profile_id}.deadline_sec"),
        requirements=_string_tuple(profile["requirements"], f"{profile_id}.requirements"),
        environment=MappingProxyType(environment),
        preflight=_command_tuple(profile["preflight"], f"{profile_id}.preflight"),
        background=_command_tuple(profile["background"], f"{profile_id}.background"),
        readiness=_command_tuple(profile["readiness"], f"{profile_id}.readiness"),
        validation=_command_tuple(profile["validation"], f"{profile_id}.validation"),
        faults=faults,
    )


def _parse_environment(raw_value: object, path: str) -> dict[str, str]:
    """Parse explicit non-secret environment overrides."""
    environment_value = _object(raw_value, path)
    environment = {}
    for name, value in environment_value.items():
        if not _ENVIRONMENT_NAME.fullmatch(name):
            raise ConfigError(f"{path}: invalid environment name {name!r}")
        environment[name] = _non_empty_string(value, f"{path}.{name}")
    return environment


def _parse_faults(
    profile: Mapping[str, object],
    profile_id: str,
) -> tuple[FaultProfile, ...]:
    """Parse ordered v2 fault transactions without accepting v1 syntax."""
    if "faults" not in profile:
        return ()
    values = _array(profile["faults"], f"{profile_id}.faults")
    if not values:
        raise ConfigError(f"{profile_id}.faults must not be empty")
    return tuple(
        _parse_fault(value, f"{profile_id}.faults[{index}]")
        for index, value in enumerate(values)
    )


def _parse_fault(raw_value: object, path: str) -> FaultProfile:
    """Parse one named transaction whose four phases are command sequences."""
    fault_value = _object(raw_value, path)
    _check_keys(
        fault_value,
        {"fault_id", "inject", "observe", "recover", "verify_recovered"},
        set(),
        path,
    )
    return FaultProfile(
        fault_id=_non_empty_string(fault_value["fault_id"], f"{path}.fault_id"),
        inject=_command_tuple(fault_value["inject"], f"{path}.inject"),
        observe=_command_tuple(fault_value["observe"], f"{path}.observe"),
        recover=_command_tuple(fault_value["recover"], f"{path}.recover"),
        verify_recovered=_command_tuple(
            fault_value["verify_recovered"], f"{path}.verify_recovered"
        ),
    )


def _command_tuple(raw_value: object, path: str) -> tuple[CommandSpec, ...]:
    """Parse an ordered array of command objects."""
    values = _array(raw_value, path)
    return tuple(
        _parse_command(value, f"{path}[{index}]")
        for index, value in enumerate(values)
    )


def _parse_command(raw_value: object, path: str) -> CommandSpec:
    """Parse one strict argv-only command object."""
    command = _object(raw_value, path)
    _check_keys(
        command,
        {"name", "argv", "timeout_sec"},
        {
            "expected_return_codes",
            "stdout_regex",
            "retry_until_pass",
            "retry_interval_sec",
            "repetitions",
        },
        path,
    )
    return CommandSpec(
        name=_non_empty_string(command["name"], f"{path}.name"),
        argv=_string_tuple(command["argv"], f"{path}.argv"),
        timeout_sec=_number(command["timeout_sec"], f"{path}.timeout_sec"),
        expected_return_codes=tuple(
            _integer(value, f"{path}.expected_return_codes")
            for value in _array(
                command.get("expected_return_codes", [0]),
                f"{path}.expected_return_codes",
            )
        ),
        stdout_regex=_string_tuple(
            command.get("stdout_regex", []), f"{path}.stdout_regex"
        ),
        retry_until_pass=_boolean(
            command.get("retry_until_pass", False),
            f"{path}.retry_until_pass",
        ),
        retry_interval_sec=_number(
            command.get("retry_interval_sec", 0.25),
            f"{path}.retry_interval_sec",
        ),
        repetitions=_integer(
            command.get("repetitions", 1),
            f"{path}.repetitions",
        ),
    )
