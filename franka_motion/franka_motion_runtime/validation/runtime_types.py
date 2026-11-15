"""
Immutable types and cross-field invariants for runtime validation.

Responsibility:
- Define immutable runtime values and their cross-field invariants.

Does not own:
- JSON parsing, process execution, retry timing or artifact persistence.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import re
from typing import Mapping

from franka_motion_runtime.validation.runtime_budget import profile_foreground_budget
from franka_motion_runtime.validation.runtime_matrix_model import (
    ConfigError,
    RuntimeMatrix as RuntimeMatrix,
)

RUNTIME_MATRIX_SCHEMA_VERSION = 3
_ENVIRONMENT_TOKEN = re.compile(r"\$\{([A-Z][A-Z0-9_]*)\}")
_IDENTIFIER = re.compile(r"[A-Za-z0-9_-]+")

__all__ = [
    "CommandSpec",
    "ConfigError",
    "EvidenceKind",
    "FaultProfile",
    "Platform",
    "RUNTIME_MATRIX_SCHEMA_VERSION",
    "RuntimeMatrix",
    "RuntimeProfile",
    "require_positive_number",
]


class Platform(str, Enum):
    """Execution environments represented by the runtime matrix."""

    GAZEBO = "gazebo"
    ISAAC = "isaac"


class EvidenceKind(str, Enum):
    """Evidence a completed profile is allowed to claim."""

    SIMULATOR_MOTION = "simulator_motion"
    DIAGNOSTIC_FAULT = "diagnostic_fault"
    SIMULATOR_SOAK = "simulator_soak"
    GPU_PERCEPTION = "gpu_perception"
    DETECTED_ROUND_TRIP = "detected_round_trip"


@dataclass(frozen=True)
class CommandSpec:
    """One argv-only command with bounded execution and output contracts."""

    name: str
    argv: tuple[str, ...]
    timeout_sec: float
    expected_return_codes: tuple[int, ...] = (0,)
    stdout_regex: tuple[str, ...] = ()
    retry_until_pass: bool = False
    retry_interval_sec: float = 0.25
    repetitions: int = 1

    def __post_init__(self) -> None:
        """Reject ambiguous commands before any process can be created."""
        if _IDENTIFIER.fullmatch(self.name) is None:
            raise ConfigError(f"invalid command name: {self.name!r}")
        if not self.argv or any(
            not isinstance(argument, str) or not argument
            for argument in self.argv
        ):
            raise ConfigError(f"{self.name}: argv must contain non-empty strings")
        require_positive_number(self.timeout_sec, f"{self.name}.timeout_sec")
        if not self.expected_return_codes:
            raise ConfigError(f"{self.name}: expected_return_codes must not be empty")
        invalid_return_codes = any(
            isinstance(code, bool) or not isinstance(code, int)
            for code in self.expected_return_codes
        )
        if invalid_return_codes:
            raise ConfigError(f"{self.name}: expected_return_codes must be integers")
        if self.repetitions < 1:
            raise ConfigError(f"{self.name}: repetitions must be positive")
        require_positive_number(
            self.retry_interval_sec,
            f"{self.name}.retry_interval_sec",
        )
        for expression in self.stdout_regex:
            try:
                re.compile(expression)
            except re.error as error:
                raise ConfigError(
                    f"{self.name}: invalid stdout_regex {expression!r}"
                ) from error

    def expanded_argv(self, environment: Mapping[str, str]) -> tuple[str, ...]:
        """Expand explicit environment tokens while preserving argv boundaries."""

        def replace_token(match: re.Match[str]) -> str:
            """Resolve one required token without invoking a shell."""
            variable_name = match.group(1)
            try:
                return environment[variable_name]
            except KeyError as error:
                raise ConfigError(
                    f"{self.name}: required environment variable "
                    f"{variable_name!r} is unavailable"
                ) from error

        expanded = tuple(
            _ENVIRONMENT_TOKEN.sub(replace_token, argument)
            for argument in self.argv
        )
        if any(not argument for argument in expanded):
            raise ConfigError(
                f"{self.name}: environment expansion produced an empty argument"
            )
        return expanded


@dataclass(frozen=True)
class FaultProfile:
    """One named, ordered and independently recoverable fault transaction."""

    fault_id: str
    inject: tuple[CommandSpec, ...]
    observe: tuple[CommandSpec, ...]
    recover: tuple[CommandSpec, ...]
    verify_recovered: tuple[CommandSpec, ...]

    def __post_init__(self) -> None:
        """Require every transaction phase to be explicit and auditable."""
        if _IDENTIFIER.fullmatch(self.fault_id) is None:
            raise ConfigError(f"invalid fault_id: {self.fault_id!r}")
        for phase in ("inject", "observe", "recover", "verify_recovered"):
            if not getattr(self, phase):
                raise ConfigError(f"{self.fault_id}.{phase} must not be empty")


@dataclass(frozen=True)
class RuntimeProfile:
    """Complete bounded runtime scenario selected by a workflow job."""

    profile_id: str
    platform: Platform
    evidence_kind: EvidenceKind
    deadline_sec: float
    requirements: tuple[str, ...]
    environment: Mapping[str, str]
    preflight: tuple[CommandSpec, ...]
    background: tuple[CommandSpec, ...]
    readiness: tuple[CommandSpec, ...]
    validation: tuple[CommandSpec, ...]
    faults: tuple[FaultProfile, ...]

    def __post_init__(self) -> None:
        """Enforce evidence-specific stages and bounded execution."""
        require_positive_number(
            self.deadline_sec,
            f"{self.profile_id}.deadline_sec",
        )
        if not self.requirements:
            raise ConfigError(f"{self.profile_id}: requirements must not be empty")
        if not self.preflight:
            raise ConfigError(f"{self.profile_id}: preflight commands are required")
        if not self.readiness:
            raise ConfigError(f"{self.profile_id}: readiness probes are required")
        if any(
            command.timeout_sec < self.deadline_sec
            for command in self.background
        ):
            raise ConfigError(
                f"{self.profile_id}: background command timeout must cover "
                "the complete profile deadline"
            )
        self._validate_unique_command_names()
        self._validate_evidence_stages()
        foreground_budget = profile_foreground_budget(self)
        if foreground_budget > self.deadline_sec:
            raise ConfigError(
                f"{self.profile_id}: deadline must cover configured foreground "
                f"budget {foreground_budget:.3f}s"
            )

    def _validate_unique_command_names(self) -> None:
        """Keep artifact names and stage audit records unambiguous."""
        commands = list(self.preflight)
        commands.extend(self.background)
        commands.extend(self.readiness)
        commands.extend(self.validation)
        for fault in self.faults:
            commands.extend(fault.inject)
            commands.extend(fault.observe)
            commands.extend(fault.recover)
            commands.extend(fault.verify_recovered)
        command_names = [command.name for command in commands]
        if len(command_names) != len(set(command_names)):
            raise ConfigError(f"{self.profile_id}: command names must be unique")
        fault_ids = [fault.fault_id for fault in self.faults]
        if len(fault_ids) != len(set(fault_ids)):
            raise ConfigError(f"{self.profile_id}: fault IDs must be unique")

    def _validate_evidence_stages(self) -> None:
        """Prevent prerequisite-only profiles from claiming runtime evidence."""
        if self.evidence_kind is EvidenceKind.DIAGNOSTIC_FAULT:
            if not self.faults:
                raise ConfigError(f"{self.profile_id}: fault contracts are required")
            if not self.validation:
                raise ConfigError(
                    f"{self.profile_id}: post-recovery validation is required"
                )
        elif self.faults:
            raise ConfigError(
                f"{self.profile_id}: faults are valid only for diagnostic_fault"
            )

        motion_evidence = {
            EvidenceKind.SIMULATOR_MOTION,
            EvidenceKind.SIMULATOR_SOAK,
        }
        if self.evidence_kind in motion_evidence and not self.validation:
            raise ConfigError(
                f"{self.profile_id}: motion/soak evidence requires validation"
            )
        if (
            self.evidence_kind is EvidenceKind.GPU_PERCEPTION
            and not self.validation
        ):
            raise ConfigError(
                f"{self.profile_id}: GPU perception evidence requires validation"
            )
        if (
            self.evidence_kind is EvidenceKind.DETECTED_ROUND_TRIP
            and not self.validation
        ):
            raise ConfigError(
                f"{self.profile_id}: detected round trip evidence requires validation"
            )
        if self.evidence_kind is EvidenceKind.SIMULATOR_SOAK and not any(
            command.repetitions > 1 for command in self.validation
        ):
            raise ConfigError(
                f"{self.profile_id}: soak requires a repeated validation command"
            )


def require_positive_number(value: float, path: str) -> None:
    """Require finite positive durations at immutable type boundaries."""
    if not math.isfinite(value) or value <= 0.0:
        raise ConfigError(f"{path} must be finite and positive")
