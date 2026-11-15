"""
Identity and state contract for one deliberately faultable runtime process.

Responsibility:
- Persist the controller/child identities used by a process-death profile.
- Prove exact same-user run-token and role ownership through Linux procfs.

Does not own:
- Child creation, signal policy, CLI output, ROS discovery or fault ordering.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import tempfile
from typing import Mapping

from franka_motion_runtime.validation.runtime_registry import (
    ProcessIdentity,
    RUN_TOKEN_VARIABLE,
    read_start_ticks,
)

CONTROL_DIRECTORY_VARIABLE = "FRANKA_RUNTIME_CONTROL_DIR"
PROCESS_ROLE_VARIABLE = "FRANKA_RUNTIME_PROCESS_ROLE"
ROLE_STATE_SCHEMA_VERSION = 1


class ProcessRoleError(RuntimeError):
    """Report an ambiguous, stale, or unowned role-process identity."""


@dataclass(frozen=True)
class RoleProcessState:
    """Atomic controller state shared with bounded fault commands."""

    role: str
    run_token: str
    controller: ProcessIdentity
    child: ProcessIdentity | None
    generation: int
    status: str
    return_code: int | None


class RoleStateStore:
    """Persist one role state below the harness-owned artifact directory."""

    def __init__(self, control_directory: Path, role: str) -> None:
        """Bind one validated role to its private state file."""
        self._control_directory = control_directory
        self._path = control_directory / f"{role}.state.v1.json"

    @property
    def path(self) -> Path:
        """Return the state path for diagnostics and tests."""
        return self._path

    def write(self, state: RoleProcessState) -> None:
        """Atomically replace state without persisting argv or other environment."""
        self._control_directory.mkdir(parents=True, exist_ok=True, mode=0o700)
        os.chmod(self._control_directory, 0o700)
        value = {
            "schema_version": ROLE_STATE_SCHEMA_VERSION,
            "role": state.role,
            "run_token": state.run_token,
            "controller": _identity_value(state.controller),
            "child": None if state.child is None else _identity_value(state.child),
            "generation": state.generation,
            "status": state.status,
            "return_code": state.return_code,
        }
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{self._path.name}.",
            dir=self._control_directory,
            text=True,
        )
        temporary_path = Path(temporary_name)
        try:
            os.fchmod(descriptor, 0o600)
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                json.dump(value, stream, indent=2, sort_keys=True)
                stream.write("\n")
            os.replace(temporary_path, self._path)
        finally:
            temporary_path.unlink(missing_ok=True)

    def read(self) -> RoleProcessState:
        """Read strict state or fail instead of guessing at partial ownership."""
        try:
            value = json.loads(self._path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise ProcessRoleError(f"cannot read role state {self._path}: {error}") from error
        try:
            if set(value) != {
                "schema_version",
                "role",
                "run_token",
                "controller",
                "child",
                "generation",
                "status",
                "return_code",
            }:
                raise ValueError("unexpected fields")
            if value["schema_version"] != ROLE_STATE_SCHEMA_VERSION:
                raise ValueError("unsupported schema_version")
            role = _non_empty_string(value["role"], "role")
            run_token = _non_empty_string(value["run_token"], "run_token")
            controller = _identity_from_value(value["controller"])
            child = (
                None
                if value["child"] is None
                else _identity_from_value(value["child"])
            )
            generation = _positive_integer(value["generation"], "generation")
            status = value["status"]
            if status not in {"running", "stopped", "terminating"}:
                raise ValueError("invalid status")
            return_code = value["return_code"]
            if return_code is not None and (
                isinstance(return_code, bool) or not isinstance(return_code, int)
            ):
                raise ValueError("invalid return_code")
        except (KeyError, TypeError, ValueError) as error:
            raise ProcessRoleError(f"invalid role state {self._path}: {error}") from error
        return RoleProcessState(
            role=role,
            run_token=run_token,
            controller=controller,
            child=child,
            generation=generation,
            status=status,
            return_code=return_code,
        )


class LinuxRoleProcessInspector:
    """Verify exact process identities and role uniqueness through procfs."""

    def __init__(self, proc_root: Path = Path("/proc")) -> None:
        """Bind inspection to one procfs and the current Unix user."""
        self._proc_root = proc_root
        self._uid = os.getuid()

    def identity(
        self,
        pid: int,
        *,
        run_token: str,
        role: str | None,
    ) -> ProcessIdentity | None:
        """Return identity only when uid, token, and optional role all match."""
        path = self._proc_root / str(pid)
        expected_entries = {f"{RUN_TOKEN_VARIABLE}={run_token}".encode("utf-8")}
        if role is not None:
            expected_entries.add(f"{PROCESS_ROLE_VARIABLE}={role}".encode("utf-8"))
        try:
            if path.stat().st_uid != self._uid:
                return None
            entries = set((path / "environ").read_bytes().split(b"\0"))
            if not expected_entries.issubset(entries):
                return None
            return ProcessIdentity(
                pid=pid,
                process_group_id=os.getpgid(pid),
                start_ticks=read_start_ticks(path / "stat"),
            )
        except (
            FileNotFoundError,
            PermissionError,
            ProcessLookupError,
            ValueError,
        ):
            return None

    def role_processes(
        self,
        *,
        run_token: str,
        role: str,
    ) -> tuple[ProcessIdentity, ...]:
        """Return every same-user process carrying the exact token and role."""
        identities = []
        for path in self._proc_root.iterdir():
            if not path.name.isdigit():
                continue
            identity = self.identity(
                int(path.name),
                run_token=run_token,
                role=role,
            )
            if identity is not None:
                identities.append(identity)
        return tuple(sorted(identities, key=lambda item: item.pid))


def current_process_identity() -> ProcessIdentity:
    """Read the caller identity for a newly written controller state."""
    pid = os.getpid()
    return ProcessIdentity(
        pid=pid,
        process_group_id=os.getpgid(pid),
        start_ticks=read_start_ticks(Path(f"/proc/{pid}/stat")),
    )


def validate_role(role: str) -> str:
    """Accept a compact identifier that maps safely to one state filename."""
    if not role or any(
        not (character.isascii() and (character.isalnum() or character in "_-"))
        for character in role
    ):
        raise ProcessRoleError(f"invalid process role: {role!r}")
    return role


def environment_contract(
    role: str,
    environment: Mapping[str, str] | None = None,
) -> tuple[str, Path]:
    """Resolve the mandatory harness token and private control directory."""
    validate_role(role)
    active_environment = os.environ if environment is None else environment
    run_token = active_environment.get(RUN_TOKEN_VARIABLE, "")
    control_directory = active_environment.get(CONTROL_DIRECTORY_VARIABLE, "")
    if not run_token:
        raise ProcessRoleError(f"{RUN_TOKEN_VARIABLE} is required")
    if not control_directory:
        raise ProcessRoleError(f"{CONTROL_DIRECTORY_VARIABLE} is required")
    return run_token, Path(control_directory)


def _identity_value(identity: ProcessIdentity) -> dict[str, int]:
    """Serialize the minimal Linux identity used to reject PID reuse."""
    return {
        "pid": identity.pid,
        "process_group_id": identity.process_group_id,
        "start_ticks": identity.start_ticks,
    }


def _identity_from_value(value: object) -> ProcessIdentity:
    """Parse one strict positive Linux identity."""
    if not isinstance(value, dict) or set(value) != {
        "pid",
        "process_group_id",
        "start_ticks",
    }:
        raise ValueError("invalid process identity")
    return ProcessIdentity(
        pid=_positive_integer(value["pid"], "pid"),
        process_group_id=_positive_integer(value["process_group_id"], "process_group_id"),
        start_ticks=_positive_integer(value["start_ticks"], "start_ticks"),
    )


def _positive_integer(value: object, field: str) -> int:
    """Reject bool and non-positive identity or generation values."""
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{field} must be a positive integer")
    return value


def _non_empty_string(value: object, field: str) -> str:
    """Return one required string field."""
    if not isinstance(value, str) or not value:
        raise ValueError(f"{field} must be a non-empty string")
    return value
