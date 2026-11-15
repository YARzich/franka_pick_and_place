"""
Explicit process identity inspection and run-token registry persistence.

Responsibility:
- Prove process ownership using same-user procfs environment plus run token.
- Persist only the active token and direct-child Linux identities atomically.

Does not own:
- Signal policy, process startup, command logs or runtime evidence.
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
import fcntl
import json
import os
from pathlib import Path
import tempfile
from typing import Iterator, Protocol

from franka_motion_runtime.validation.runtime_deadline import ProcessError

STATE_SCHEMA_VERSION = 1
RUN_TOKEN_VARIABLE = "FRANKA_RUNTIME_RUN_TOKEN"


@dataclass(frozen=True)
class ProcessIdentity:
    """Stable-enough Linux identity used to detect PID reuse."""

    pid: int
    process_group_id: int
    start_ticks: int


class ProcessInspector(Protocol):
    """Read process identities carrying one inherited run token."""

    def owned_processes(self, run_token: str) -> tuple[ProcessIdentity, ...]:
        """Return current same-user processes with the exact run token."""


class LinuxProcessInspector:
    """Inspect same-user process environment and identity through procfs."""

    def __init__(self, proc_root: Path = Path("/proc")) -> None:
        """Bind inspection to one procfs root and the current Unix user."""
        self._proc_root = proc_root
        self._uid = os.getuid()
        self._own_pid = os.getpid()
        self._own_group = os.getpgrp()

    def owned_processes(self, run_token: str) -> tuple[ProcessIdentity, ...]:
        """Return token-bearing processes while excluding the harness itself."""
        expected_entry = f"{RUN_TOKEN_VARIABLE}={run_token}".encode("utf-8")
        identities = []
        for process_path in self._proc_root.iterdir():
            identity = self._owned_identity(process_path, expected_entry)
            if identity is not None:
                identities.append(identity)
        return tuple(identities)

    def _owned_identity(
        self,
        process_path: Path,
        expected_entry: bytes,
    ) -> ProcessIdentity | None:
        """Return a verified identity or ignore a raced/unowned procfs entry."""
        if not process_path.name.isdigit():
            return None
        try:
            if process_path.stat().st_uid != self._uid:
                return None
            pid = int(process_path.name)
            if pid == self._own_pid:
                return None
            environment_entries = (
                process_path / "environ"
            ).read_bytes().split(b"\0")
            if expected_entry not in environment_entries:
                return None
            process_group_id = os.getpgid(pid)
            if process_group_id == self._own_group:
                return None
            start_ticks = read_start_ticks(process_path / "stat")
        except (
            FileNotFoundError,
            PermissionError,
            ProcessLookupError,
            ValueError,
        ):
            return None
        return ProcessIdentity(
            pid=pid,
            process_group_id=process_group_id,
            start_ticks=start_ticks,
        )


class ProcessRegistry:
    """Atomically persist one active run token and its direct child roots."""

    def __init__(self, state_path: Path) -> None:
        """Bind registry operations to one caller-owned state path."""
        self._state_path = state_path

    @property
    def state_path(self) -> Path:
        """Return the configured state-file path."""
        return self._state_path

    def read_token(self) -> str | None:
        """Return the previous run token or fail on malformed state."""
        if not self._state_path.exists():
            return None
        try:
            value = json.loads(self._state_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise ProcessError(
                f"cannot read process registry {self._state_path}: {error}"
            ) from error
        if not _valid_registry_value(value):
            raise ProcessError(f"invalid process registry: {self._state_path}")
        return value["run_token"]

    def claim(
        self,
        run_token: str,
        *,
        adopt_existing: bool = False,
    ) -> None:
        """Claim empty state or explicitly adopt the same registered token."""
        with self._exclusive_lock():
            registered_token = self.read_token()
            if registered_token is not None and (
                not adopt_existing or registered_token != run_token
            ):
                raise ProcessError(
                    "process registry is already active; cleanup is required "
                    "before starting another supervisor"
                )
            if registered_token is None:
                self.write(run_token, ())

    def write(
        self,
        run_token: str,
        roots: tuple[ProcessIdentity, ...],
    ) -> None:
        """Replace state atomically without persisting argv or environment."""
        self._state_path.parent.mkdir(parents=True, exist_ok=True)
        value = {
            "schema_version": STATE_SCHEMA_VERSION,
            "run_token": run_token,
            "roots": [
                {
                    "pid": root.pid,
                    "process_group_id": root.process_group_id,
                    "start_ticks": root.start_ticks,
                }
                for root in roots
            ],
        }
        file_descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{self._state_path.name}.",
            dir=self._state_path.parent,
            text=True,
        )
        temporary_path = Path(temporary_name)
        try:
            os.fchmod(file_descriptor, 0o600)
            with os.fdopen(file_descriptor, "w", encoding="utf-8") as stream:
                json.dump(value, stream, indent=2, sort_keys=True)
                stream.write("\n")
            os.replace(temporary_path, self._state_path)
        finally:
            temporary_path.unlink(missing_ok=True)

    def clear_if_token(self, run_token: str) -> None:
        """Remove state only when it still belongs to the completed run."""
        with self._exclusive_lock():
            if self.read_token() == run_token:
                self._state_path.unlink(missing_ok=True)

    @contextmanager
    def _exclusive_lock(self) -> Iterator[None]:
        """Serialize claim and clear without introducing global state."""
        self._state_path.parent.mkdir(parents=True, exist_ok=True)
        lock_path = self._state_path.with_name(
            f".{self._state_path.name}.lock"
        )
        file_descriptor = os.open(
            lock_path,
            os.O_CREAT | os.O_RDWR,
            0o600,
        )
        with os.fdopen(file_descriptor, "a+", encoding="utf-8") as stream:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


def read_start_ticks(stat_path: Path) -> int:
    """Read Linux procfs field 22 while allowing spaces in process names."""
    stat_value = stat_path.read_text(encoding="utf-8")
    closing_parenthesis = stat_value.rfind(")")
    if closing_parenthesis < 0:
        raise ValueError(f"invalid proc stat: {stat_path}")
    fields_after_name = stat_value[closing_parenthesis + 1:].split()
    return int(fields_after_name[19])


def _valid_registry_value(value: object) -> bool:
    """Validate the minimal state used for ownership recovery."""
    if (
        not isinstance(value, dict)
        or value.get("schema_version") != STATE_SCHEMA_VERSION
        or not isinstance(value.get("run_token"), str)
        or not value["run_token"]
        or not isinstance(value.get("roots"), list)
    ):
        return False
    return all(_valid_root(root) for root in value["roots"])


def _valid_root(value: object) -> bool:
    """Validate one persisted Linux identity without trusting extra fields."""
    if not isinstance(value, dict):
        return False
    if set(value) != {"pid", "process_group_id", "start_ticks"}:
        return False
    return all(
        isinstance(value[field], int)
        and not isinstance(value[field], bool)
        and value[field] > 0
        for field in ("pid", "process_group_id", "start_ticks")
    )
