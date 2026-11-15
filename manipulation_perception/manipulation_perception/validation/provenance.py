"""
Bounded provenance collectors for GPU perception runtime evidence.

Responsibility:
    Hash explicitly listed existing files and execute one explicit argv-only
    nvidia-smi identity query without shell expansion or network activity.

Does not own:
    Asset discovery, downloads, environment installation, or GPU inference.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
import csv
from dataclasses import dataclass
import hashlib
import io
from pathlib import Path
import shutil
import subprocess
from typing import Any

from manipulation_perception.validation.contracts import (
    AssetContract,
    GpuIdentityContract,
)


class ProvenanceError(RuntimeError):
    """Report missing assets or an invalid bounded GPU identity response."""


@dataclass(frozen=True)
class AssetDigest:
    """Identity of one explicitly supplied regular file."""

    name: str
    path: str
    size_bytes: int
    sha256: str

    def as_dict(self) -> dict[str, object]:
        """Return the stable artifact representation."""
        return {
            "name": self.name,
            "path": self.path,
            "size_bytes": self.size_bytes,
            "sha256": self.sha256,
        }


@dataclass(frozen=True)
class GpuIdentityEvidence:
    """Parsed rows and raw-output digest from one nvidia-smi query."""

    command: tuple[str, ...]
    devices: tuple[dict[str, str], ...]
    stdout_sha256: str

    def as_dict(self) -> dict[str, object]:
        """Return argv and identified devices without operator-dependent parsing."""
        return {
            "command": list(self.command),
            "devices": [dict(device) for device in self.devices],
            "stdout_sha256": self.stdout_sha256,
        }


def hash_assets(
    assets: Sequence[AssetContract],
    explicit_paths: Mapping[str, Path],
) -> tuple[AssetDigest, ...]:
    """Hash only the explicit regular files listed by the scenario contract."""
    expected_names = {asset.name for asset in assets}
    supplied_names = set(explicit_paths)
    if supplied_names != expected_names:
        missing = sorted(expected_names - supplied_names)
        unknown = sorted(supplied_names - expected_names)
        details = []
        if missing:
            details.append("missing=" + ",".join(missing))
        if unknown:
            details.append("unknown=" + ",".join(unknown))
        raise ProvenanceError(
            "explicit asset paths do not match contract (" + "; ".join(details) + ")"
        )
    result = []
    resolved_paths: set[Path] = set()
    for asset in assets:
        supplied_path = Path(explicit_paths[asset.name])
        if not supplied_path.is_absolute():
            raise ProvenanceError(
                f"asset {asset.name!r} path must be absolute: {supplied_path}"
            )
        try:
            path = supplied_path.resolve(strict=True)
        except OSError as error:
            raise ProvenanceError(
                f"asset {asset.name!r} does not resolve: {supplied_path}"
            ) from error
        if not path.is_file():
            raise ProvenanceError(
                f"asset {asset.name!r} is not a regular file: {path}"
            )
        if path in resolved_paths:
            raise ProvenanceError(f"asset paths resolve to the same file: {path}")
        resolved_paths.add(path)
        digest = hashlib.sha256()
        try:
            with path.open("rb") as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                    digest.update(chunk)
            size_bytes = path.stat().st_size
        except OSError as error:
            raise ProvenanceError(f"failed to hash asset {path}: {error}") from error
        result.append(
            AssetDigest(
                name=asset.name,
                path=str(path),
                size_bytes=size_bytes,
                sha256=digest.hexdigest(),
            )
        )
    return tuple(result)


def collect_gpu_identity(
    contract: GpuIdentityContract,
    *,
    runner: Callable[..., Any] = subprocess.run,
    executable_resolver: Callable[[str], str | None] = shutil.which,
) -> GpuIdentityEvidence:
    """Execute exactly one direct nvidia-smi argv and parse its CSV rows."""
    executable = executable_resolver(contract.argv[0])
    if executable is None:
        raise ProvenanceError(f"GPU identity executable not found: {contract.argv[0]}")
    if Path(executable) != Path(contract.argv[0]):
        raise ProvenanceError(
            "GPU identity resolver returned an unexpected executable: "
            f"{executable}"
        )
    argv = (executable, *contract.argv[1:])
    try:
        completed = runner(
            argv,
            check=False,
            capture_output=True,
            text=True,
            timeout=contract.timeout_sec,
            stdin=subprocess.DEVNULL,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise ProvenanceError(f"GPU identity query failed: {error}") from error
    if int(completed.returncode) != 0:
        stderr = str(completed.stderr).strip()
        suffix = f": {stderr}" if stderr else ""
        raise ProvenanceError(
            f"GPU identity query returned {completed.returncode}{suffix}"
        )
    stdout = str(completed.stdout)
    devices = _parse_csv_devices(stdout, contract.fields)
    return GpuIdentityEvidence(
        command=argv,
        devices=devices,
        stdout_sha256=hashlib.sha256(stdout.encode("utf-8")).hexdigest(),
    )


def _parse_csv_devices(
    stdout: str,
    fields: Sequence[str],
) -> tuple[dict[str, str], ...]:
    reader = csv.reader(io.StringIO(stdout))
    devices = []
    for row_index, row in enumerate(reader):
        if not row or all(not value.strip() for value in row):
            continue
        values = tuple(value.strip() for value in row)
        if len(values) != len(fields):
            raise ProvenanceError(
                f"GPU identity CSV row {row_index} has {len(values)} fields; "
                f"expected {len(fields)}"
            )
        if any(not value for value in values):
            raise ProvenanceError(f"GPU identity CSV row {row_index} has empty fields")
        devices.append(dict(zip(fields, values, strict=True)))
    if not devices:
        raise ProvenanceError("GPU identity query returned no devices")
    return tuple(devices)


__all__ = [
    "AssetDigest",
    "GpuIdentityEvidence",
    "ProvenanceError",
    "collect_gpu_identity",
    "hash_assets",
]
