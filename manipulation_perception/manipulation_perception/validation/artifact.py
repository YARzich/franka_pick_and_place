"""
Deterministic GPU perception evidence artifact model and persistence.

Responsibility:
    Combine pure runtime evaluation with explicit GPU/model provenance and
    atomically write `gpu-perception-evidence.v1.json`.

Does not own:
    ROS observation, inference, runtime cleanup, accuracy scoring, or replay.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import tempfile
from typing import Sequence

from manipulation_perception.validation.contracts import GpuPerceptionContract
from manipulation_perception.validation.evidence import EvidenceEvaluation
from manipulation_perception.validation.provenance import (
    AssetDigest,
    GpuIdentityEvidence,
)


ARTIFACT_FILENAME = "gpu-perception-evidence.v1.json"
ARTIFACT_SCHEMA_VERSION = 1
_REVISION_PATTERN = re.compile(r"^(?:[0-9a-f]{40}|[0-9a-f]{64})$")


@dataclass(frozen=True)
class EvidenceArtifact:
    """Schema-versioned data with deliberately narrow runtime claims."""

    value: dict[str, object]

    @property
    def evidence_status(self) -> str:
        """Return `pass` only when runtime and provenance both succeeded."""
        return str(self.value["evidence_status"])


def build_artifact(
    *,
    contract: GpuPerceptionContract,
    run_token: str,
    revision: str,
    started_at: str,
    finished_at: str,
    evaluation: EvidenceEvaluation,
    gpu_identity: GpuIdentityEvidence | None,
    asset_digests: Sequence[AssetDigest],
    provenance_errors: Sequence[str] = (),
) -> EvidenceArtifact:
    """Build one honest correlation artifact without accuracy/replay claims."""
    token, revision_value = validate_evidence_identity(run_token, revision)
    _required_text(started_at, "started_at")
    _required_text(finished_at, "finished_at")
    errors = list(evaluation.errors)
    errors.extend(str(error) for error in provenance_errors if str(error).strip())
    if gpu_identity is None:
        errors.append("GPU identity evidence is unavailable")
    if len(asset_digests) != len(contract.assets):
        errors.append(
            f"asset provenance is incomplete: {len(asset_digests)} of "
            f"{len(contract.assets)} files"
        )
    errors = list(dict.fromkeys(errors))
    terminal_pose = None
    if evaluation.terminal_pose is not None:
        terminal_pose = {
            **evaluation.terminal_pose.as_dict(),
            "age_sec": evaluation.terminal_pose_age_sec,
        }
    value: dict[str, object] = {
        "schema_version": ARTIFACT_SCHEMA_VERSION,
        "artifact_kind": "gpu_perception_evidence",
        "evidence_status": "pass" if not errors else "fail",
        "run_token": token,
        "revision": {
            "value": revision_value,
            "source": "explicit_argument",
        },
        "contract": {
            "contract_id": contract.contract_id,
            "sha256": contract.sha256,
        },
        "started_at": started_at,
        "finished_at": finished_at,
        "observation_elapsed_sec": evaluation.elapsed_sec,
        "claim_scope": {
            "runtime_correlation": True,
            "accuracy": False,
            "replay": False,
        },
        "gpu_identity": gpu_identity.as_dict() if gpu_identity is not None else None,
        "asset_digests": [digest.as_dict() for digest in asset_digests],
        "correlated_samples": [
            sample.as_dict() for sample in evaluation.correlated_samples
        ],
        "terminal_pose": terminal_pose,
        "diagnostics": [item.as_dict() for item in evaluation.diagnostics],
        "errors": errors,
    }
    return EvidenceArtifact(value=value)


def validate_evidence_identity(run_token: str, revision: str) -> tuple[str, str]:
    """Validate explicit run and source identities before runtime observation."""
    token = _required_text(run_token, "run_token")
    revision_value = _required_text(revision, "revision")
    if _REVISION_PATTERN.fullmatch(revision_value) is None:
        raise ValueError("revision must be a lowercase 40- or 64-character Git object ID")
    return token, revision_value


def write_artifact(directory: Path, artifact: EvidenceArtifact) -> Path:
    """Atomically persist deterministic UTF-8 JSON in an explicit directory."""
    output_directory = Path(directory)
    output_directory.mkdir(parents=True, exist_ok=True)
    target = output_directory / ARTIFACT_FILENAME
    payload = json.dumps(
        artifact.value,
        ensure_ascii=True,
        indent=2,
        sort_keys=True,
    ) + "\n"
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=output_directory,
            prefix=f".{ARTIFACT_FILENAME}.",
            delete=False,
        ) as stream:
            temporary_path = Path(stream.name)
            os.chmod(temporary_path, 0o600)
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, target)
    except OSError:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
        raise
    return target


def utc_now() -> str:
    """Return a timezone-aware UTC timestamp for artifact boundaries."""
    return datetime.now(timezone.utc).isoformat()


def _required_text(value: str, name: str) -> str:
    text = str(value)
    if not text.strip() or any(ord(character) < 32 for character in text):
        raise ValueError(f"{name} must be non-empty and contain no control characters")
    return text


__all__ = [
    "ARTIFACT_FILENAME",
    "ARTIFACT_SCHEMA_VERSION",
    "EvidenceArtifact",
    "build_artifact",
    "utc_now",
    "validate_evidence_identity",
    "write_artifact",
]
